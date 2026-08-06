# Copyright © 2019-2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# xsdb (xsct) script: program the bitstream, init the PS, load the host app + kernel
# image over JTAG, and run. Watch the UART at 115200 in another terminal.
#
# Usage (from the zynq *build* directory, after `make build` and
# `vitis -s host/build_app.py`):
#   xsdb <source-dir>/host/run.tcl [bit] [ps7_init] [elf] [kernel_bin] [kernel_addr]

set bit        [expr {$::argc > 0 ? [lindex $::argv 0] : "vortex_z7.bit"}]
set ps7init    [expr {$::argc > 1 ? [lindex $::argv 1] : "ws/ps7_init.tcl"}]
set elf        [expr {$::argc > 2 ? [lindex $::argv 2] : "ws/vortex_host/build/vortex_host.elf"}]
set kernel     [expr {$::argc > 3 ? [lindex $::argv 3] : ""}]
set kernaddr   [expr {$::argc > 4 ? [lindex $::argv 4] : 0x10000000}]

foreach {f} [list $bit $ps7init $elf] {
    if {![file exists $f]} { puts "ERROR: missing $f"; exit 1 }
}

connect

# reset the whole PS and program the PL
targets -set -filter {name =~ "APU*"}
rst -system
after 500

fpga -file $bit

source $ps7init
ps7_init
ps7_post_config

# Halt CPU1 so nothing else allocates into the shared L2 while we run.
targets -set -filter {name =~ "*Cortex-A9*#1"}
catch {stop}

# Flush + disable the PL310 L2 cache: the SD-boot U-Boot leaves it enabled
# and full of valid/dirty lines over DDR (its stack/heap sit at the top of
# DRAM — exactly our COUT/IO window). Stale hits there would mask Vortex's
# HP-port writes from the CPU, and dirty evictions would clobber DRAM.
targets -set -filter {name =~ "*Cortex-A9*#0"}
catch {stop}
mwr 0xF8F027FC 0xFF                        ;# clean+invalidate all 8 ways
while {[mrd -value 0xF8F027FC 1] != 0} {}
mwr 0xF8F02730 0                           ;# cache sync
while {[mrd -value 0xF8F02730 1] != 0} {}
mwr 0xF8F02100 0                           ;# disable L2
puts "L2 flushed and disabled"

# load the host app and the kernel image on CPU0 (start.S also forces
# MMU/caches off, so accesses land in physical DDR)
dow $elf
if {$kernel ne ""} {
    if {![file exists $kernel]} { puts "ERROR: missing $kernel"; exit 1 }
    # Zero a 2 MB window first: the flat image carries no BSS, and vx_start
    # does not clear it (the vxbin runtime loader normally does). Without
    # this, the kernel's globals/heap inherit stale DRAM garbage.
    puts "zeroing kernel window @ [format 0x%08x $kernaddr]"
    mwr $kernaddr 0 0x80000
    puts "loading kernel image $kernel @ [format 0x%08x $kernaddr]"
    dow -data $kernel $kernaddr
}

con
puts "running — watch the UART (115200) for output"
exit
