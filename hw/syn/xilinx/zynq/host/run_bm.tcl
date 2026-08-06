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

# xsdb script: program the PL, init the PS, stage arbitrary data blobs into
# DDR, load a bare-metal host ELF, and run. Generalization of run.tcl for
# hosts that consume JTAG-preloaded files (see host/baremetal/rt).
#
# Usage:
#   xsdb run_bm.tcl <bit> <ps7_init.tcl> <host.elf> [file@hexaddr ...]
# e.g.
#   xsdb run_bm.tcl vortex_z7.bit ws/ps7_init.tcl llama2_260k.elf \
#        data/stories260K.bin@0x26000000 data/tok512.bin@0x27000000 \
#        kernel_hf/kernel.vxbin@0x27800000

if { $::argc < 3 } { puts "usage: xsdb run_bm.tcl <bit> <ps7_init> <elf> \[file@addr ...\]"; exit 1 }
set bit     [lindex $::argv 0]
set ps7init [lindex $::argv 1]
set elf     [lindex $::argv 2]
set blobs   [lrange $::argv 3 end]

foreach {f} [list $bit $ps7init $elf] {
    if {![file exists $f]} { puts "ERROR: missing $f"; exit 1 }
}

connect

targets -set -filter {name =~ "APU*"}
rst -system
after 500

fpga -file $bit

source $ps7init
ps7_init
ps7_post_config

# Halt CPU1; flush + disable PL310 L2 (see run.tcl for the full story).
targets -set -filter {name =~ "*Cortex-A9*#1"}
catch {stop}
targets -set -filter {name =~ "*Cortex-A9*#0"}
catch {stop}
mwr 0xF8F027FC 0xFF
while {[mrd -value 0xF8F027FC 1] != 0} {}
mwr 0xF8F02730 0
while {[mrd -value 0xF8F02730 1] != 0} {}
mwr 0xF8F02100 0
puts "L2 flushed and disabled"

# The previous occupant (e.g. the Zybo QSPI factory demo) may leave an MMU
# table that does not flat-map all of DDR — dow would then hit section
# translation faults. A processor reset clears SCTLR (MMU/caches off) so
# all following loads see physical DDR.
catch {rst -processor}
after 100
catch {stop}

# Stage the blobs (CPU stopped: physical DDR writes, no MMU in the way).
foreach spec $blobs {
    set at [split $spec @]
    set f  [lindex $at 0]
    set a  [lindex $at 1]
    if {![file exists $f]} { puts "ERROR: missing blob $f"; exit 1 }
    puts "loading [file tail $f] ([file size $f] B) @ $a"
    dow -data $f $a
}

dow $elf
con
puts "running — watch the UART (115200) for output"
exit
