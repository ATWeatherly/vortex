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

# xsdb script: JTAG-boot Linux on a board with no usable boot medium
# (e.g. Zybo Z7-20 whose QSPI holds only a factory demo). No FSBL and no
# SD card: ps7_init does the PS bring-up, the PL is programmed directly,
# U-Boot runs from DDR, and the kernel FIT + any payload blobs are already
# staged in DDR when U-Boot's prompt appears (drive it over UART:
# "bootm <fit_addr>").
#
# Usage:
#   xsdb jtag_boot_linux.tcl <bit> <ps7_init.tcl> <u-boot.elf> [file@hexaddr ...]

if { $::argc < 3 } { puts "usage: xsdb jtag_boot_linux.tcl <bit> <ps7_init> <u-boot.elf> \[file@addr ...\]"; exit 1 }
set bit     [lindex $::argv 0]
set ps7init [lindex $::argv 1]
set uboot   [lindex $::argv 2]
set blobs   [lrange $::argv 3 end]

foreach {f} [list $bit $ps7init $uboot] {
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

# Halt CPU1; flush + disable PL310 L2; clear any leftover MMU state
# (see run.tcl / run_bm.tcl for the full story).
targets -set -filter {name =~ "*Cortex-A9*#1"}
catch {stop}
targets -set -filter {name =~ "*Cortex-A9*#0"}
catch {stop}
mwr 0xF8F027FC 0xFF
while {[mrd -value 0xF8F027FC 1] != 0} {}
mwr 0xF8F02730 0
while {[mrd -value 0xF8F02730 1] != 0} {}
mwr 0xF8F02100 0
catch {rst -processor}
after 100
catch {stop}
puts "PS initialized, L2 off, CPU0 clean"

foreach spec $blobs {
    set at [split $spec @]
    set f  [lindex $at 0]
    set a  [lindex $at 1]
    if {![file exists $f]} { puts "ERROR: missing blob $f"; exit 1 }
    puts "staging [file tail $f] ([file size $f] B) @ $a"
    dow -data $f $a
}

dow $uboot
con
puts "U-Boot running — drive it over the UART"
exit
