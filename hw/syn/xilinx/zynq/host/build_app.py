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

# Vitis 2025.x unified-CLI build of the bare-metal Vortex host app.
# Run from this directory (with the bitstream built one level up):
#   vitis -s build_app.py [<path-to-xsa>]
# Produces ws/vortex_host/build/vortex_host.elf and extracts ps7_init.tcl
# into the workspace root for run.tcl.

import os
import shutil
import sys
import glob

import vitis

script_dir = os.path.dirname(os.path.abspath(__file__))
xsa = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.getcwd(), "..", "vortex_z7.xsa")
xsa = os.path.abspath(xsa)
if not os.path.isfile(xsa):
    raise SystemExit(f"XSA not found: {xsa} (build the bitstream first)")

ws = os.path.abspath("./ws")
client = vitis.create_client(workspace=ws)

# platform (standalone on Cortex-A9 core 0)
try:
    plat = client.get_component(name="vortex_z7_plat")
    print("reusing existing platform component")
except Exception:
    plat = client.create_platform_component(
        name="vortex_z7_plat",
        hw_design=xsa,
        os="standalone",
        cpu="ps7_cortexa9_0",
    )
plat.build()

xpfms = glob.glob(os.path.join(ws, "vortex_z7_plat", "export", "**", "*.xpfm"), recursive=True)
if not xpfms:
    raise SystemExit("platform build produced no .xpfm")
xpfm = xpfms[0]
print(f"platform: {xpfm}")

# app
try:
    app = client.get_component(name="vortex_host")
    print("reusing existing app component")
except Exception:
    app = client.create_app_component(
        name="vortex_host",
        platform=xpfm,
        template="empty_application",
    )

# drop our source in
src_dst = os.path.join(ws, "vortex_host", "src")
os.makedirs(src_dst, exist_ok=True)
shutil.copy(os.path.join(script_dir, "vortex_host.c"), src_dst)
app.build()

# surface ps7_init.tcl for run.tcl
ps7s = glob.glob(os.path.join(ws, "vortex_z7_plat", "**", "ps7_init.tcl"), recursive=True)
if ps7s:
    shutil.copy(ps7s[0], os.path.join(ws, "ps7_init.tcl"))
    print(f"ps7_init.tcl -> {os.path.join(ws, 'ps7_init.tcl')}")

elfs = glob.glob(os.path.join(ws, "vortex_host", "**", "*.elf"), recursive=True)
print("ELF outputs:", elfs)
print("DONE")
