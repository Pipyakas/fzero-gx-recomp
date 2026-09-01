#!/usr/bin/env python3
"""Add fzero-gx.exe as non-Steam game to all userdata shortcuts.vdf (binary VDF). Requires pip install vdf."""
import pathlib, struct, os
import sys
try:
    import vdf
except ImportError:
    print("pip install vdf first: pip install vdf")
    sys.exit(1)
exe = pathlib.Path("C:/code/fzero-gx-decomp/build/host/Release/fzero-gx.exe").resolve()
start_dir = str(exe.parent)
appname = "F-Zero GX (Pipyakas decomp — dev)"
userdata = pathlib.Path("C:/Steam/userdata")
for user in userdata.iterdir():
    if not user.is_dir(): continue
    vdf_path = user / "config" / "shortcuts.vdf"
    vdf_path.parent.mkdir(parents=True, exist_ok=True)
    data = {"shortcuts": {}}
    if vdf_path.exists() and vdf_path.stat().st_size > 0:
        try:
            with open(vdf_path, "rb") as f:
                data = vdf.binary_load(f)
        except Exception as e:
            print(f"load fail {vdf_path}: {e}")
            data = {"shortcuts": {}}
    shortcuts = data.get("shortcuts", {})
    # check existing by Exe
    already = any(s.get("Exe") == f'"{exe}"' or s.get("Exe") == str(exe) for s in shortcuts.values())
    if already:
        print(f"already in {vdf_path}")
        continue
    nid = str(max([int(k) for k in shortcuts.keys()] + [-1]) + 1)
    shortcuts[nid] = {
        "AppName": appname,
        "Exe": f'"{exe}"',
        "StartDir": f'"{start_dir}"',
        "LaunchOptions": "",
        "AllowDesktopConfig": 1,
        "AllowOverlay": 1,
        "OpenVR": 0,
        "Devkit": 0,
        "LastPlayed": 0,
        "FlatpakAppID": "",
        "ShortcutPath": "",
        "Hidden": 0,
        "Icon": "",
        "Tags": {},
    }
    data["shortcuts"] = shortcuts
    with open(vdf_path, "wb") as f:
        vdf.binary_dump(data, f)
    print(f"added {appname} -> {vdf_path} id {nid}")

# Also add steam grid image placeholder if wanted
print("Done — restart Steam to see F-Zero GX.")
