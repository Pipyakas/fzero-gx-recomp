@echo off
set INCLUDE=
set LIB=
set PATH=C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\Program Files\Git\cmd;C:\Program Files\LLVM\bin;C:\VulkanSDK\1.4.350.0\Bin
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
set INCLUDE=%INCLUDE%;C:\Program Files\Microsoft Visual Studio\18\Community\VC\Tools\MSVC\14.51.36231\include
set LIB=%LIB%;C:\Program Files\Microsoft Visual Studio\18\Community\VC\Tools\MSVC\14.51.36231\lib\x64
cmake -S host -B build/host-ninja -G Ninja -DCMAKE_C_COMPILER="C:/Program Files/LLVM/bin/clang.exe" -DCMAKE_CXX_COMPILER="C:/Program Files/LLVM/bin/clang++.exe" -DFZERO_HOST_GX_NULL=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/host-ninja -j
