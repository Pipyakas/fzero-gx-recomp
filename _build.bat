call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" -arch=x64 >nul
cmake -S host -B build/host-ninja -G Ninja -DCMAKE_C_COMPILER="C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/x64/bin/clang.exe" -DCMAKE_CXX_COMPILER="C:/Program Files/Microsoft Visual Studio/2022/Community/VC/Tools/Llvm/x64/bin/clang.exe" -DFZERO_HOST_GX_NULL=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build/host-ninja -j
