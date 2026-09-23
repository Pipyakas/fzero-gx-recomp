call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" -arch=x64 >nul
set INCLUDE > C:\code\fzero-gx-recomp\_inc.txt
set LIB > C:\code\fzero-gx-recomp\_lib.txt
"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Tools\Llvm\x64\bin\clang.exe" --version >> C:\code\fzero-gx-recomp\_inc.txt 2>&1
