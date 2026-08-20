@echo off
call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat" >nul
cd /d C:\Users\rymcc\dolphin
cmake -B Build\libretro-x64 -G Ninja -DLIBRETRO=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo || exit /b 1
cmake --build Build\libretro-x64 --target dolphin_libretro || exit /b 1
echo BUILD_OK
