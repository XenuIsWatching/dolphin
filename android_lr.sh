#!/usr/bin/env bash
# Dolphin libretro core for Quest (arm64). Mirrors .github/workflows/build-android-libretro.yml,
# with two local substitutions: the NDK already installed here, and Visual Studio's
# ninja — it is the only one on this box and is not on PATH outside a VS shell.
set -e
export ANDROID_NDK_HOME=/c/android/android-ndk-r27d
NINJA="C:/Program Files/Microsoft Visual Studio/18/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja/ninja.exe"

cmake -B Build/libretro-android-arm64 \
  -DLIBRETRO=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DANDROID_ABI=arm64-v8a \
  -DANDROID_PLATFORM=android-26 \
  -DCMAKE_TOOLCHAIN_FILE="$ANDROID_NDK_HOME/build/cmake/android.toolchain.cmake" \
  -DCMAKE_MAKE_PROGRAM="$NINJA" \
  -G Ninja
cmake --build Build/libretro-android-arm64 --target dolphin_libretro
echo ANDROID_BUILD_OK
