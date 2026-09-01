cmake_minimum_required(VERSION 3.20)
project(fzero-gx-android C CXX)
# Android arm64: same host/src + vendor/RingOut/ModernGekko
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
find_package(SDL2 QUIET)
add_library(fzero_android SHARED
    ../host/src/gx_null.c
    ../host/src/os_win32.c
    ../host/src/dvd_host.c
)
target_include_directories(fzero_android PUBLIC ../host/include ../include)
target_compile_definitions(fzero_android PRIVATE ANDROID HOST_BUILD=1)
# TODO: replace gx_null with gx_vulkan + ModernGekko link, add gradle wrapper
