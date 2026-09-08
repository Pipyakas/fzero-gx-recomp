cmake_minimum_required(VERSION 3.20)
project(fzero-gx-host C CXX)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Host build: compiles src/ against Win32 shims.
# Configure: cmake -S host -B build/host
#   -DFZERO_HOST_GX_NULL=OFF  -> gx_vulkan.c + Vulkan (SDK 1.4.350)

option(FZERO_HOST_GX_NULL "Use null GX renderer (no GPU output yet)" ON)

if(FZERO_HOST_GX_NULL)
    set(FZERO_GX_SRC src/gx_null.c)
else()
    set(FZERO_GX_SRC src/gx_vulkan.c)
endif()

add_library(fzero_host_shims STATIC
    ${FZERO_GX_SRC}
    src/os_win32.c
    src/dvd_host.c
)
target_include_directories(fzero_host_shims PUBLIC
    include
    ../include
    ../build/GFZE01/include
)

if(NOT FZERO_HOST_GX_NULL)
    # Prefer system Vulkan; fallback to C:/VulkanSDK/1.4.350.0
    find_package(Vulkan QUIET)
    if(Vulkan_FOUND)
        target_include_directories(fzero_host_shims PRIVATE ${Vulkan_INCLUDE_DIRS})
        target_link_libraries(fzero_host_shims PUBLIC ${Vulkan_LIBRARIES})
        message(STATUS "Vulkan found: ${Vulkan_LIBRARY}")
    else()
        set(_vk_root "$ENV{VULKAN_SDK}")
        if(NOT _vk_root OR NOT EXISTS "${_vk_root}/Include/vulkan/vulkan.h")
            set(_vk_root "C:/VulkanSDK/1.4.350.0")
        endif()
        if(EXISTS "${_vk_root}/Include/vulkan/vulkan.h")
            target_include_directories(fzero_host_shims PRIVATE "${_vk_root}/Include")
            if(EXISTS "${_vk_root}/Lib/vulkan-1.lib")
                target_link_libraries(fzero_host_shims PUBLIC "${_vk_root}/Lib/vulkan-1.lib")
            endif()
            message(STATUS "Vulkan fallback: ${_vk_root}")
        else()
            message(WARNING "Vulkan not found; gx_vulkan will fail to link")
        endif()
    endif()
endif()

file(GLOB_RECURSE RECOMP_DOL_CHUNKS "${CMAKE_CURRENT_SOURCE_DIR}/../build/recomp_all/generated/chunks/*.c")
file(GLOB RECOMP_DOL_CHUNKS_FB "${CMAKE_CURRENT_SOURCE_DIR}/../build/recomp_all/generated/dol/generated/chunks/*.c")
list(APPEND RECOMP_DOL_CHUNKS ${RECOMP_DOL_CHUNKS_FB})

# GXRuntime chassis (C-only, no Aurora): VI/PI interrupt model + retrace clock
# + DI (DVD interface) model + DVD image/FST layer.
set(GXRUNTIME_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../vendor/RingOut/ModernGekko/vendor/dolphin/GXRuntime")
# NOTE: guest_memory.c/memory_card.c/aram.c/platform.c/headless_backend.c
# are NOT included: guest_memory.c targets a newer CPUState (exram +
# external_pointer tail) than DolRecomp's cpu.h in this TU, and the rest
# pull platform/GX deps. hle_core.c's Card/LC/ARAM helpers that need them
# are unused here (only queue/poll/return + OSReport/DVD/PAD paths matter);
# if link errors name them, add minimal local stubs instead of the files.
set(GXRUNTIME_SOURCES
    ${GXRUNTIME_DIR}/src/mmio_bus.c
    ${GXRUNTIME_DIR}/src/interrupts.c
    ${GXRUNTIME_DIR}/src/vi_clock.c
    ${GXRUNTIME_DIR}/src/event_clock.c
    ${GXRUNTIME_DIR}/src/di.c
    ${GXRUNTIME_DIR}/src/dvd.c
    ${GXRUNTIME_DIR}/src/si.c
    ${GXRUNTIME_DIR}/src/hle/hle_abi.c
    src/hle_callback.c
)

add_library(recomp_core STATIC
    src/gx_fifo_bridge.c
    src/recomp_runner.c
    ../vendor/RingOut/DolRecomp/src/cpu/cpu.c
    ${GXRUNTIME_SOURCES}
    ${RECOMP_DOL_CHUNKS}
)
target_include_directories(recomp_core PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/../vendor/RingOut/DolRecomp/src"
    "${GXRUNTIME_DIR}/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../build/recomp_all/generated"
)
target_compile_definitions(recomp_core PRIVATE _CRT_SECURE_NO_WARNINGS)
# dvd.c uses POSIX fseeko()/off_t; the Windows UCRT provides
# _fseeki64/__int64 instead. Force-include the compat header for that TU
# only (a global -Dfseeko= would break <stdio.h>'s own declaration).
target_include_directories(recomp_core PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src/compat")
if(MSVC)
    # cpu.c + GXRuntime use GCC/Clang spellings (__attribute__, __builtin_bswap,
    # <stdatomic.h>). msvc_compat.h nop-defines them; the shadow stdatomic.h in
    # the same dir covers the include. MSVC-only: never on the clang path.
    target_compile_options(recomp_core PRIVATE "/FI${CMAKE_CURRENT_SOURCE_DIR}/src/compat/msvc_compat.h")
endif()
if(WIN32 AND NOT MSVC)
    # clang path only: MSVC gets fseeko/off_t from msvc_compat.h (/FI above)
    # instead; -include is not a valid cl.exe flag.
    set_source_files_properties(
        "${GXRUNTIME_DIR}/src/dvd.c"
        PROPERTIES COMPILE_FLAGS "-include \"${CMAKE_CURRENT_SOURCE_DIR}/src/compat/dvd_port.h\"")
endif()

add_executable(fzero-gx WIN32
    src/main_host.c
)
target_link_libraries(fzero-gx PRIVATE fzero_host_shims recomp_core)
if(WIN32)
  target_compile_definitions(fzero-gx PRIVATE _WIN32 HOST_BUILD=1)
endif()
if(FZERO_HOST_GX_NULL)
  target_compile_definitions(fzero_host_shims PUBLIC HOST_GX_NULL=1)
endif()

message(STATUS "Host scaffold configured - GX_NULL=${FZERO_HOST_GX_NULL}")
