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

add_executable(fzero-gx WIN32
    src/main_host.c
)
target_link_libraries(fzero-gx PRIVATE fzero_host_shims)
if(WIN32)
  target_compile_definitions(fzero-gx PRIVATE _WIN32 HOST_BUILD=1)
endif()
if(FZERO_HOST_GX_NULL)
  target_compile_definitions(fzero_host_shims PUBLIC HOST_GX_NULL=1)
endif()

message(STATUS "Host scaffold configured - GX_NULL=${FZERO_HOST_GX_NULL}")
