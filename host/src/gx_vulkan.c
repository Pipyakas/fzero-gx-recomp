// host/src/gx_vulkan.c — Vulkan GX backend with Win32 WSI.
// Phase: instance+device (previous) -> now + surface+swapchain+clear.
// SDK 1.4.350 (C:/VulkanSDK/1.4.350.0) via vulkan-1.lib.
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#ifdef _WIN32
#define VK_USE_PLATFORM_WIN32_KHR
#include <vulkan/vulkan.h>
#include <windows.h>

static VkInstance g_inst = VK_NULL_HANDLE;
static VkPhysicalDevice g_pdev = VK_NULL_HANDLE;
static VkDevice g_dev = VK_NULL_HANDLE;
static VkQueue g_queue = VK_NULL_HANDLE;
static VkSurfaceKHR g_surf = VK_NULL_HANDLE;
static VkSwapchainKHR g_swap = VK_NULL_HANDLE;
static VkRenderPass g_rp = VK_NULL_HANDLE;
static VkCommandPool g_pool = VK_NULL_HANDLE;
static VkCommandBuffer g_cmd = VK_NULL_HANDLE;
static VkFence g_fence = VK_NULL_HANDLE;
static VkSemaphore g_acq = VK_NULL_HANDLE, g_present = VK_NULL_HANDLE;
static VkImage g_images[8]; uint32_t g_nimg = 0;
static VkImageView g_views[8] = {0};
static VkFramebuffer g_fbs[8] = {0};
static VkExtent2D g_ext = {960,540};
static uint32_t g_qfam = 0;
static int g_inited = 0, g_failed = 0, g_has_swap = 0;

static void vklog(const char *m, VkResult r){ fprintf(stderr,"[gx_vk] %s: %d\n",m,(int)r); }

static VkResult create_renderpass(void){
    VkAttachmentDescription att={0};
    att.format = VK_FORMAT_B8G8R8A8_SRGB;
    // fallback handled: pick from surface format later; for now assume SRGB
    att.samples = VK_SAMPLE_COUNT_1_BIT;
    att.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    att.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    att.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    att.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    VkAttachmentReference ref={0}; ref.attachment=0; ref.layout=VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    VkSubpassDescription sub={0}; sub.pipelineBindPoint=VK_PIPELINE_BIND_POINT_GRAPHICS; sub.colorAttachmentCount=1; sub.pColorAttachments=&ref;
    VkRenderPassCreateInfo ci={0}; ci.sType=VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    ci.attachmentCount=1; ci.pAttachments=&att; ci.subpassCount=1; ci.pSubpasses=&sub;
    return vkCreateRenderPass(g_dev,&ci,NULL,&g_rp);
}

int gx_vulkan_init(void){
    if(g_inited||g_failed) return g_inited?0:-1;
    const char *exts[]={"VK_KHR_surface","VK_KHR_win32_surface"};
    VkApplicationInfo app={0}; app.sType=VK_STRUCTURE_TYPE_APPLICATION_INFO;
    app.pApplicationName="fzero-gx"; app.apiVersion=VK_API_VERSION_1_1;
    VkInstanceCreateInfo ci={0}; ci.sType=VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    ci.pApplicationInfo=&app; ci.enabledExtensionCount=2; ci.ppEnabledExtensionNames=exts;
    VkResult r=vkCreateInstance(&ci,NULL,&g_inst);
    if(r!=VK_SUCCESS){ vklog("vkCreateInstance",r); g_failed=1; return -1; }
    uint32_t n=0; vkEnumeratePhysicalDevices(g_inst,&n,NULL);
    if(n==0){ vklog("no pdev",VK_ERROR_INITIALIZATION_FAILED); g_failed=1; return -1; }
    VkPhysicalDevice devs[8]; if(n>8)n=8; vkEnumeratePhysicalDevices(g_inst,&n,devs); g_pdev=devs[0];
    VkPhysicalDeviceProperties pr; vkGetPhysicalDeviceProperties(g_pdev,&pr);
    printf("[gx_vk] GPU: %s api %u.%u\n",pr.deviceName,VK_VERSION_MAJOR(pr.apiVersion),VK_VERSION_MINOR(pr.apiVersion));
    uint32_t qc=0; vkGetPhysicalDeviceQueueFamilyProperties(g_pdev,&qc,NULL);
    VkQueueFamilyProperties qp[16]; if(qc>16)qc=16; vkGetPhysicalDeviceQueueFamilyProperties(g_pdev,&qc,qp);
    int found=0; for(uint32_t i=0;i<qc;i++) if(qp[i].queueFlags&VK_QUEUE_GRAPHICS_BIT){ g_qfam=i; found=1; break; }
    if(!found){ fprintf(stderr,"[gx_vk] no gfx queue\n"); g_failed=1; return -1; }
    float prio=1.0f; VkDeviceQueueCreateInfo dq={0}; dq.sType=VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
    dq.queueFamilyIndex=g_qfam; dq.queueCount=1; dq.pQueuePriorities=&prio;
    const char *dext[]={"VK_KHR_swapchain"};
    VkDeviceCreateInfo dci={0}; dci.sType=VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    dci.queueCreateInfoCount=1; dci.pQueueCreateInfos=&dq; dci.enabledExtensionCount=1; dci.ppEnabledExtensionNames=dext;
    r=vkCreateDevice(g_pdev,&dci,NULL,&g_dev);
    if(r!=VK_SUCCESS){ vklog("vkCreateDevice",r); g_failed=1; return -1; }
    vkGetDeviceQueue(g_dev,g_qfam,0,&g_queue);
    g_inited=1;
    printf("[gx_vk] init OK (instance+device, no surface yet)\n");
    return 0;
}

int gx_vulkan_create_surface(HWND hwnd, HINSTANCE hi){
    if(!g_inited){ if(gx_vulkan_init()!=0) return -1; }
    if(g_has_swap) return 0;
    VkWin32SurfaceCreateInfoKHR sci={0}; sci.sType=VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
    sci.hinstance=hi; sci.hwnd=hwnd;
    VkResult r=vkCreateWin32SurfaceKHR(g_inst,&sci,NULL,&g_surf);
    if(r!=VK_SUCCESS){ vklog("vkCreateWin32SurfaceKHR",r); return -1; }
    VkBool32 sup=VK_FALSE; vkGetPhysicalDeviceSurfaceSupportKHR(g_pdev,g_qfam,g_surf,&sup);
    if(!sup){ fprintf(stderr,"[gx_vk] surface not supported on qfam\n"); return -1; }
    // surface format
    uint32_t nfmt=0; vkGetPhysicalDeviceSurfaceFormatsKHR(g_pdev,g_surf,&nfmt,NULL);
    VkSurfaceFormatKHR fmts[16]; if(nfmt>16)nfmt=16; vkGetPhysicalDeviceSurfaceFormatsKHR(g_pdev,g_surf,&nfmt,fmts);
    VkFormat fmt = VK_FORMAT_B8G8R8A8_SRGB; VkColorSpaceKHR cs = VK_COLOR_SPACE_SRGB_NONLINEAR_KHR;
    if(nfmt>0 && fmts[0].format!=VK_FORMAT_UNDEFINED){ fmt=fmts[0].format; cs=fmts[0].colorSpace; }
    // prefer SRGB if available
    for(uint32_t i=0;i<nfmt;i++) if(fmts[i].format==VK_FORMAT_B8G8R8A8_SRGB){ fmt=fmts[i].format; cs=fmts[i].colorSpace; break; }
    VkSurfaceCapabilitiesKHR cap; vkGetPhysicalDeviceSurfaceCapabilitiesKHR(g_pdev,g_surf,&cap);
    if(cap.currentExtent.width!=0xFFFFFFFF) g_ext=cap.currentExtent; else { g_ext.width=960; g_ext.height=540; }
    uint32_t nmode=0; vkGetPhysicalDeviceSurfacePresentModesKHR(g_pdev,g_surf,&nmode,NULL);
    VkPresentModeKHR modes[8]; if(nmode>8)nmode=8; vkGetPhysicalDeviceSurfacePresentModesKHR(g_pdev,g_surf,&nmode,modes);
    VkPresentModeKHR pm=VK_PRESENT_MODE_FIFO_KHR; // always available
    VkSwapchainCreateInfoKHR sw={0}; sw.sType=VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    sw.surface=g_surf; sw.minImageCount=cap.minImageCount<2?2:cap.minImageCount;
    if(cap.maxImageCount && sw.minImageCount>cap.maxImageCount) sw.minImageCount=cap.maxImageCount;
    sw.imageFormat=fmt; sw.imageColorSpace=cs; sw.imageExtent=g_ext;
    sw.imageArrayLayers=1; sw.imageUsage=VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    sw.imageSharingMode=VK_SHARING_MODE_EXCLUSIVE; sw.preTransform=cap.currentTransform;
    sw.compositeAlpha=VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR; sw.presentMode=pm; sw.clipped=VK_TRUE;
    r=vkCreateSwapchainKHR(g_dev,&sw,NULL,&g_swap);
    if(r!=VK_SUCCESS){ vklog("vkCreateSwapchainKHR",r); return -1; }
    vkGetSwapchainImagesKHR(g_dev,g_swap,&g_nimg,NULL);
    if(g_nimg>8) g_nimg=8; vkGetSwapchainImagesKHR(g_dev,g_swap,&g_nimg,g_images);
    printf("[gx_vk] swapchain %ux%u fmt %d images %u\n", g_ext.width,g_ext.height,(int)fmt,g_nimg);
    // render pass + image views + framebuffers
    // recreate rp with actual fmt
    {
        VkAttachmentDescription att={0}; att.format=fmt; att.samples=VK_SAMPLE_COUNT_1_BIT;
        att.loadOp=VK_ATTACHMENT_LOAD_OP_CLEAR; att.storeOp=VK_ATTACHMENT_STORE_OP_STORE;
        att.initialLayout=VK_IMAGE_LAYOUT_UNDEFINED; att.finalLayout=VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
        VkAttachmentReference ref={0}; ref.layout=VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
        VkSubpassDescription sub={0}; sub.pipelineBindPoint=VK_PIPELINE_BIND_POINT_GRAPHICS; sub.colorAttachmentCount=1; sub.pColorAttachments=&ref;
        VkRenderPassCreateInfo rpi={0}; rpi.sType=VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        rpi.attachmentCount=1; rpi.pAttachments=&att; rpi.subpassCount=1; rpi.pSubpasses=&sub;
        r=vkCreateRenderPass(g_dev,&rpi,NULL,&g_rp); if(r!=VK_SUCCESS){ vklog("vkCreateRenderPass",r); return -1; }
    }
    for(uint32_t i=0;i<g_nimg;i++){
        VkImageViewCreateInfo vi={0}; vi.sType=VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        vi.image=g_images[i]; vi.viewType=VK_IMAGE_VIEW_TYPE_2D; vi.format=fmt;
        vi.components=(VkComponentMapping){VK_COMPONENT_SWIZZLE_IDENTITY,VK_COMPONENT_SWIZZLE_IDENTITY,VK_COMPONENT_SWIZZLE_IDENTITY,VK_COMPONENT_SWIZZLE_IDENTITY};
        vi.subresourceRange.aspectMask=VK_IMAGE_ASPECT_COLOR_BIT; vi.subresourceRange.levelCount=1; vi.subresourceRange.layerCount=1;
        r=vkCreateImageView(g_dev,&vi,NULL,&g_views[i]); if(r!=VK_SUCCESS){ vklog("vkCreateImageView",r); return -1; }
        VkFramebufferCreateInfo fbi={0}; fbi.sType=VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fbi.renderPass=g_rp; fbi.attachmentCount=1; fbi.pAttachments=&g_views[i]; fbi.width=g_ext.width; fbi.height=g_ext.height; fbi.layers=1;
        r=vkCreateFramebuffer(g_dev,&fbi,NULL,&g_fbs[i]); if(r!=VK_SUCCESS){ vklog("vkCreateFramebuffer",r); return -1; }
    }
    VkCommandPoolCreateInfo pci={0}; pci.sType=VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO; pci.flags=VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT; pci.queueFamilyIndex=g_qfam;
    r=vkCreateCommandPool(g_dev,&pci,NULL,&g_pool); if(r!=VK_SUCCESS){ vklog("vkCreateCommandPool",r); return -1; }
    VkCommandBufferAllocateInfo ai={0}; ai.sType=VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO; ai.commandPool=g_pool; ai.level=VK_COMMAND_BUFFER_LEVEL_PRIMARY; ai.commandBufferCount=1;
    r=vkAllocateCommandBuffers(g_dev,&ai,&g_cmd); if(r!=VK_SUCCESS){ vklog("vkAllocateCommandBuffers",r); return -1; }
    VkFenceCreateInfo fci={0}; fci.sType=VK_STRUCTURE_TYPE_FENCE_CREATE_INFO; fci.flags=VK_FENCE_CREATE_SIGNALED_BIT;
    vkCreateFence(g_dev,&fci,NULL,&g_fence);
    VkSemaphoreCreateInfo sci2={0}; sci2.sType=VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
    vkCreateSemaphore(g_dev,&sci2,NULL,&g_acq); vkCreateSemaphore(g_dev,&sci2,NULL,&g_present);
    g_has_swap=1;
    printf("[gx_vk] WSI ready\n");
    return 0;
}

void gx_vulkan_draw_frame(void){
    if(!g_has_swap) return;
    vkWaitForFences(g_dev,1,&g_fence,VK_TRUE,1000000000); vkResetFences(g_dev,1,&g_fence);
    uint32_t idx=0; VkResult r=vkAcquireNextImageKHR(g_dev,g_swap,1000000000,g_acq,VK_NULL_HANDLE,&idx);
    if(r!=VK_SUCCESS) return;
    vkResetCommandPool(g_dev,g_pool,0);
    VkCommandBufferBeginInfo bi={0}; bi.sType=VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO; bi.flags=VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(g_cmd,&bi);
    VkClearValue cv; cv.color.float32[0]=6.0f/255.0f; cv.color.float32[1]=24.0f/255.0f; cv.color.float32[2]=64.0f/255.0f; cv.color.float32[3]=1.0f;
    VkRenderPassBeginInfo rp={0}; rp.sType=VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    rp.renderPass=g_rp; rp.framebuffer=g_fbs[idx]; rp.renderArea.extent=g_ext; rp.clearValueCount=1; rp.pClearValues=&cv;
    vkCmdBeginRenderPass(g_cmd,&rp,VK_SUBPASS_CONTENTS_INLINE);
    vkCmdEndRenderPass(g_cmd);
    vkEndCommandBuffer(g_cmd);
    VkPipelineStageFlags wait = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo si={0}; si.sType=VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.waitSemaphoreCount=1; si.pWaitSemaphores=&g_acq; si.pWaitDstStageMask=&wait;
    si.commandBufferCount=1; si.pCommandBuffers=&g_cmd;
    si.signalSemaphoreCount=1; si.pSignalSemaphores=&g_present;
    vkQueueSubmit(g_queue,1,&si,g_fence);
    VkPresentInfoKHR pi={0}; pi.sType=VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
    pi.waitSemaphoreCount=1; pi.pWaitSemaphores=&g_present; pi.swapchainCount=1; pi.pSwapchains=&g_swap; pi.pImageIndices=&idx;
    vkQueuePresentKHR(g_queue,&pi);
}

void gx_vulkan_shutdown(void){
    if(g_dev) vkDeviceWaitIdle(g_dev);
    if(g_fence) vkDestroyFence(g_dev,g_fence,NULL); g_fence=VK_NULL_HANDLE;
    if(g_acq) vkDestroySemaphore(g_dev,g_acq,NULL); g_acq=VK_NULL_HANDLE;
    if(g_present) vkDestroySemaphore(g_dev,g_present,NULL); g_present=VK_NULL_HANDLE;
    if(g_pool) vkDestroyCommandPool(g_dev,g_pool,NULL); g_pool=VK_NULL_HANDLE; g_cmd=VK_NULL_HANDLE;
    for(uint32_t i=0;i<g_nimg;i++){ if(g_fbs[i]) vkDestroyFramebuffer(g_dev,g_fbs[i],NULL); if(g_views[i]) vkDestroyImageView(g_dev,g_views[i],NULL); }
    g_nimg=0;
    if(g_rp) vkDestroyRenderPass(g_dev,g_rp,NULL); g_rp=VK_NULL_HANDLE;
    if(g_swap) vkDestroySwapchainKHR(g_dev,g_swap,NULL); g_swap=VK_NULL_HANDLE;
    if(g_surf) vkDestroySurfaceKHR(g_inst,g_surf,NULL); g_surf=VK_NULL_HANDLE;
    if(g_dev) vkDestroyDevice(g_dev,NULL); g_dev=VK_NULL_HANDLE;
    if(g_inst) vkDestroyInstance(g_inst,NULL); g_inst=VK_NULL_HANDLE;
    g_inited=0; g_has_swap=0; printf("[gx_vk] shutdown\n");
}

void GXInit(void){ gx_vulkan_init(); }
void GXBegin(void){}
void GXSetLineWidth(unsigned char w,unsigned char f){(void)w;(void)f;}
void GXSetPointSize(unsigned char s,unsigned char f){(void)s;(void)f;}

#else
void GXInit(void){}
void GXBegin(void){}
void GXSetLineWidth(unsigned char w,unsigned char f){(void)w;(void)f;}
void GXSetPointSize(unsigned char s,unsigned char f){(void)s;(void)f;}
int gx_vulkan_init(void){return -1;}
int gx_vulkan_create_surface(void*p,void*q){(void)p;(void)q; return -1;}
void gx_vulkan_draw_frame(void){}
void gx_vulkan_shutdown(void){}
#endif
