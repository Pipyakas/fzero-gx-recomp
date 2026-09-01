// host/src/main_host.c — Windows host entry with Vulkan WSI.
#include <stdio.h>
#include "../recomp_runner.h"
#ifdef _WIN32
#include <windows.h>

static const wchar_t *kClass = L"FZeroGXHost";
static const wchar_t *kTitle = L"F-Zero GX — Host (recomp main)";

extern void GXInit(void);
extern int gx_vulkan_init(void);
extern int gx_vulkan_create_surface(HWND hwnd, HINSTANCE hi);
extern void gx_vulkan_draw_frame(void);
extern void gx_vulkan_shutdown(void);
extern void gx_vulkan_set_clear(float r,float g,float b);

static int g_vk = 0; // 0=null, 1=ok, -1=failed

LRESULT CALLBACK WndProc(HWND h, UINT m, WPARAM w, LPARAM l) {
    if (m == WM_DESTROY) { PostQuitMessage(0); return 0; }
    // Vulkan presents via swapchain; WM_PAINT still clears GDI for null path
    if (m == WM_PAINT) {
        if (g_vk != 1) {
            PAINTSTRUCT ps; HDC dc = BeginPaint(h, &ps);
            RECT rc; GetClientRect(h, &rc);
            HBRUSH bg = CreateSolidBrush(RGB(6, 24, 64));
            FillRect(dc, &rc, bg); DeleteObject(bg);
            SetBkMode(dc, TRANSPARENT); SetTextColor(dc, RGB(255,255,255));
            const wchar_t *msg = (g_vk==-1)?L"F-Zero GX — Host (Vulkan failed)":L"F-Zero GX — Host (GX null)";
            DrawTextW(dc, msg, -1, &rc, DT_CENTER|DT_VCENTER|DT_SINGLELINE);
            EndPaint(h, &ps);
        } else {
            PAINTSTRUCT ps; BeginPaint(h,&ps); EndPaint(h,&ps);
        }
        return 0;
    }
    return DefWindowProcW(h, m, w, l);
}

int WINAPI wWinMain(HINSTANCE hi, HINSTANCE pi, PWSTR cmd, int show) {
    (void)pi; (void)cmd;
    AllocConsole(); freopen("CONOUT$", "w", stdout);
    printf("fzero-gx host — main (recomp) %s %s GX_NULL=", __DATE__, __TIME__);
#ifdef HOST_GX_NULL
    printf("ON\n[host] GX null\n");
#else
    printf("OFF\n");
    GXInit();
    g_vk = gx_vulkan_init();
    printf("[host] Vulkan init %s (%d)\n", g_vk==0?"OK":"failed", g_vk);
    if(g_vk==0) g_vk=1; else if(g_vk!=0) g_vk=-1;
#endif
    printf("RVZ: orig/GFZE01/f-zero gx (usa).rvz + 14 RELs\n");
    // init CPU + load DOL into RAM
    const char* dol_candidates[] = {"orig/GFZE01/sys/main.dol","C:/code/fzero-gx-recomp/orig/GFZE01/sys/main.dol","../../../orig/GFZE01/sys/main.dol",NULL};
    int ok=0; for(int i=0;dol_candidates[i];i++) if(recomp_init(dol_candidates[i])){ok=1;break;}
    printf("[recomp] %s pc=0x%08X\n", ok?"ready":"DOL load FAILED", recomp_pc());

    WNDCLASSW wc={0}; wc.lpfnWndProc=WndProc; wc.hInstance=hi; wc.lpszClassName=kClass;
    wc.hCursor=LoadCursor(NULL,IDC_ARROW); wc.hbrBackground=(HBRUSH)(COLOR_WINDOW+1);
    wc.hIcon=LoadIcon(NULL,IDI_APPLICATION);
    RegisterClassW(&wc);
    HWND w = CreateWindowW(kClass,kTitle,WS_OVERLAPPEDWINDOW,CW_USEDEFAULT,CW_USEDEFAULT,960,540,NULL,NULL,hi,NULL);
    if(!w){ MessageBoxW(NULL,L"CreateWindow failed",kTitle,MB_ICONERROR); return 1; }
    ShowWindow(w,show); UpdateWindow(w);
#ifndef HOST_GX_NULL
    if(g_vk==1){
        if(gx_vulkan_create_surface(w,hi)==0) printf("[host] WSI ready — presenting GX-blue\n");
        else { printf("[host] WSI failed — GDI fallback\n"); g_vk=-1; }
    }
#endif
    printf("Window 960x540 — close to exit. WSI=%s\n", g_vk==1?"Vulkan":"GDI");

    MSG msg; DWORD lastTitle=0; uint32_t lastFr=0;
    while(1){
        while(PeekMessageW(&msg,NULL,0,0,PM_REMOVE)){
            if(msg.message==WM_QUIT) goto done;
            TranslateMessage(&msg); DispatchMessageW(&msg);
        }
        if(recomp_inited()) recomp_run_slice();
        if(GetTickCount()-lastTitle>300){
            lastTitle=GetTickCount();
            uint32_t gb=recomp_gp_bytes(), fr=recomp_frames();
            uint64_t mr=recomp_mmio_reads();
            // guest-driven clear: prove FIFO is live — green pulse when guest flushed
            if(g_vk==1){
                if(fr!=lastFr){ gx_vulkan_set_clear(0.08f,0.55f,0.22f); lastFr=fr; }
                else if(gb>32) gx_vulkan_set_clear(0.18f,0.42f,0.78f);
                else gx_vulkan_set_clear(6.0f/255,24.0f/255,64.0f/255);
            }
            wchar_t t[160]; swprintf(t,160,L"F-Zero GX pc=0x%08X gp=%u fr=%u mmio=%llu", recomp_pc(), gb, fr, (unsigned long long)mr);
            SetWindowTextW(w,t);
        }
#ifndef HOST_GX_NULL
        if(g_vk==1) gx_vulkan_draw_frame();
#endif
        Sleep(8);
    }
done:
    recomp_shutdown();
#ifndef HOST_GX_NULL
    gx_vulkan_shutdown();
#endif
    return (int)msg.wParam;
}
#else
#include <stdio.h>
int main(void){ puts("host — non-Win32"); return 0; }
#endif
