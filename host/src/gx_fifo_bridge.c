// gx_fifo_bridge.c — minimal GX FIFO parser (C) that counts draws
// Mirrors ModernGekko GxCommandProcessor DecodeOne for the subset F-Zero uses:
// 0x00 NOP, 0x08 CP, 0x10 XF, 0x61 BP, 0x80-0xBF draw, 0x40 display list, 0x44/0x48
#include "gx_fifo_bridge.h"
#include <string.h>
#include <stdint.h>

static uint64_t s_draws=0, s_cmds=0, s_unknown=0;
static uint32_t s_vcd_low=0, s_vcd_high=0;
static uint32_t s_vat[8][3]={0};
static uint8_t s_buf[1<<20];
static size_t s_len=0;

static uint32_t be32(const uint8_t*p){ return ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|((uint32_t)p[2]<<8)|p[3]; }
static uint16_t be16(const uint8_t*p){ return (uint16_t)((p[0]<<8)|p[1]); }
static uint32_t bits(uint32_t v, unsigned o, unsigned c){ return (v>>o)&((1u<<c)-1u); }
static uint32_t cbs(uint32_t f){ return f<2u?1u:f<4u?2u:4u; }
static uint32_t attr_sz(uint32_t d, uint32_t ds){ switch(d){case 0:return 0;case 1:return ds;case 2:return 1;case 3:return 2;default:return 0;}}
static uint32_t col_ds(uint32_t fmt){ static const uint32_t s[8]={2,3,4,2,3,4,0,0}; return s[fmt&7u]; }

static uint32_t vtx_size(uint8_t vat){
  const uint32_t g0=s_vat[vat&7][0], g1=s_vat[vat&7][1], g2=s_vat[vat&7][2];
  uint32_t sz=0; for(int i=0;i<9;i++) if(s_vcd_low & (1u<<i)) sz++;
  uint32_t pos=bits(s_vcd_low,9,2); uint32_t pc=bits(g0,0,1)+2u;
  sz+= attr_sz(pos, cbs(bits(g0,1,3))*pc);
  uint32_t nrm=bits(s_vcd_low,11,2); int ntb=bits(g0,9,1)!=0, i3=bits(g0,31,1)!=0;
  if(nrm==1) sz+= cbs(bits(g0,10,3))*(ntb?9u:3u); else if(nrm==2||nrm==3) sz+=(nrm-1u)*(ntb&&i3?3u:1u);
  for(unsigned c=0;c<2;c++){ uint32_t d=bits(s_vcd_low,13+c*2,2), f=bits(g0,14+c*4,3); sz+= attr_sz(d,col_ds(f)); }
  for(unsigned t=0;t<8;t++){
    uint32_t d=bits(s_vcd_high,t*2,2);
    uint32_t fmt,elts;
    switch(t){ case 0: fmt=bits(g0,22,3); elts=bits(g0,21,1); break;
               case 1: fmt=bits(g1,1,3); elts=bits(g1,0,1); break;
               case 2: fmt=bits(g1,10,3); elts=bits(g1,9,1); break;
               case 3: fmt=bits(g1,19,3); elts=bits(g1,18,1); break;
               case 4: fmt=bits(g1,28,3); elts=bits(g1,27,1); break;
               case 5: fmt=bits(g2,6,3); elts=bits(g2,5,1); break;
               case 6: fmt=bits(g2,15,3); elts=bits(g2,14,1); break;
               default: fmt=bits(g2,24,3); elts=bits(g2,23,1); break; }
    sz+= attr_sz(d, cbs(fmt)*(elts+1u));
  }
  return sz;
}

static size_t decode_one(const uint8_t* d, size_t n){
  if(n==0) return 0;
  uint8_t cmd=d[0];
  if(cmd==0x00){ size_t i=0; while(i<n && d[i]==0) i++; return i?i:0; }
  if(cmd==0x44||cmd==0x48) return 1;
  if(cmd==0x08){ if(n<6) return 0; uint8_t c=d[1]; uint32_t v=be32(d+2);
    switch(c&0xF0){ case 0x50: s_vcd_low=v; break; case 0x60: s_vcd_high=v; break;
      case 0x70: s_vat[c&7][0]=v; break; case 0x80: s_vat[c&7][1]=v; break;
      case 0x90: s_vat[c&7][2]=v; break; default: break; } return 6; }
  if(cmd==0x10){ if(n<5) return 0; uint32_t h=be32(d+1); uint8_t cnt=(uint8_t)(((h>>16)&0xF)+1u); size_t sz=5u+cnt*4u; if(n<sz) return 0; return sz; }
  if(cmd==0x20||cmd==0x28||cmd==0x30||cmd==0x38){ if(n<5) return 0; return 5; }
  if(cmd==0x40){ if(n<9) return 0; return 9; }
  if(cmd==0x61){ if(n<5) return 0; return 5; }
  if(cmd>=0x80 && cmd<=0xBF){ if(n<3) return 0; uint8_t vat=cmd&7u; uint16_t cnt=be16(d+1); uint32_t vs=vtx_size(vat);
    size_t sz=3u+(size_t)cnt*vs; if(n<sz) return 0; if(vs) s_draws++; return sz; }
  s_unknown++; return 1;
}

void gx_fifo_write(const uint8_t* data, size_t len){
  if(!data||!len) return;
  if(s_len+len > sizeof(s_buf)){ s_len=0; } // overflow: drop
  memcpy(s_buf+s_len, data, len); s_len+=len;
  size_t consumed=0;
  while(consumed < s_len){
    size_t sz=decode_one(s_buf+consumed, s_len-consumed);
    if(sz==0) break;
    consumed+=sz; s_cmds++;
  }
  if(consumed){ memmove(s_buf, s_buf+consumed, s_len-consumed); s_len-=consumed; }
}
uint64_t gx_fifo_draws(void){ return s_draws; }
uint64_t gx_fifo_cmds(void){ return s_cmds; }
