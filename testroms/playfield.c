#include "playfield.h"
#include "util.h"

static uint16_t pf_control[4] = { 0, 0, 0, 0 };
static uint16_t pf_x[4] = { 0, 0, 0, 0 };
static uint16_t pf_y[4] = { 0, 0, 0, 0 };

static inline void pf_submit(uint8_t idx)
{
    uint16_t port = 0x90 + (idx << 1);
    uint16_t x_port = 0x82 + (idx << 2);
    uint16_t y_port = 0x80 + (idx << 2);

    __outw(port, pf_control[idx]);
    __outw(x_port, pf_x[idx]);
    __outw(y_port, pf_y[idx]);
}

void pf_reset()
{
    memsetw(VRAM, 0, 0x8000);

    pf_control[0] = pf_control[1] = pf_control[2] = pf_control[3] = 0;

    pf_enable(0, false);
    pf_enable(1, false);
    pf_enable(2, false);
    pf_enable(3, false);

    pf_set_xy(0, -83, -144);
    pf_set_xy(1, -81, -144);
    pf_set_xy(2, -79, -144);
    pf_set_xy(3, -77, -144);

    pf_set_vram(0, 0x0000);
    pf_set_vram(1, 0x4000);
    pf_set_vram(2, 0x8000);
    pf_set_vram(3, 0xc000);
}

void pf_enable(uint8_t idx, bool enabled)
{
    if (enabled)
        pf_control[idx] &= ~0x0080;
    else
        pf_control[idx] |= 0x0080;
    
    pf_submit(idx);
}

void pf_set_vram(uint8_t idx, uint16_t base)
{
    pf_control[idx] &= 0xf0ff;
    pf_control[idx] |= (base >> 4) & 0x0f00;
    pf_submit(idx);
}

void pf_set_flags(uint8_t idx, uint8_t flags)
{
    pf_control[idx] &= 0xff80;
    pf_control[idx] |= flags & 0x7f;
    pf_submit(idx);
}

void pf_set_xy(uint8_t idx, uint16_t x, uint16_t y)
{
    pf_x[idx] = x;
    pf_y[idx] = y;

    pf_submit(idx);
}

uint16_t pf_get_x(uint8_t idx)
{
    return pf_x[idx];
}

uint16_t pf_get_y(uint8_t idx)
{
    return pf_y[idx];
}

__far uint16_t *pf_addr(uint8_t idx)
{
    __far uint16_t *addr = VRAM;
    addr += (pf_control[idx] & 0x0f00) << 3;
    return addr;
}

__far uint16_t *pf_rowscroll_addr(uint8_t idx)
{
    return VRAM + ((0xe000 + (0x200 * idx)) >> 1);
}

__far uint16_t *pf_rowselect_addr(uint8_t idx)
{
    return VRAM + ((0xe800 + (0x200 * idx)) >> 1);
}

void pf_text(uint8_t layer, uint16_t color, uint16_t x, uint16_t y, const char *str)
{
    int ofs = ( x * 64 ) + y;
    __far uint16_t *addr = pf_addr(layer);

    while(*str)
    {
        if( *str == '\n' )
        {
            x++;
            ofs = (x * 64) + y;
        }
        else
        {
            addr[(ofs << 1) + 1] = color;
            addr[(ofs << 1)] = 0x2000 + *str;
            ofs += 64;
        }
        str++;
    }
}

void pf_sym(uint8_t layer, uint16_t color, uint16_t x, uint16_t y, uint16_t sym)
{
    int ofs = ( x * 64 ) + y;
    __far uint16_t *addr = pf_addr(layer);

    addr[(ofs << 1) + 1] = color;
    addr[(ofs << 1)] = sym;
}
