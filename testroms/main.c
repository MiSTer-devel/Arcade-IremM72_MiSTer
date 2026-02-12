#include <stdint.h>
#include <stdbool.h>
#include "printf/printf.h"

#include "input.h"
#include "playfield.h"
#include "util.h"
#include "interrupts.h"
#include "comms.h"

char last_cmd[32];

enum
{
    CMD_IDLE = 0,
    CMD_WRITE_BYTES = 1,
    CMD_WRITE_WORDS = 2,
    CMD_READ_BYTES = 3,
    CMD_READ_WORDS = 4,
    CMD_FILL_BYTES = 5,
    CMD_FILL_WORDS = 6,
    CMD_OUT_WORD = 7,
    CMD_IN_WORD = 8
};

typedef struct Cmd
{
    uint8_t cmd;
    uint32_t arg0;
    uint32_t arg1;

    uint16_t total_bytes;
    uint16_t total_bytes_read;
    uint16_t total_bytes_consumed;

    uint16_t bytes_avail;
    uint16_t bytes_consumed;

    bool is_new;

    uint8_t buffer[32] __attribute__((aligned(2)));
} Cmd;

void update_cmd(Cmd *cmd)
{
    uint8_t buf[10];

    if (cmd->bytes_consumed > 0)
    {
        memcpy(cmd->buffer, cmd->buffer + cmd->bytes_consumed, cmd->bytes_avail - cmd->bytes_consumed);
        cmd->bytes_avail -= cmd->bytes_consumed;
        cmd->total_bytes_consumed += cmd->bytes_consumed;
        cmd->bytes_consumed = 0;
    }

    if (cmd->total_bytes_consumed == cmd->total_bytes && cmd->cmd != CMD_IDLE)
    {
        comms_write(&cmd->cmd, 1);
        cmd->cmd = CMD_IDLE;

    }

    if (cmd->cmd == CMD_IDLE)
    {
        if( comms_read(&cmd->cmd, 1) == 0 )
        {
            return;
        }

        if (cmd->cmd == CMD_IDLE)
        {
            return;
        }

        int pos = 0;
        while (pos < 10)
        {
            pos += comms_read(buf + pos, 10 - pos);
        }

        cmd->arg0 = *(uint32_t *)(buf + 0);
        cmd->arg1 = *(uint32_t *)(buf + 4);
        cmd->total_bytes = *(uint16_t *)(buf + 8);

        cmd->bytes_avail = 0;
        cmd->bytes_consumed = 0;
        cmd->total_bytes_read = 0;
        cmd->total_bytes_consumed = 0;
        cmd->is_new = true;
    }
    else
    {
        cmd->is_new = false;
    }

    if (cmd->total_bytes_read < cmd->total_bytes && cmd->bytes_avail < sizeof(cmd->buffer))
    {
        uint16_t remaining = cmd->total_bytes - cmd->total_bytes_read;
        uint16_t space = sizeof(cmd->buffer) - cmd->bytes_avail;

        uint16_t max_read = space < remaining ? space : remaining;
        uint16_t bytes_read = comms_read(cmd->buffer + cmd->bytes_avail, max_read);
        cmd->total_bytes_read += bytes_read;
        cmd->bytes_avail += bytes_read;
    }
}

void process_cmd(Cmd *cmd)
{
    switch(cmd->cmd)
    {
        case CMD_WRITE_BYTES:
        {
            //if (cmd->is_new) snprintf(last_cmd, sizeof(last_cmd), "WRITE %X BYTES @ %08X", cmd->total_bytes, cmd->arg0);
            uint8_t __far *addr = (__far uint8_t *)(cmd->arg0 + cmd->total_bytes_consumed);
            if( cmd->bytes_avail > 0 )
            {
                memcpyb(addr, cmd->buffer, cmd->bytes_avail);
                cmd->bytes_consumed = cmd->bytes_avail;
            }
            break;
        }
        case CMD_WRITE_WORDS:
        {
            //if (cmd->is_new) snprintf(last_cmd, sizeof(last_cmd), "WRITE %X WORDS @ %08X", cmd->total_bytes >> 1, cmd->arg0);
            uint16_t __far *addr = (__far uint16_t *)(cmd->arg0 + cmd->total_bytes_consumed);
            memcpyw(addr, cmd->buffer, cmd->bytes_avail >> 1);
            cmd->bytes_consumed = (cmd->bytes_avail & ~0x1);
            break;
        }
        case CMD_READ_BYTES:
        {
            //if (cmd->is_new) snprintf(last_cmd, sizeof(last_cmd), "READ %X BYTES @ %08X", cmd->arg1, cmd->arg0);
            uint8_t __far *addr = (__far uint8_t *)cmd->arg0;
            comms_write(addr, cmd->arg1);
            break;
        }
        case CMD_READ_WORDS:
        {
            //if (cmd->is_new) snprintf(last_cmd, sizeof(last_cmd), "READ %X WORDS @ %08X", cmd->arg1, cmd->arg0);
            uint8_t __far *addr = (__far uint8_t *)cmd->arg0;
            for( int ofs = 0; ofs < cmd->arg1; ofs++)
            {
                comms_write(addr + (ofs << 1), 2);
            }
            break;
        }
        case CMD_FILL_BYTES:
        {
            //if (cmd->is_new) snprintf(last_cmd, sizeof(last_cmd), "FILL %X BYTES @ %08X", cmd->arg1, cmd->arg0);
            if (cmd->bytes_avail > 0)
            {
                int v = *(uint8_t *)cmd->buffer;
                cmd->bytes_consumed = cmd->bytes_avail; 
                memsetb((__far void *)cmd->arg0, v, cmd->arg1);
            }
            break;
        }
        case CMD_FILL_WORDS:
        {
            //if (cmd->is_new) snprintf(last_cmd, sizeof(last_cmd), "FILL %X WORDS @ %08X", cmd->arg1, cmd->arg0);
            if (cmd->bytes_avail > 1)
            {
                uint16_t v = *(uint16_t *)cmd->buffer;
                cmd->bytes_consumed = cmd->bytes_avail; 
                memsetw((__far void *)cmd->arg0, v, cmd->arg1);
            }
            break;
        }
        
        case CMD_OUT_WORD:
        {
            __outw(cmd->arg0, cmd->arg1);
            break;
        }

        case CMD_IN_WORD:
        {
            uint16_t val = __inw(cmd->arg0);
            comms_write(&val, 2);
            break;
        }
        
        case CMD_IDLE: break;

        default:
            cmd->bytes_consumed = cmd->bytes_avail;
            break;
    }
}

Cmd active_cmd;


typedef struct
{
    uint16_t y : 11;
    uint16_t height : 2;
    uint16_t pad0 : 3;
    uint16_t sprite : 15;
    uint16_t pad1 : 1;
    uint16_t color : 7;
    uint16_t prio : 1;
    uint16_t flipx : 1;
    uint16_t flipy : 1;
    uint16_t pad2 : 6;
    uint16_t x : 11;
    uint16_t pad3 : 5;
} ObjInst;

typedef struct
{
    uint16_t hdr0;
    uint16_t hdr1;
    uint16_t hdr2;
    uint16_t hdr3;

    ObjInst insts[0x1ff];
    uint16_t colors[2048];
} ObjPalBuffer;

_Static_assert(sizeof(ObjInst) == 8, "sizeof(ObjInst) != 4");
_Static_assert(sizeof(ObjPalBuffer) == 8192, "sizeof(ObjPalBuffer) != 2048");

static __far ObjPalBuffer *BUFFER = (__far ObjPalBuffer *)0xf8000000;
static __far uint16_t *BUFFER_U16 = (__far uint16_t *)0xf8000000;

__far uint16_t *palette_ram = (__far uint16_t *)0xf0009000;

uint16_t base_palette[] = {
    // Light Gray
    0x0000, 0x7FFF, 0x7FFF, 0x77BD, 0x6F7B, 0x5EF7, 0x56B5, 0x4E73,
    0x7FFD, 0x7F93, 0x7ECD, 0x7E28, 0x79A3, 0x6940, 0x2108, 0x0C63,

    // Red
    0x0000, 0x73FF, 0x4F3F, 0x329F, 0x29FF, 0x297F, 0x08DD, 0x1419,
    0x53FF, 0x3B5F, 0x269F, 0x0DDE, 0x00FE, 0x005A, 0x0010, 0x0007,

    // Orange
    0x0000, 0x7FFF, 0x3FFF, 0x03FF, 0x02FF, 0x01FF, 0x015F, 0x001F,
    0x03F4, 0x0327, 0x02A4, 0x7FE0, 0x5AC0, 0x35A0, 0x2108, 0x0C63,

    // Light Blue
    0x0000, 0x7FFF, 0x7FF6, 0x7FF2, 0x7FCD, 0x736A, 0x6707, 0x5EC7,
    0x5687, 0x4E45, 0x45E4, 0x3DA2, 0x3541, 0x28E0, 0x574B, 0x1000,

    // Green
    0x0000, 0x77FF, 0x53FD, 0x3BD7, 0x2F90, 0x1B09, 0x0280, 0x01E0,
    0x4E73, 0x3DEF, 0x2D6B, 0x227F, 0x7FFF, 0x7FFF, 0x2108, 0x0C63,

    // Dark Blue
    0x0000, 0x7FFF, 0x7F2C, 0x7E43, 0x7DA0, 0x7CE0, 0x6440, 0x4000,
    0x3000, 0x2F7F, 0x0E1F, 0x011F, 0x040E, 0x0407, 0x0035, 0x0000,

    // Light yellow
    0x01CA, 0x7FFF, 0x7BFF, 0x73FF, 0x5BFF, 0x37FF, 0x17FF, 0x039F,
    0x031F, 0x029F, 0x021F, 0x019F, 0x011F, 0x009F, 0x001F, 0x0424,
};

volatile uint32_t vblank_count = 0;
__attribute__((interrupt)) void __far vblank_handler()
{
	vblank_count++;
    return;
}

void wait_vblank()
{
    uint32_t cnt = vblank_count;
    while( cnt == vblank_count ) {}
}

bool test_memory_region(__far void *region, uint16_t count)
{
    __far uint16_t *ptr = (__far uint16_t *)region;
    
    for( uint16_t initial = 0x5; initial != 0x0; initial-- )
    {
        uint16_t second = 0xd;
        uint16_t val = initial;
        for( uint16_t x = 0; x < count; x++ )
        {
            ptr[x] = val;
            val++;
            second--;
            if( second == 0)
            {
                val++;
                second = 0xd;
            }
        }

        val = initial;
        second = 0xd;

        for( uint16_t x = 0; x < count; x++ )
        {
            if( ptr[x] != val ) return false;

            val++;
            second--;
            if( second == 0)
            {
                val++;
                second = 0xd;
            }
        }
    }

    return true;
}

char tmp[64];

extern void vram_timing();

typedef enum
{
    COMMS = 0,
    PF_BASIC,
    PF_DBG,
    BUFRAM,
    SPRITE,
    PRIORITY,

    NUM_TEST_MODES
} TestMode;

TestMode current_mode = BUFRAM;

void init_comms_test()
{
    pf_reset();

    memcpyw(palette_ram, base_palette, sizeof(base_palette) >> 1);

    __outw(0xb0, 0x0800);
    __outw(0x04, 0x0800);

    __outw(0x98, 0x0000);


    pf_enable(0, true);
}

void update_comms_test()
{
    if (comms_update() )
    {
        update_cmd(&active_cmd);
        process_cmd(&active_cmd);
    }

    comms_status(tmp, sizeof(tmp));
    pf_text(0, 6, 2, 2, tmp);
}

void init_pf_test()
{
    pf_reset();

    memcpyw(palette_ram, base_palette, sizeof(base_palette) >> 1);

    memsetw(VRAM + (0xf000 >> 1), 0x08f0, 0x800);

    __outw(0xb0, 0x0800);
    __outw(0x04, 0x0800);

    __outw(0x98, 0x0002);

    pf_enable(0, true);
    pf_enable(1, true);
    pf_enable(2, true);
    pf_enable(3, true);

    for( int i = 0; i < 4; i++ )
    {
        pf_sym(i, i, i, i, 0x10);
        pf_sym(i, i, 28 - i, i, 0x11);
        pf_sym(i, i, 28 - i, 39 - i, 0x13);
        pf_sym(i, i, i, 39 - i, 0x12);
    }

    pf_text(0, 0, 10, 10, "LAYER 1");
    pf_text(1, 1, 10, 10, "LAYER 2");
    pf_text(2, 2, 10, 10, "LAYER 3");
    pf_text(3, 3, 10, 9, "LAYER 4");

    pf_text(1, 1, 8, 15, "NO  SCROLL");
    pf_text(3, 3, 8, 15, "ROW SCROLL");

    pf_text(1, 1, 8, 18, "NO  SELECT");
    pf_text(2, 2, 8, 17, "ROW SELECT");

    pf_set_flags(3, PF_ROWSCROLL);
    pf_set_flags(2, PF_ROWSELECT);

    __far uint16_t *sel = pf_rowselect_addr(2);
    __far uint16_t *scroll = pf_rowscroll_addr(3);

    uint16_t ofs = 0;
    for( int r = 8 * 8; r < 8 * 20; r++ )
    {
        scroll[r] = ofs >> 3;
        ofs--;
    } 

    ofs = 0;
    for( int r = 8 * 8; r < 12 * 8; r++ )
    {
        sel[r] = ofs;
        ofs--;
    } 
    for( int r = 12 * 8; r < 20 * 8; r++ )
    {
        sel[r] = ofs;
        ofs++;
    } 

}

void update_pf_test()
{
}

void init_pf_debug_test()
{
    pf_reset();

    memcpyw(palette_ram, base_palette, sizeof(base_palette) >> 1);

    __outw(0xb0, 0x0800);
    __outw(0x04, 0x0800);

    __outw(0x98, 0x0000);

    pf_enable(0, true);
    pf_enable(3, true);

    pf_set_xy(3, 0, 0);

    pf_set_flags(3, PF_DEBUG);
}

void update_pf_debug_test()
{

    if (comms_update() )
    {
        update_cmd(&active_cmd);
        process_cmd(&active_cmd);
    }
    static uint16_t accel = 0;
    if (input_down(LEFT | RIGHT | UP | DOWN))
    {
        accel = accel + 1;
        if (accel > 127) accel = 127;
    }
    else
    {
        accel = 0;
    }

    uint16_t x = pf_get_x(3);
    uint16_t y = pf_get_y(3);


    if (input_down(LEFT)) x = x + (accel >> 2);
    if (input_down(RIGHT)) x = x - (accel >> 2);
    if (input_down(UP)) y = y + (accel >> 2);
    if (input_down(DOWN)) y = y - (accel >> 2);

    snprintf(tmp, sizeof(tmp), "X: %04X   Y: %04X", x, y);
    pf_text(0, 3, 10, 10, tmp);

    pf_set_xy(3, x, y);
}

void init_bufram_test()
{
    pf_reset();

    memsetw(BUFFER, 0, sizeof(ObjPalBuffer) / 2);

    memcpyw(palette_ram, base_palette, sizeof(base_palette) >> 1);

    __outw(0xb0, 0x0800);
    __outw(0x04, 0x0800);

    __outw(0x98, 0x0000);

    pf_enable(0, true);

    pf_text(0, 1, 10, 10, "BUFRAM TEST");
}

void update_bufram_test()
{
    static uint16_t ctrl = 0;
    static bool run_test = false;
    static uint16_t valid = 0x0;
    if (run_test)
    {
        valid = 0x0;
        __outw(0xb0, 0);
        if( test_memory_region(BUFFER_U16, 0x1000) ) valid |= 0x1;
        __outw(0xb0, 1);
        if( test_memory_region(BUFFER_U16, 0x800) ) valid |= 0x2;
        __outw(0xb0, 2);
        if( test_memory_region(BUFFER_U16, 0x1000) ) valid |= 0x4;
        __outw(0xb0, 3);
        if( test_memory_region(BUFFER_U16, 0x400) ) valid |= 0x8;
        __outw(0xb0, 4);
        if( test_memory_region(BUFFER_U16, 0x400) ) valid |= 0x10;
        __outw(0xb0, 0);

        run_test = false;
    }

    if (input_pressed(UP)) ctrl += 1;
    if (input_pressed(DOWN)) ctrl -= 1;
    if (input_pressed(LEFT)) ctrl <<= 1;
    if (input_pressed(RIGHT)) ctrl >>= 1;

    if (input_pressed(UP)) run_test = true;

    snprintf(tmp, sizeof(tmp), "VALID: %04X", valid);
    pf_text(0, 1, 5, 14, tmp);
}


void init_sprite_test()
{
    pf_reset();

    __outw(0x98, 0x0000);

    pf_enable(0, true);
}

void init_priority_test()
{
    pf_reset();

    __outw(0x98, 0x0000);

    pf_enable(0, true);
    pf_enable(1, true);
    pf_enable(2, true);
    pf_enable(3, true);
}

void update_priority_test()
{
    while ((__inw(0x02) & 0x0080) == 0x0000) {}

    __outw(0xb0, 0x800);
    
    // clear sprite
    memsetw(BUFFER->insts, 0xe000, sizeof(BUFFER->insts) / 2);

    BUFFER->hdr0 = 248;
    BUFFER->hdr1 = 518;
    BUFFER->hdr2 = 0;
    BUFFER->hdr3 = 64;

    __far ObjInst *inst = BUFFER->insts;

    inst->y = 308;
    inst->height = 0;
    inst->pad0 = 3;
    inst->sprite = 6196;
    inst->pad1 = 0;
    inst->color = 30;
    inst->prio = 1;
    inst->flipx = 0;
    inst->flipy = 0;
    inst->pad2 = 32;
    inst->x = 164;
    inst->pad3 = 16;

    inst++;

    inst->y = 308;
    inst->height = 0;
    inst->pad0 = 3;
    inst->sprite = 6196;
    inst->pad1 = 0;
    inst->color = 30;
    inst->prio = 1;
    inst->flipx = 0;
    inst->flipy = 1;
    inst->pad2 = 32;
    inst->x = 324;
    inst->pad3 = 16;

    inst++;

    inst->y = 196;
    inst->height = 0;
    inst->pad0 = 3;
    inst->sprite = 6196;
    inst->pad1 = 0;
    inst->color = 30;
    inst->prio = 0;
    inst->flipx = 1;
    inst->flipy = 1;
    inst->pad2 = 0x0;
    inst->x = 340;
    inst->pad3 = 0xf;

    inst++;

    inst->y = 196;
    inst->height = 0;
    inst->pad0 = 3;
    inst->sprite = 6196;
    inst->pad1 = 0;
    inst->color = 30;
    inst->prio = 0;
    inst->flipx = 1;
    inst->flipy = 0;
    inst->pad2 = 32;
    inst->x = 180;
    inst->pad3 = 16;

    wait_vblank();

    __outw(0xb0, 0x800);
    __outw(0x04, 0x800);

    for( int x = 0; x < 30; x++)
    {
        for( int y = 0; y < 41; y++)
            pf_sym(3, 0, x, y, (y^x) & 1 ? 0x7 : 0x4);
    }

/*    pf_text(0, 2, 0, 19, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    pf_text(1, 1, 0, 20, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    pf_text(2, 1, 0, 21, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    //pf_text(3, 1, 0, 22, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");

    pf_text(0, 1 | PF_PRIO0, 0, 24, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    pf_text(1, 1 | PF_PRIO0, 0, 25, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    pf_text(2, 1 | PF_PRIO0, 0, 26, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
    //pf_text(3, 1 | PF_PRIO0, 0, 27, "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXX");
*/
}

void init_mode()
{
    switch(current_mode)
    {
        case COMMS:
            init_comms_test();
            break;

        case PF_BASIC:
            init_pf_test();
            break;
        
        case PF_DBG:
            init_pf_debug_test();
            break;

        case BUFRAM:
            init_bufram_test();
            break;

        case SPRITE:
            init_sprite_test();
            break;

        case PRIORITY:
            init_priority_test();
            break;

        default:
            break;
    }
}

void update_mode()
{
    switch(current_mode)
    {
        case COMMS:
            update_comms_test();
            break;

        case PF_BASIC:
            update_pf_test();
            break;

        case PF_DBG:
            update_pf_debug_test();
            break;

        case BUFRAM:
            update_bufram_test();
            break;

        case SPRITE:
            break;

        case PRIORITY:
            break;

        default:
            break;
    }
}

int main()
{
    __outb(0x40, 0x13);
    __outb(0x42, 0x08);
    __outb(0x42, 0x0f);
    __outb(0x42, 0xf2);

    memset(&active_cmd, 0, sizeof(active_cmd));
    last_cmd[0] = 0;

    //memsetw((__far void *)0xf0008000, 0x0000, 0x2000);

    enable_interrupts();

    init_pf_test();

    while(1)
    {
        input_update();

        update_pf_test();
        
        wait_vblank();

    }

    return 0;
}

