//============================================================================
//  Irem M72 for MiSTer FPGA - Common definitions
//
//  Copyright (C) 2022 Martin Donlon
//
//  This program is free software; you can redistribute it and/or modify it
//  under the terms of the GNU General Public License as published by the Free
//  Software Foundation; either version 2 of the License, or (at your option)
//  any later version.
//
//  This program is distributed in the hope that it will be useful, but WITHOUT
//  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
//  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
//  more details.
//
//  You should have received a copy of the GNU General Public License along
//  with this program; if not, write to the Free Software Foundation, Inc.,
//  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//============================================================================

package m72_pkg;

    typedef struct packed {
        bit [24:0] base_addr;
        bit reorder_64;
        bit [4:0] bram_cs;
    } region_t;

    parameter region_t REGION_CPU_ROM = '{ base_addr:'h000_0000, reorder_64:0, bram_cs:5'b00000 };
    parameter region_t REGION_SPRITE =  '{ base_addr:'h010_0000, reorder_64:1, bram_cs:5'b00000 };
    parameter region_t REGION_BG_A =    '{ base_addr:'h100_0000, reorder_64:0, bram_cs:5'b00000 };
    parameter region_t REGION_BG_B =    '{ base_addr:'h020_0000, reorder_64:0, bram_cs:5'b00000 };
    parameter region_t REGION_MCU =     '{ base_addr:'h000_0000, reorder_64:0, bram_cs:5'b00001 };
    parameter region_t REGION_SAMPLES = '{ base_addr:'h000_0000, reorder_64:0, bram_cs:5'b00010 };
    parameter region_t REGION_OFFSETS = '{ base_addr:'h000_0000, reorder_64:0, bram_cs:5'b00100 };
    parameter region_t REGION_PROTECT = '{ base_addr:'h000_0000, reorder_64:0, bram_cs:5'b01000 };
    parameter region_t REGION_SOUND   = '{ base_addr:'h000_0000, reorder_64:0, bram_cs:5'b10000 };

    parameter region_t LOAD_REGIONS[9] = '{
        REGION_CPU_ROM,
        REGION_SPRITE,
        REGION_BG_A,
        REGION_BG_B,
        REGION_MCU,
        REGION_SAMPLES,
        REGION_OFFSETS,
        REGION_PROTECT,
        REGION_SOUND
    };

    parameter region_t REGION_CPU_RAM = '{ 'h400000, 0, 5'b00000 };

    typedef struct packed {
        bit [2:0] reserved;
        bit m84;
        bit main_mculatch;
        bit [2:0] memory_map;
    } board_cfg_t;

    typedef enum bit[1:0] {
        VIDEO_55HZ = 2'd0,
        VIDEO_50HZ = 2'd1,
        VIDEO_57HZ = 2'd2,
        VIDEO_60HZ = 2'd3
    } video_timing_t;

    // Savestate section indices (ssbus chunk ids). R-Type scope: hardware the
    // game doesn't use (MCU, samples, mailbox) has no section.
    parameter int SSIDX_GLOBAL        = 0;   // m72.v: sys_flags, CE counters, paused_v/h
    parameter int SSIDX_WORK_RAM      = 1;   // 64K x 16 CPU work RAM
    parameter int SSIDX_V30           = 2;   // 202 x 16 v30_core register file
    parameter int SSIDX_Z80           = 3;   // tv80_auto_ss via auto_save_adaptor2
    parameter int SSIDX_SOUND_RAM     = 4;   // 64KB Z80 program/work RAM
    parameter int SSIDX_SOUND_REGS    = 5;
    parameter int SSIDX_SPRITE_RAM_L  = 6;   // 512 x 8
    parameter int SSIDX_SPRITE_RAM_H  = 7;
    parameter int SSIDX_SPRITE_OBJRAM = 8;   // 128 x 64 post-DMA sprite table
    parameter int SSIDX_SPRITE_REGS   = 9;
    parameter int SSIDX_LAYER_A_RAM0  = 10;  // ..RAM3 = 13
    parameter int SSIDX_LAYER_A_REGS  = 14;
    parameter int SSIDX_LAYER_B_RAM0  = 15;  // ..RAM3 = 18
    parameter int SSIDX_LAYER_B_REGS  = 19;
    parameter int SSIDX_PAL_BG        = 20;  // kna91h014 in board_b_d
    parameter int SSIDX_PAL_OBJ       = 21;  // kna91h014 in m72.v
    parameter int SSIDX_CRTC          = 22;  // kna70h015
    parameter int SSIDX_PIC           = 23;  // m72_pic
    parameter int SSIDX_JT51          = 24;  // jt51_auto_ss via auto_save_adaptor2
    parameter int SSIDX_VERSION       = 25;  // build stamp
    parameter int SSIDX_COUNT         = 26;

    // DDR window for savestate slots: 4 slots x 4MB from this base
    // (save_state_data hardcodes index * 0x400000, length 0x400000)
    parameter bit [31:0] SS_DDR_BASE  = 32'h3E00_0000;

endpackage