CC = ia16-elf-gcc
NASM = nasm
OBJCOPY = ia16-elf-objcopy
MAME = ../util/irem_emu
SPLIT_ROM = ../util/split_rom.py
MISTER=root@mister-dev

TARGET = testbed

C_SRCS = main.c comms.c interrupts_default.c init.c input.c playfield.c printf/printf.c
ASM_SRCS = entry.S
NASM_SRCS = 

BUILD_DIR = build/$(TARGET)
ORIGINAL_DIR = ../roms

OBJS = $(addprefix $(BUILD_DIR)/, $(C_SRCS:c=o) $(ASM_SRCS:S=o) $(NASM_SRCS:asm=o))
BUILD_DIRS = $(sort $(dir $(OBJS))) 
GLOBAL_DEPS = Makefile

DEFINES = -DPRINTF_SUPPORT_DECIMAL_SPECIFIERS=0 \
	-DPRINTF_SUPPORT_EXPONENTIAL_SPECIFIERS=0 \
	-DPRINTF_SUPPORT_LONG_LONG=0 \
	-DPRINTF_ALIAS_STANDARD_FUNCTION_NAMES=1 \
	-DPRINTF_ALIAS_STANDARD_FUNCTION_NAMES_HARD=1

CFLAGS = -march=v30 -mcmodel=small -ffreestanding $(DEFINES) -O2
LIBS = -lgcc
LDFLAGS = $(CFLAGS) -static -nostdlib

ifeq ($(TARGET),testbed)
GAME = dbreed
CPU_ROM_L0 = db_a-l0-.ic68
CPU_ROM_H0 = db_a-h0-.ic59
CPU_ROM_SIZE = 0x20000
else
error
endif

ORIGINAL ?= 0
GAME_DIR = $(BUILD_DIR)/$(GAME)
BUILT_BINS = $(addprefix $(GAME_DIR)/, $(CPU_ROM_L0) $(CPU_ROM_H0))

ifeq ($(ORIGINAL),0)
ROMPATH = ../$(BUILD_DIR);../$(ORIGINAL_DIR)
else
ROMPATH = ../$(ORIGINAL_DIR)
endif

all: $(BUILT_BINS)

$(BUILD_DIR)/cpu.bin: $(BUILD_DIR)/cpu.elf
	$(OBJCOPY) -O binary --change-section-lma .data=0x10000 $< $@

$(GAME_DIR)/$(CPU_ROM_H0): $(BUILD_DIR)/cpu.bin $(SPLIT_ROM) | $(GAME_DIR)
	$(SPLIT_ROM) $@ $< 2 1 $(CPU_ROM_SIZE)

$(GAME_DIR)/$(CPU_ROM_L0): $(BUILD_DIR)/cpu.bin $(SPLIT_ROM) | $(GAME_DIR)
	$(SPLIT_ROM) $@ $< 2 0 $(CPU_ROM_SIZE)


$(BUILD_DIR)/%.o: %.c $(GLOBAL_DEPS) | $(BUILD_DIRS)
	@echo $@
	@$(CC) -MMD -o $@ $(CFLAGS) -c $<

$(BUILD_DIR)/%.o: %.S $(GLOBAL_DEPS) | $(BUILD_DIRS)
	@echo $@
	@$(CC) -MMD -o $@ $(CFLAGS) -c $<

$(BUILD_DIR)/%.o: %.asm $(GLOBAL_DEPS) | $(BUILD_DIRS)
	@echo $@
	@$(NASM) -MD $($@:o=d) -o $@ -f elf -D_TEXT=.text -D_BSS=.bss -D_DATA=.data $<

$(BUILD_DIR)/cpu.elf: $(OBJS) $(GAME).ld
	@echo $@
	@$(CC) -T $(GAME).ld -o $@ $(LDFLAGS) $(OBJS) $(LIBS)
	ia16-elf-objdump -d -M i8086,intel,intel-mnemonic $@ > $(BUILD_DIR)/cpu.asm

$(BUILD_DIRS):
	mkdir -p $@

$(GAME_DIR):
	mkdir -p $@

.PHONY: run debug


debug: $(BUILT_BINS)
	mkdir -p mame
	cd mame && ../$(MAME) -window -nomaximize -resolution0 640x480 -debug -rompath "$(ROMPATH)" $(GAME)

trace: $(BUILT_BINS)
	mkdir -p mame
	cd mame && ../$(MAME) -window -nomaximize -resolution0 640x480 -debug -debugscript ../add.scr -rompath "$(ROMPATH)" $(GAME)

run: $(BUILT_BINS)
	mkdir -p mame
	cd mame && ../$(MAME) -window -nomaximize -resolution0 640x480 -rompath "$(ROMPATH)" $(GAME)

mister: $(BUILD_DIR)/cpu.bin src/m107_test.mra
	zip -j $(BUILD_DIR)/m107_test.zip $(BUILD_DIR)/cpu.bin
	scp $(BUILD_DIR)/m107_test.zip $(MISTER):/media/fat/games/mame/
	scp src/m107_test.mra $(MISTER):/media/fat/_Arcade/
	ssh $(MISTER) "echo load_core _Arcade/m107_test.mra > /dev/MiSTer_cmd"

picorom: $(GAME_DIR)/$(CPU_ROM_H0) $(GAME_DIR)/$(CPU_ROM_L0)
	picorom upload cpu_l0 $(GAME_DIR)/$(CPU_ROM_L0) 512kbit
	picorom upload cpu_h0 $(GAME_DIR)/$(CPU_ROM_H0) 512kbit

-include $(OBJS:o=d)
