UNAME=$(shell uname -s)
ifeq ($(UNAME),Linux)
	OS := Linux
endif
ifeq ($(UNAME),Darwin)
	OS := OSX
endif

OBJ_DIR = obj
VERILATED_DIR = verilated

# Use the generated tv80 with auto_ss savestate ports (rtl/tv80_auto_ss.sv)
USE_AUTO_SS = 1

GAME ?= hharryu

VERILATOR = verilator
VERILATOR_ARGS = --cc --make gmake --no-timing --trace-fst --vpi --Mdir $(VERILATED_DIR) --MMD --MP \
				-CFLAGS -Wno-unknown-warning-option \
				-DV30_BACKDOOR +incdir+../rtl/v30 +incdir+../rtl/nu8051 \
				-Wno-TIMESCALEMOD \
				-Wno-STMTDLY \
				-Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
				-Wno-CASEINCOMPLETE -Wno-UNUSEDSIGNAL

PYTHON = python3

VERILATOR_INC = $(shell pkg-config --variable=includedir verilator)
VERILATOR_CPP = verilated.cpp verilated_fst_c.cpp verilated_threads.cpp verilated_vpi.cpp
VERILATOR_OBJS = $(patsubst %.cpp, $(OBJ_DIR)/verilator/%.o, $(VERILATOR_CPP))

SRCS = imgui/imgui.cpp \
		imgui/imgui_draw.cpp \
		imgui/imgui_tables.cpp \
		imgui/imgui_widgets.cpp \
		imgui/backends/imgui_impl_sdl2.cpp \
		imgui/backends/imgui_impl_sdlrenderer2.cpp \
		sim_state.cpp \
		sim_main.cpp \
		sim_core.cpp \
		sim_audio_capture.cpp \
		testrom_gui.cpp \
		sim_ui.cpp \
		sim_hw_ui.cpp \
		sim_video.cpp \
		sim_controller.cpp \
		sim_protocol.cpp \
		sim_server.cpp \
		games.cpp \
		imgui_wrap.cpp \
		third_party/miniz.cpp \
		third_party/pugixml.cpp \
		file_search.cpp \
		mra_loader.cpp

OBJS = $(patsubst %.cpp, $(OBJ_DIR)/%.o, $(SRCS))

DEPFLAGS = -MT $@ -MMD -MP -MF $(OBJ_DIR)/$*.d

CC=cc
CXX=c++
CPPFLAGS= --std=gnu++17 -g $(DEPFLAGS) -O2
CPPFLAGS+=-I$(VERILATED_DIR) $(shell pkg-config --cflags verilator)
CPPFLAGS+=$(shell pkg-config --cflags sdl2)
CPPFLAGS+=-Iimgui/ -Iimgui/backends/

# Optional: capstone for V30 disassembly in the CPU window
HAVE_CAPSTONE := $(shell pkg-config --exists capstone && echo 1)
ifeq ($(HAVE_CAPSTONE),1)
CPPFLAGS+=-DHAVE_CAPSTONE $(shell pkg-config --cflags capstone)
CAPSTONE_LIBS=$(shell pkg-config --libs capstone)
endif

LDFLAGS=-g

ifeq ($(OS),OSX)
LDFLAGS+=-Wl,-U,__Z15vl_time_stamp64v,-U,__Z13sc_time_stampv
endif

LDLIBS=$(shell pkg-config --libs sdl2) $(CAPSTONE_LIBS)

# The V30 and the nu8051 MCU are now native SystemVerilog (rtl/v30/*.sv,
# rtl/nu8051/*.sv), no netlist needed. No generated netlists remain.
NETLISTS =

# ddr_debug.sv first: it declares the file-scope ddr_debug_data_t typedef
HDL_SRC  = ../rtl/ddr_debug.sv ../rtl/m72_pkg.sv
# Savestate infrastructure: ssbus_if/ddr_if interfaces + streamer + primitives
HDL_SRC += ../rtl/savestates.sv ../rtl/ddram.sv ../rtl/memory_stream.sv ../rtl/ram.sv
# V30 ucore (package first) + microcode ROM + bus adapter
HDL_SRC += ../rtl/v30/v30u_ss_pkg.sv ../rtl/v30/v30_core.sv
HDL_SRC += ../rtl/v30/v30u_biu.sv ../rtl/v30/v30u_ucrom.sv ../rtl/v30/v30u_eu.sv
HDL_SRC += ../rtl/v30/v30_bus.sv
V30_UCORE_DEPS = $(wildcard ../rtl/v30/*.svh) ../rtl/v30/ucdecode.hex ../rtl/v30/ucrom.hex
# nu8051 MCU core (package first)
HDL_SRC += ../rtl/nu8051/nu8051_ss_pkg.sv ../rtl/nu8051/nu8051_core.sv
HDL_SRC += ../rtl/nu8051/nu8051_seq.sv ../rtl/nu8051/nu8051_alu.sv
HDL_SRC += ../rtl/nu8051/nu8051_iram.sv ../rtl/nu8051/nu8051_sfr.sv
HDL_SRC += ../rtl/nu8051/nu8051_ports.sv ../rtl/nu8051/nu8051_timer.sv
HDL_SRC += ../rtl/nu8051/nu8051_uart.sv ../rtl/nu8051/nu8051_irq.sv
HDL_SRC += ../rtl/m72.v ../rtl/m72_pic.sv ../rtl/pal.sv
HDL_SRC += ../rtl/kna70h015.sv ../rtl/kna91h014.v ../rtl/kna6034201.v
HDL_SRC += ../rtl/board_b_d.sv ../rtl/board_b_d_layer.sv ../rtl/board_b_d_sdram.sv
HDL_SRC += ../rtl/sprite.sv
HDL_SRC += ../rtl/sound.sv ../rtl/sample_rom.sv
HDL_SRC += ../rtl/mcu.sv ../rtl/mcu_emulator.sv ../rtl/dualport_mailbox.sv
HDL_SRC += ../rtl/dpramv.sv ../rtl/rom.sv ../rtl/jtframe_frac_cen.v
ifeq ($(USE_AUTO_SS),1)
VERILATOR_ARGS += -DUSE_AUTO_SS
HDL_SRC += ../rtl/tv80_auto_ss.sv
HDL_SRC += ../rtl/jt51_auto_ss.sv
else
HDL_SRC += ../rtl/tv80/tv80s.v ../rtl/tv80/tv80_core.v ../rtl/tv80/tv80_alu.v ../rtl/tv80/tv80_mcode.v ../rtl/tv80/tv80_reg.v
HDL_SRC += $(wildcard ../rtl/jt51/jt51*.v)
endif
HDL_SRC += ../sys/iir_filter.v
HDL_SRC += $(NETLISTS)
HDL_SRC += sim_top.sv
HDL_SRC += verilator.vlt

all: sim

.PHONY: netlists
netlists:
	scripts/gen_vhdl_netlists.sh

$(VERILATED_DIR)/M72.mk: $(HDL_SRC) $(V30_UCORE_DEPS) Makefile
	$(VERILATOR) $(VERILATOR_ARGS) -o M72 --prefix M72 --top sim_top $(HDL_SRC)

$(VERILATED_DIR)/M72__ALL.a: $(VERILATED_DIR)/M72.mk $(HDL_SRC) $(V30_UCORE_DEPS)
	$(MAKE) -C $(VERILATED_DIR) -f M72.mk

$(OBJ_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) -o $@ -c $< $(CPPFLAGS)

$(OBJ_DIR)/verilator/%.o: $(VERILATOR_INC)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) -o $@ -c $< $(CPPFLAGS)

$(OBJS): $(VERILATED_DIR)/M72__ALL.a

sim: $(OBJS) $(VERILATOR_OBJS) $(VERILATED_DIR)/M72__ALL.a
	$(CXX) -o $@ $^ $(CPPFLAGS) $(LDFLAGS) $(LDLIBS) -lpthread -lz

validate_mra: validate_mra.cpp mra_loader.cpp file_search.cpp third_party/miniz.cpp third_party/pugixml.cpp
	$(CXX) -o $@ $^ $(CPPFLAGS) $(LDFLAGS) -lz

run: sim
	./sim $(GAME)

.PHONY: clean all run format

format:
	clang-format -i *.cpp *.h

clean:
	rm -r $(OBJ_DIR) $(VERILATED_DIR)

DEPFILES := $(SRCS:%.cpp=$(OBJ_DIR)/%.d)
$(DEPFILES):
-include $(wildcard $(DEPFILES))
