MAKEFLAGS+=w
PYTHON=uv run
QUARTUS_DIR = C:/intelFPGA_lite/17.0/quartus/bin64
PROJECT = Arcade-IremM72
CONFIG = Arcade-IremM72
MISTER = root@mister-dev
OUTDIR = output_files
RELEASES_DIR = releases

# Use wsl for submakes on windows
ifeq ($(OS),Windows_NT)
MAKE = wsl make
endif

RBF = $(OUTDIR)/$(CONFIG).rbf

.DEFAULT_GOAL := rbf

rwildcard=$(foreach d,$(wildcard $(1:=/*)),$(call rwildcard,$d,$2) $(filter $(subst *,%,$2),$d))

SRCS_FULL = \
	$(call rwildcard,sys,*.v *.sv *.vhd *.vhdl *.qip *.sdc) \
	$(call rwildcard,rtl,*.v *.sv *.vhd *.vhdl *.qip *.sdc) \
	$(wildcard *.sdc *.v *.sv *.vhd *.vhdl *.qip)

SRCS = $(filter-out %_auto_ss.sv,$(SRCS_FULL))

PROJECT_FILES = Arcade-IremM72.qsf Arcade-IremM72-Fast.qsf Arcade-IremM72.qpf

# quartus_sh --flow compile saves the project on the way out, which appends
# every resolved assignment - the whole of files.qip and sys/sys.tcl - inline
# after the `source files.qip` line that already provides them. That stale copy
# then silently overrides the real files.qip: it is what re-added jt51.qip
# after the jt51_auto_ss switch and broke the build with duplicate modules.
# Snapshot the project files and restore them once the flow is done, so
# files.qip stays the single source of truth. Deliberate .qsf edits made before
# the build survive; only Quartus' write-back is undone.
define quartus_compile
	@for f in $(PROJECT_FILES); do cp "$$f" "$$f.premake"; done
	-@$(QUARTUS_DIR)/quartus_sh --flow compile $(PROJECT) -c $(1); \
	  status=$$?; \
	  for f in $(PROJECT_FILES); do mv "$$f.premake" "$$f"; done; \
	  exit $$status
endef

$(OUTDIR)/Arcade-IremM72-Fast.rbf: $(SRCS)
	$(call quartus_compile,Arcade-IremM72-Fast)

$(OUTDIR)/Arcade-IremM72.rbf: $(SRCS)
	$(call quartus_compile,Arcade-IremM72)

rbf: $(OUTDIR)/$(CONFIG).rbf

# Quick syntax/elaboration check without a full fit
analyze:
	$(QUARTUS_DIR)/quartus_map --analyze_file_and_save_all_files $(PROJECT) -c $(CONFIG)

deploy.done: $(RBF)
	scp $(RBF) $(MISTER):/media/fat/_Development/cores/IremM72.rbf
	echo done > deploy.done

deploy: deploy.done

mister/%: $(RELEASES_DIR)/% deploy.done
	scp "$<" $(MISTER):/media/fat/_Development/
	ssh $(MISTER) "echo \"load_core _Development/$(notdir $<)\" > /dev/MiSTer_cmd"

mister: mister/rtype
mister/rtype: mister/R-Type\ (World).mra
mister/rtype2: mister/R-Type\ II\ (World).mra
mister/nspirit: mister/Ninja\ Spirit\ (Japan).mra
mister/imgfight: mister/Image\ Fight\ (World).mra
mister/hharryu: mister/Hammerin'\ Harry\ (US,\ M84\ hardware).mra
mister/gallop: mister/Gallop\ -\ Armed\ Police\ Unit\ (Japan,\ M72\ hardware).mra
mister/loht: mister/Legend\ of\ Hero\ Tonma\ (Japan).mra
mister/xmultipl: mister/X\ Multiply\ (Japan,\ M72\ hardware).mra
mister/dbreed: mister/Dragon\ Breed\ (Japan,\ M72\ hardware).mra
mister/airduel: mister/Air\ Duel\ (World,\ M72\ hardware).mra
mister/mrheli: mister/Mr.\ HELI\ no\ Daibouken\ (Japan).mra

sim:
	$(MAKE) -j8 -C sim sim

sim/run: sim/rtype

sim/%:
	$(MAKE) -j8 -C sim run GAME=$*

rtl/tv80_auto_ss.sv:
	$(PYTHON) util/state_module.py --generate-csv docs/tv80_mapping.csv tv80s rtl/tv80_auto_ss.sv rtl/tv80/*.v

clean:
	rm -rf $(OUTDIR) db incremental_db deploy.done

.PHONY: rbf analyze deploy sim sim/run mister rtl/tv80_auto_ss.sv clean
