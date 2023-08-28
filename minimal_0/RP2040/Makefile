include ../Makefile.inc

SRC_FILES = $(wildcard aeabi-cortexm0/*.S)

OUTDIR  = ../$(OBJDIR)

TARGET 	= $(notdir $(CURDIR))

OUT_LIB = $(OUTDIR)/lib$(TARGET).a

$(OUTDIR)/%.o: %.S
	-@mkdir -p $(OUTDIR)/$(dir $@)
	@$(AS) -mthumb $< -o $@

$(OUT_LIB): $(SRC_FILES:%.S=$(OUTDIR)/%.o)
	@echo [AR] $(OUT_LIB)
	@$(AR) rcs $(OUT_LIB) $(SRC_FILES:%.S=$(OUTDIR)/%.o)


all: $(OUT_LIB)

clean:
	-@rm -rf $(OUTDIR)



