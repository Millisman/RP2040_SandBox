include Makefile.inc

LIBS = RP2040 app2

SRC_DIRS = . src src/*

LIBA = $(patsubst %, $(OBJDIR)/lib%.a, $(LIBS))
LIBL = -L$(OBJDIR) $(patsubst %, -l%, $(LIBS))

CFILES  = $(patsubst ./%, %, $(foreach D, $(SRC_DIRS), $(wildcard $(D)/*.c)))
SFILES  = $(patsubst ./%, %, $(foreach D, $(SRC_DIRS), $(wildcard $(D)/*.S)))
SFILES += $(BLDR_ASM)

OFILES  = $(addprefix $(OBJDIR)/, $(patsubst %.c, %.o, $(CFILES)))
OFILES += $(addprefix $(OBJDIR)/, $(patsubst %.S, %.o, $(SFILES)))

DEPS    = $(addprefix $(OBJDIR)/, $(patsubst %.c, %.d, $(CFILES)))

TARGET 	= $(notdir $(CURDIR))
PRELF 	= $(addprefix $(OBJDIR)/, $(TARGET).elf)

all: $(OBJDIR) $(TARGET).uf2

# elf2 --> uf2
E2U2   = $(OBJDIR)/elf2uf2
E2U2_SRC = RP2040/elf2uf2
$(E2U2): $(E2U2_SRC)
	@echo [HOST CPP] $@
	@g++ -std=c++14 -O2 -I$(E2U2_SRC) $(E2U2_SRC)/main.cpp -o $@

$(OBJDIR)/%.a: force_look
	@cd $(patsubst lib%, %, $(basename $(notdir $@)) ; $(MAKE) $(MFLAGS))

$(OBJDIR)/%.o: %.c
	@echo [C] $<
	-@mkdir -p $(dir $@)
	@$(C) $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.o: %.S
	@echo [AS] $<
	-@mkdir -p $(dir $@)
	@$(C) $(CFLAGS) -c $< -o $@

$(PRELF): $(LIBA) $(OFILES)
	@echo [LINK] $(notdir $@)
	@$(C) $(LFLAGS) $^ $(LIBL) -o $@
	@$(OBJDUMP) -d $@ > $(TARGET).dis


$(TARGET).uf2: $(PRELF) $(E2U2)
	@echo [ELF2UF2] $@
	@$(E2U2) $< $@

force_look:
	@true

$(OBJDIR):
	-@mkdir -p $@

clean:
	-@for lib in $(LIBS); do (cd $$lib; $(MAKE) clean); done
	-@rm $(E2U2)
	-@rm $(PRELF)
	-@rm $(TARGET).uf2
	-@rm $(TARGET).dis
	-@rm -rf $(OBJDIR)

upload: $(TARGET).uf2
	@cp $< /run/media/user/RPI-RP2/

-include $(DEPS)
