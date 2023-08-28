include ../Makefile.inc

SRCDIR = .

SHAREDINC = -I. -I./include -I../RP2040


SOURCE = $(wildcard $(SRCDIR)/*.c)
OBJS = $(patsubst $(SRCDIR)%, $(OBJDIR)%, $(SOURCE:.c=.o))
DEPS = $(patsubst $(OBJDIR)%, $(DEPDIR)%, $(OBJS:.o=.d))

OUTDIR = ../$(OBJDIR)
TARGET = $(OUTDIR)/lib$(notdir $(CURDIR)).a

all: $(OUTDIR) $(TARGET)

clean:
	-@rm $(TARGET)
	-@rm -rf .dep
	-@rm -rf .obj

$(OUTDIR):
	-@mkdir -p $@

$(TARGET): $(OBJS)
	@echo [LIB] $(notdir $@)
	@$(AR) rcs $@ $^

$(DEPDIR)/%.d: %.c
	-@mkdir -p $(dir $@)
	@$(C) $(CFLAGS) $(SHAREDINC) -MM $< > $@
	@sed -i -e "s,$*.o:,$(OBJDIR)/$*.o $@: ,g" $@

$(OBJDIR)/%.o: %.c
	-@mkdir -p $(dir $@)
	@echo [C] $(notdir $<)
	@$(C) -c $(CFLAGS) $(SHAREDINC) $< -o $@

-include $(DEPS)
