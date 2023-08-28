TOOLS    = /home/user/.arduino15/packages/rp2040/tools/pqt-gcc/1.5.0-b-c7bab52/bin/

C  	= $(abspath $(TOOLS)arm-none-eabi-gcc)
AR 	= $(abspath $(TOOLS)arm-none-eabi-ar)
AS 	= $(abspath $(TOOLS)arm-none-eabi-as)
OBJCOPY = $(abspath $(TOOLS)arm-none-eabi-objcopy)
OBJDUMP = $(abspath $(TOOLS)arm-none-eabi-objdump)

BLDR_ASM = RP2040/boot2/boot2_generic_03h_4_padded_checksum.S

DEFINES = RP2040 __PROGRAM_START=notmain XOSC_VALUE=12000000UL

INC_DIRS = . ./include/
INC_DIRS += RP2040
# INC_DIRS += RP2040/CMSIS/5.7.0/Include
# INC_DIRS += RP2040/CMSIS/RP2040/Include/
# INC_DIRS += RP2040/CMSIS/Driver/Include/
# INC_DIRS += RP2040/Drivers/Include/

CPU = cortex-m0plus
OBJDIR = .obj
DEPDIR = .dep

CFLAGS  = -mcpu=$(CPU) -mthumb -Wall -Wextra -O2
CFLAGS += $(foreach INC,$(INC_DIRS), -I$(INC))
CFLAGS += $(foreach DEF,$(DEFINES),  -D$(DEF))

LFLAGS  = -mcpu=$(CPU) -mthumb -nostdlib -Wl,--print-memory-usage --specs=nosys.specs
LFLAGS += -T RP2040/gcc_arm.ld

