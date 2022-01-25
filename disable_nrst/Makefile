#
#  Generic and Simple GNU ARM Makefile 
#
#  Desinged for the gnu-arm-none-eabi tool chain
#
# 	Features
#		- create hex file
#		- create assembler listing (.dis)
#
#	Limitations
#		- only C-files supported
#		- no automatic dependency checking (call 'make clean' if any .h files are changed)
#
#	Targets:
#		make
#			create hex file, no upload
#		make upload
#			create and upload hex file
#		make clean
#			delete all generated files
#
#  Note:
#  	Display list make database: make -p -f/dev/null | less
#
#================================================
# External tools

# The base directory of gcc-arm-none-eabi
# Can be empty on Ubuntu and installed gcc-arm-none-eabi
# If set, GCCBINPATH must contain a "/" at the end.

# GCCBINPATH:=/usr/bin/   <<<< Ubuntu version is broken at the moment
GCCBINPATH:=/home/kraus/prg/gcc-arm-none-eabi-7-2018-q2-update/bin/

#================================================
# Project Information

# The name for the project
TARGETNAME:=enable_boot0

# The source files of the project
CSRC:=$(wildcard *.c) 
SSRC:=$(wildcard ../sys/*.s)

# The CPU architecture (will be used for -mcpu)
# for the STM32G0, can we use "cortex-m0plus"?
MCPU:=cortex-m0plus

# Include directory for the system include files
SYSINC:=../sys
SYSSRC:=$(wildcard ../sys/*.c)

# Include directory for other common files
COMMONINC:=../common
COMMONSRC:=$(wildcard ../common/*.c)

# Include directory for the u8g2 include files
#U8G2INC:=../../../../csrc/
#U8G2SRC:=$(wildcard ../../../../csrc/*.c)

# directory for FatFS
#FFINC:=../fatfs
#FFSRC:=$(wildcard ../fatfs/*.c)

# Directory for the linker script
LDSCRIPTDIR:=../sys
# Name of the linker script (must be the "keep" script, because the other script is not always working)
# LDSCRIPT:=stm32l072xz.ld
# LDSCRIPT:=$(wildcard *.ld)
LDSCRIPT:=stm32g031x8.ld


#================================================
# Main part of the Makefile starts here. Usually no changes are needed.

# Internal Variable Names
LIBNAME:=$(TARGETNAME).a
ELFNAME:=$(TARGETNAME).elf
HEXNAME:=$(TARGETNAME).hex
DISNAME:=$(TARGETNAME).dis
MAPNAME:=$(TARGETNAME).map
OBJ:=$(CSRC:.c=.o) $(SSRC:.s=.o) $(SYSSRC:.c=.o) $(COMMONSRC:.c=.o)
# $(SYSSRC:.c=.o) $(U8G2SRC:.c=.o) $(FFSRC:.c=.o)

# Replace standard build tools by arm tools
AS:=$(GCCBINPATH)arm-none-eabi-as
CC:=$(GCCBINPATH)arm-none-eabi-gcc
AR:=$(GCCBINPATH)arm-none-eabi-ar
OBJCOPY:=$(GCCBINPATH)arm-none-eabi-objcopy
OBJDUMP:=$(GCCBINPATH)arm-none-eabi-objdump
SIZE:=$(GCCBINPATH)arm-none-eabi-size

# Common flags
COMMON_FLAGS = -mthumb -mcpu=$(MCPU)
COMMON_FLAGS += -DSTM32G031xx
COMMON_FLAGS += -Wall -I. -I$(SYSINC) -I$(COMMONINC) 
# define stack size (defaults to 0x0100)  
# COMMON_FLAGS += -D__STACK_SIZE=0x0100
# COMMON_FLAGS += -Os -flto
COMMON_FLAGS += -Os 
# COMMON_FLAGS += -fstack-protector 
# COMMON_FLAGS += -finstrument-functions
# Do not use stand libs startup code. Uncomment this for gcclib procedures
# memcpy still works, but might be required for __aeabi_uidiv
# COMMON_FLAGS +=  -nostdlib
# remove unused data and function
#COMMON_FLAGS += -ffunction-sections -fdata-sections -fshort-wchar
COMMON_FLAGS += -ffunction-sections -fdata-sections -DUSER_VECT_TAB_ADDRESS
# C flags
CFLAGS:=$(COMMON_FLAGS) -std=gnu99 
# LD flags
# remove unreferenced procedures and variables, but __isr_vector
GC:=-Wl,--gc-sections  -Wl,--undefined=__isr_vector
MAP:=-Wl,-Map=$(MAPNAME)
LFLAGS:=$(COMMON_FLAGS) $(GC) $(MAP)
#LDLIBS:=--specs=nosys.specs -lc -lc -lnosys -L$(LDSCRIPTDIR) -T $(LDSCRIPT)
LDLIBS:=--specs=nosys.specs -L$(LDSCRIPTDIR) -T $(LDSCRIPT)

# Additional Suffixes
.SUFFIXES: .elf .hex .bin .dis

# Targets
.PHONY: all
all: $(DISNAME) $(HEXNAME)
	$(SIZE) $(ELFNAME)

.PHONY: upload
# stm32flash v0.5 does not support STM32L011, so use the latest snapshot 
upload: $(DISNAME) $(HEXNAME) $(ELFNAME)
	/data/kraus/git/stm32flash-code/stm32flash -e 127 -g 0 -w $(HEXNAME) -v /dev/ttyUSB0
	$(SIZE) $(ELFNAME)

.PHONY: flash
flash: $(DISNAME) $(HEXNAME) $(ELFNAME)
	../../stlink/build/Release/bin/st-flash --format ihex write $(HEXNAME)

.PHONY: clean
clean:
	$(RM) $(OBJ) $(HEXNAME) $(ELFNAME) $(LIBNAME) $(DISNAME) $(MAPNAME) libssp.a libssp_nonshared.a

.PHONY: show_option
show_option: 
	../../stlink/build/Release/bin/st-flash  --area=option read
# show option reports fffffeaa for a new device

# implicit rules
.elf.hex:
	$(OBJCOPY) -O ihex $< $@



# explicit rules

$(ELFNAME): $(LIBNAME)($(OBJ)) libssp.a libssp_nonshared.a
	$(LINK.o) $(LFLAGS) $(LIBNAME) $(LDLIBS) -o $@

$(DISNAME): $(ELFNAME)
	$(OBJDUMP) -D -S $< > $@

# create empty ssp libs for -fstack-protector-all -fstack-protector
libssp.a:
	$(AR) rcs $@
	
libssp_nonshared.a:
	$(AR) rcs $@

	
