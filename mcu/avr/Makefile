#
# @file Makefile
# @brief 
# @author Liao MY
# @date 2014-07-21
#

# This Makefile is used to build specified source files and 
# source dircetions. Options:
#	TARGET_NAME	Target name.
#	C_SOURCES	All C source files.
#	MODULE_DIRS	Build all C files in this dir.
#	BUILD_DIR	Output dir.
#	HEADER_DIRS	The list of header files's dir.
#	DEFINES		Macro definition.
#	DEVICE		Chip type
#	FCPU		Clock speed
#
# END
HEADER_DIRS += $(MCULIB_AVR_PATH)/include

CC      = avr-gcc
AR      = avr-ar
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE    = avr-size
NM      = avr-nm

#####################################
# Check files or dirs if these exist.
#####################################
ERROR = $(foreach file,$(C_SOURCES),$(if $(wildcard $(file)),,$(file)))
ifneq ($(ERROR),)
  $(error $(ERROR) NOT EXIST)
endif

ifneq ($(MODULE_DIRS),)
  ERROR = $(foreach dir,$(MODULE_DIRS),$(if $(wildcard $(dir)/*),,$(dir)))
  ifneq ($(ERROR),)
    $(error $(ERROR) NOT EXIST OR EMPTY)
  endif
endif

ifeq  ($(TARGET_NAME),)
  $(error TARGET_NAME IS EMPTY )
endif
ifeq  ($(TARGET_NAME),)
  $(error TARGET_NAME IS EMPTY)
endif
ifeq  ($(C_SOURCES),)
  $(error C_SOURCES IS EMPTY)
endif
ifeq  ($(DEVICE),)
  $(error DEVICE IS EMPTY)
endif
ifeq  ($(FCPU),)
  $(error FCPU IS EMPTY)
endif
ifeq ($(BUILD_DIR),)
  BUILD_DIR = .
endif
ifeq ($(NO_ECHO),)
  NECHO	= @
else
  NECHO = 
endif

#################
# Set search path
#################
VPATH	+= :$(BUILD_DIR)
VPATH	+= $(addprefix :,$(MODULE_DIRS))
VPATH	+= $(addprefix :,$(sort $(dir $(C_SOURCES))))


###############
# All c files
##############
C_FILES := $(notdir $(C_SOURCES))
C_FILES += $(foreach dir,$(MODULE_DIRS),$(notdir $(wildcard $(dir)/*c)))
OBJ_FILES := $(C_FILES:.c=.o)


###############
# FLAG
##############
# Optimization level (0, 1, 2, 3, 4 or s)
OPTIMIZE = s
# Warning contorls
WARNINGS = all extra
CSTD = gnu89
# Warning contorls

CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -Wno-deprecated-declarations -D__PROG_TYPES_COMPAT__
CFLAGS += -Wl,--gc-sections
CFLAGS += -Wall
CFLAGS += -std=$(CSTD)
CFLAGS += -mmcu=$(DEVICE)
CFLAGS += -O$(OPTIMIZE) -mcall-prologues
CFLAGS += $(addprefix -W,$(WARNINGS))
CFLAGS += $(addprefix -I,$(HEADER_DIRS))
CFLAGS += $(addprefix -D,$(DEFINES))
#CFLAGS += -Wp,-M,-MP,-MT,$(BUILDDIR)/$(*F).o,-MF,$(BUILDDIR)/$(*F).d

LDFLAGS += -Wl,-lm -Wl,--gc-sections,-Map,$(TARGET_NAME).map

all: $(addprefix $(TARGET_NAME),.hex .bin .lst .sym) size 

# RULES
%.hex	: %.elf
	$(NECHO)echo "    --> " $@
	$(NECHO)$(OBJCOPY) -O ihex $< $@
%.bin	: %.elf
	$(NECHO)echo "    --> " $@
	$(NECHO)$(OBJCOPY) -O binary $< $@
%.lst	: %.elf
	$(NECHO)echo "    --> " $@
	$(NECHO)$(OBJDUMP) -h -S -C $< > $@
%.sym	: %.elf
	$(NECHO)echo "    --> " $@
	$(NECHO)$(NM) -n $< > $@
%.elf	:$(OBJ_FILES)
	$(NECHO)echo "    --> " $@
	$(NECHO)$(CC) $(LDFLAGS) $(CFLAGS) $^ -o $@

%.o	: %.c
	$(NECHO)echo "    --> " $@
	$(NECHO)$(CC)  $(CFLAGS) -c $< -o $@

%.d	: %.c 
	$(NECHO)echo "    --> " $@
	$(NECHO)$(CC) -MM $(CFLAGS) $< > $@.$$$$; \
		sed 's,\($*\)\.o[ :]*,\1.o $@ : ,g' < $@.$$$$ > $@; \
		rm -f $@.$$$$
sinclude $(C_FILES:.c=.d)

.PHONY	: size
size	: $(TARGET_NAME).elf
	$(NECHO)echo
	$(NECHO)$(SIZE) -C --mcu=$(DEVICE) $(TARGET_NAME).elf

.PHONY :clear
clean:
	$(NECHO)echo "Clear ..."
	rm -f $(BUILD_DIR)/*.a
	rm -f $(BUILD_DIR)/*.o
	rm -f $(BUILD_DIR)/*.hex
	rm -f $(BUILD_DIR)/*.sym
	rm -f $(BUILD_DIR)/*.elf
	rm -f $(BUILD_DIR)/*.lst
	rm -f $(BUILD_DIR)/*.d
	rm -f $(BUILD_DIR)/*.map
	rm -f $(BUILD_DIR)/*.bin

###################
# Download
###################
ifeq ($(PROGRAMMER),)
  PROGRAMMER = avrdude
endif

avrdude_DOWNLOAD = avrdude -c usbasp -p $(DEVICE) -U flash:w:$(TARGET_NAME).hex
avrdude_RESET = avrdude -c usbasp -p $(DEVICE) -E reset

.PHONY : download
download:
	$(NECHO)$($(PROGRAMMER)_DOWNLOAD)
reset:
	$(NECHO)$($(PROGRAMMER)_RESET)
	
