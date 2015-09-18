PROJECT_NAME := rfsa

SDK_PATH=$(abspath ../nRF51_SDK_9.0.0)
MICROESB_PATH=../nrf51-micro-esb
LIBRARIES_PATH=$(abspath ../libraries)
export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = ${SDK_PATH}/components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy"
SIZE    		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size"

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
main.c\
$(LIBRARIES_PATH)/Adafruit_SSD1306/Adafruit_SSD1306.c\
$(abspath $(SDK_PATH)/components/toolchain/system_nrf51.c) \
$(abspath $(SDK_PATH)/components/libraries/util/app_error.c) \
$(abspath $(SDK_PATH)/components/drivers_nrf/uart/app_uart_fifo.c)\
$(abspath $(SDK_PATH)/components/libraries/fifo/app_fifo.c)\
$(abspath $(SDK_PATH)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c)\
$(abspath $(SDK_PATH)/components/drivers_nrf/common/nrf_drv_common.c)\
$(abspath $(SDK_PATH)/components/drivers_nrf/spi_master/spi_master.c)\
$(abspath $(SDK_PATH)/components/drivers_nrf/clock/nrf_drv_clock.c)\
$(abspath $(SDK_PATH)/components/libraries/uart/retarget.c) \
$(abspath $(SDK_PATH)/components/libraries/trace/app_trace.c) \
$(SDK_PATH)/components/libraries/timer/app_timer.c\
$(SDK_PATH)/components/libraries/timer/app_timer_appsh.c\
$(SDK_PATH)/components/libraries/scheduler/app_scheduler.c\
$(SDK_PATH)/components/libraries/util/app_util_platform.c\
$(SDK_PATH)/components/drivers_nrf/hal/nrf_delay.c\
$(SDK_PATH)/components/drivers_nrf/nrf_soc_nosd/nrf_soc.c\

## $(MICROESB_PATH)/common/micro_esb.c\

#CPP_SOURCE_FILES = \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath $(SDK_PATH)/components/toolchain/gcc/gcc_startup_nrf51.s)

#assembly files common to all targets
LIBS  = $(abspath $(SDK_PATH)/components/properitary_rf/esb/gcc/esb_gcc.a)

#includes common to all targets
INC_PATHS  = -I$(abspath $(SDK_PATH)/components/device)
INC_PATHS += -I$(abspath $(SDK_PATH)/components/libraries/util)
INC_PATHS += -I$(SDK_PATH)/components/libraries/timer
INC_PATHS += -I$(SDK_PATH)/components/libraries/scheduler
INC_PATHS += -I$(abspath $(SDK_PATH)/components/libraries/trace)
INC_PATHS += -I$(abspath $(SDK_PATH)/components/libraries/fifo)
INC_PATHS += -I$(abspath $(SDK_PATH)/components/drivers_nrf/hal)
INC_PATHS += -I$(abspath $(SDK_PATH)/components/drivers_nrf/uart)
INC_PATHS += -I$(abspath $(SDK_PATH)/components/properitary_rf/esb)
INC_PATHS += -I$(abspath $(SDK_PATH)/components/toolchain/gcc)
INC_PATHS += -I$(abspath ../../../../../../bsp)
INC_PATHS += -I$(abspath $(SDK_PATH)/components/toolchain)
INC_PATHS += -I$(abspath $(SDK_PATH)/components/drivers_nrf/nrf_soc_nosd)
INC_PATHS += -I$(abspath $(SDK_PATH)/components/drivers_nrf/gpiote)
INC_PATHS += -I$(abspath $(SDK_PATH)/components/drivers_nrf/common)
INC_PATHS += -I$(abspath $(SDK_PATH)/components/drivers_nrf/spi_master)
INC_PATHS += -I$(SDK_PATH)/components/drivers_nrf/clock
INC_PATHS += -I$(abspath $(MICROESB_PATH)/common)
INC_PATHS += -I$(LIBRARIES_PATH)/Adafruit_SSD1306
INC_PATHS += -I$(LIBRARIES_PATH)/Adafruit-GFX-Library
INC_PATHS += -I.

OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
COMMON_FLAGS  = -DESB_PRESENT
COMMON_FLAGS += -DNRF51
COMMON_FLAGS += -DDEBUG
COMMON_FLAGS += -DENABLE_DEBUG_LOG_SUPPORT
COMMON_FLAGS += -DBOARD_CUSTOM
COMMON_FLAGS += -DBSP_DEFINES_ONLY
COMMON_FLAGS += -DSPI_MASTER_0_ENABLE
COMMON_FLAGS += -mcpu=cortex-m0
COMMON_FLAGS += -mthumb -mabi=aapcs 
COMMON_FLAGS += -Wall -Werror
#COMMON_FLAGS += -O3 
COMMON_FLAGS +=  -g
COMMON_FLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
COMMON_FLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
COMMON_FLAGS += -fno-builtin --short-enums

CFLAGS = ${COMMON_FLAGS}
CFLAGS += --std=gnu99
CPPFLAGS = ${COMMON_FLAGS}


# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DESB_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DBSP_DEFINES_ONLY
#default target - first one defined
default: clean nrf51422_xxac

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_xxac

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51422_xxac


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

CPP_SOURCE_FILE_NAMES = $(notdir $(CPP_SOURCE_FILES))
CPP_PATHS = $(call remduplicates, $(dir $(CPP_SOURCE_FILES) ) )
CPP_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(CPP_SOURCE_FILE_NAMES:.cpp=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.cpp $(CPP_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS) $(CPP_OBJECTS)

nrf51422_xxac: OUTPUT_FILENAME := nrf51422_xxac
nrf51422_xxac: LINKER_SCRIPT=esb_gcc_nrf51.ld
nrf51422_xxac: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Create objects from C++ SRC files
$(OBJECT_DIRECTORY)/%.o: %.cpp
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CPPFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash: $(MAKECMDGOALS)
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	nrfjprog --reset --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex

## Flash softdevice

ctags:
	ctags --exclude=$(abspath $(SDK_PATH))/examples --recurse . $(abspath $(SDK_PATH)) $(LIBRARIES_PATH)
