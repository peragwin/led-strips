
# Put your stlink folder here so make burn will work.
#STLINK=~/stlink.git

# Put your source files here (or *.c, etc)
SRCS =  $(wildcard ./Src/*.c)

# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=ledstrips_nucleo

OTHER_INCLUDES = -I./Inc
OTHER_INCLUDES += -L./Lib/Drivers/CMSIS/Lib/GCC -larm_cortexM4lf_math

# Put your STM32F4 library code directory here
STM_COMMON=./Lib

# STM32F4xx model
MCU_MODEL = STM32F401xE

# Normally you shouldn't need to change anything below this line!
#######################################################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
LINKER=STM32F401RETx_FLASH.ld

CFLAGS  = -g -O0 -Wall -T$(LINKER) --specs=nosys.specs
CFLAGS += -D$(MCU_MODEL)
CFLAGS += -lm #-lgloss
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -I.

# Include files from STM libraries
# CFLAGS += -I$(STM_COMMON)/Utilities/STM32F4-Discovery
CFLAGS += -I$(STM_COMMON)/Drivers/CMSIS/Include
CFLAGS += -I$(STM_COMMON)/Drivers/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS += -I$(STM_COMMON)/Drivers/STM32F4xx_HAL_Driver/Inc
CFLAGS += $(OTHER_INCLUDES)

# add startup files to build
SRCS += ./Lib/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
MCU_MODEL_LOWER := $(shell echo $(MCU_MODEL) | tr A-Z a-z)
SRCS += $(STM_COMMON)/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/gcc/startup_$(MCU_MODEL_LOWER).s

# Add Hal lib src files
SRC_HAL := $(wildcard ./Lib/Drivers/STM32F4xx_HAL_Driver/Src/*.c)
SRC_HAL := $(filter-out $(wildcard ./Lib/Drivers/STM32F4xx_HAL_Driver/Src/*_template.c), $(SRC_HAL))
SRCS += $(SRC_HAL)

OBJS = $(SRCS:.c=.o)


.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	rm -f *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin

# Flash the STM32F4
burn: proj
#	$(STLINK)/
	st-flash write $(PROJ_NAME).bin 0x8000000


colors:
	clang -lm colors.c -o colors.o && ./colors.o > colors.out
