
# Put your stlink folder here so make burn will work.
#STLINK=~/stlink.git

# Put your source files here (or *.c, etc)
SRCS =  $(wildcard ./Src/*.c)

libarmmath = ./Lib/Drivers/CMSIS/Lib/GCC/libarm_cortexM4lf_math.a


# Binaries will be generated with this name (.elf, .bin, .hex, etc)
PROJ_NAME=ledstrips_nucleo

OTHER_INCLUDES = -I./Inc
OTHER_INCLUDES += -L.
OTHER_INCLUDES += -L./Lib/Drivers/CMSIS/Lib/GCC
OTHER_INCLUDES += -larm_cortexM4lf_math

# Put your STM32F4 library code directory here
STM_COMMON=./Lib

# STM32F4xx model
MCU_MODEL = STM32F401xE

# Normally you shouldn't need to change anything below this line!
#######################################################################################

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy
AS=arm-none-eabi-as

CORTEX_M4_HWFP_CC_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16
CORTEX_M4_HWFP_LIB_PATH = /usr/lib/gcc/arm-none-eabi/6.1.1/armv7e-m/fpu

LINKER=STM32F401RETx_FLASH.ld

CFLAGS  = -g -O3 -Wall -T$(LINKER) --specs=nosys.specs
CFLAGS += -D$(MCU_MODEL)
CFLAGS += -DARM_MATH_CM4
#CFLAGS += -D__FPU_PRESENT=1
CFLAGS += -lm -lc -lgcc
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 #-mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -march=armv7e-m
CFLAGS += -I.
CFLAGS += -L$(CORTEX_M4_HWFP_LIB_PATH)

LDFLAGS = -lm #-lgloss
LDFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 #-mthumb-interwork
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
LDFLAGS += -march=armv7e-m

# Include files from STM libraries
# CFLAGS += -I$(STM_COMMON)/Utilities/STM32F4-Discovery
CFLAGS += -I$(STM_COMMON)/Drivers/CMSIS/Include
CFLAGS += -I$(STM_COMMON)/Drivers/CMSIS/Device/ST/STM32F4xx/Include
CFLAGS += -I$(STM_COMMON)/Drivers/STM32F4xx_HAL_Driver/Inc
CFLAGS += $(OTHER_INCLUDES)

# add startup files to build
SRCS += ./Lib/Drivers/CMSIS/Device/ST/STM32F4xx/Source/Templates/system_stm32f4xx.c
MCU_MODEL_LOWER := $(shell echo $(MCU_MODEL) | tr A-Z a-z)
SRCS += Src/startup_$(MCU_MODEL_LOWER).s

# Add Hal lib src files
SRC_HAL := $(wildcard ./Lib/Drivers/STM32F4xx_HAL_Driver/Src/*.c)
SRC_HAL := $(filter-out $(wildcard ./Lib/Drivers/STM32F4xx_HAL_Driver/Src/*_template.c), $(SRC_HAL))
SRCS += $(SRC_HAL)

OBJS := ${SRCS:.c=.o}

#MATHSRCS = $(wildcard ./Lib/Drivers/CMSIS/DSP_Lib/Source/*/*.[sSc])
armmath = ./Lib/Drivers/CMSIS/DSP_Lib/Source
#MATHSRCS := $(wildcard $(armmath)/TransformFunctions/*) \
$(wildcard $(armmath)/CommonTables/*) \
$(wildcard $(armmath)/BasicMathFunctions/*)\
$(wildcard $(armmath)/ComplexMathFunctions/*) \
$(wildcard $(armmath)/FastMathFunctions/*)

#SRCS += $(MATHSRCS)

#MATHOBJS := ${MATHSRCS:.c=.o}



#$(startup):
#	$(AS) $(CORTEX_M4_HWFP_CC_FLAGS) -o $@ $<

.PHONY: proj

all: proj

proj: $(PROJ_NAME).elf

$(PROJ_NAME).elf: $(OBJS)
	$(CC) $(CFLAGS) $^ $(libarmmath) -o $@
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	$(RM) $(OBJS)
	rm -f *.o $(PROJ_NAME).elf $(PROJ_NAME).hex $(PROJ_NAME).bin

math: $(MATHOBJS)
	ar rcs libarm_cortexM4lf_math.a $^

cleanmath:
	$(RM) $(MATHOBJS)

# Flash the STM32F4
burn: proj
#	$(STLINK)/
	st-flash write $(PROJ_NAME).bin 0x8000000


colors:
	clang -lm colors.c -o colors.o && ./colors.o > colors.out
