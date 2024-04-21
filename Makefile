##############################################################################
BUILD = build
BIN = emon32
##############################################################################
.PHONY: all directory clean size

# Path to toolchain, e.g. /path/to/bin/ Leave empty if already on path.
TC_PATH =
CC = $(TC_PATH)arm-none-eabi-gcc
OBJCOPY = $(TC_PATH)arm-none-eabi-objcopy
SIZE = $(TC_PATH)arm-none-eabi-size

ifeq ($(OS), Windows_NT)
  MKDIR = gmkdir
else
  MKDIR = mkdir
endif

CFLAGS += -W -Wall -Wpedantic --std=gnu99 -Os
CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -Wuninitialized
CFLAGS += -mcpu=cortex-m0plus -mthumb
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m0plus -mthumb
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--print-memory-usage
LDFLAGS += -Wl,--script=./linker/samd21j17.ld

INCLUDES += \
  -I./include/samd21 \
  -I./third_party/printf \
  -I./third_party/qfplib \
  -I./third_party/RFM69 \
  -I./src/

SRCS += $(wildcard ./src/*.c) \
  $(wildcard ./third_party/printf/*.c)

DEFINES += \
  -D__SAMD21J17A__ \
  -DDONT_USE_CMSIS_INIT

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))
OBJS += $(BUILD)/qfplib-m0-full.o $(BUILD)/qfpio.o

# Always update the build information. This forces this to run every time
BUILD_INFO := $(shell python3 ./scripts/build_info.py ./src/emon32_build_info.c)

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin $(BUILD)/$(BIN).uf2 size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

$(BUILD)/$(BIN).uf2: $(BUILD)/$(BIN).bin
	@echo BIN_TO_UF2 $@
	@python3 ./scripts/bin_to_uf2.py $(BUILD)/$(BIN).bin $(BUILD)/$(BIN).uf2

$(BUILD)/qfplib-m0-full.o:
	@echo AS $@
	@$(CC) $(CFLAGS) third_party/qfplib/qfplib-m0-full.s -c -o $@

$(BUILD)/qfpio.o:
	@echo AS $@
	@$(CC) $(CFLAGS) third_party/qfplib/qfpio.s -c -o $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRCS)) -c -o $@

directory:
	@$(MKDIR) -p $(BUILD)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)
