##############################################################################
BUILD = build
BIN = emon32

# Qfplib-M0 floating point library
# https://www.quinapalus.com/qfplib-m0-tiny.html
QFPLIB_VER=20200617
QFPLIB_BUNDLE=qfplib-$(QFPLIB_VER)
QFPLIB_ARC=$(QFPLIB_BUNDLE).tar.gz
QFPLIB_URL=https://www.quinapalus.com/$(QFPLIB_ARC)

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
# LDFLAGS += -Wl,--script=./linker/samd11d14.ld
LDFLAGS += -Wl,--print-memory-usage
LDFLAGS += -Wl,--script=./linker/samd21j17.ld

INCLUDES += \
  -I./include/samd11 \
  -I./include/samd21 \
  -I./third_party/printf \
  -I./src/

SRCS += \
  ./src/startup_samd21.c \
  ./src/driver_ADC.c \
  ./src/driver_CLK.c \
  ./src/driver_DMAC.c \
  ./src/driver_EVSYS.c \
  ./src/driver_PORT.c \
  ./src/driver_SAMD.c \
  ./src/driver_SERCOM.c \
  ./src/driver_TIME.c \
  ./src/driver_WDT.c \
  ./src/board_def.c \
  ./src/configuration.c \
  ./src/data_pack.c \
  ./src/eeprom.c \
  ./src/emon_CM.c \
  ./src/emon32.c \
  ./src/periph_rfm69.c \
  ./src/periph_DS18B20.c \
  ./src/pulse.c \
  ./src/temperature.c \
  ./src/util.c \
  ./third_party/printf/printf.c


#-D__SAMD11D14AM__
DEFINES += \
  -D__SAMD21J17A__ \
  -DDONT_USE_CMSIS_INIT

CFLAGS += $(INCLUDES) $(DEFINES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRCS))))
OBJS += build/qfplib.o build/qfpio.o

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

build/qfplib.o: src/qfplib.s src/qfplib.h
	@echo AS $@
	@$(CC) $(CFLAGS) src/qfplib.s -c -o $@

build/qfpio.o: src/qfpio.s src/qfpio.h
	@echo AS $@
	@$(CC) $(CFLAGS) src/qfpio.s -c -o $@

%.o: src/qfpio.h src/qfplib.h
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

# Fetch Qfplib
$(QFPLIB_ARC):
	curl -O $(QFPLIB_URL)
$(QFPLIB_BUNDLE): $(QFPLIB_ARC)
	tar xvf $(QFPLIB_ARC)
$(QFPLIB_BUNDLE)/qfplib.s:	$(QFPLIB_BUNDLE)
$(QFPLIB_BUNDLE)/qfplib.h:	$(QFPLIB_BUNDLE)
$(QFPLIB_BUNDLE)/qfpio.s:	$(QFPLIB_BUNDLE)
$(QFPLIB_BUNDLE)/qfpio.h:	$(QFPLIB_BUNDLE)
src/qfplib.s:	$(QFPLIB_BUNDLE)/qfplib.s
	ln -sf ../$(QFPLIB_BUNDLE)/qfplib.s ./src/
src/qfplib.h:	$(QFPLIB_BUNDLE)/qfplib.h
	ln -sf ../$(QFPLIB_BUNDLE)/qfplib.h ./src/
src/qfpio.s:	$(QFPLIB_BUNDLE)/qfpio.s
	ln -sf ../$(QFPLIB_BUNDLE)/qfpio.s ./src/
src/qfpio.h:	$(QFPLIB_BUNDLE)/qfpio.h
	ln -sf ../$(QFPLIB_BUNDLE)/qfpio.h ./src/
