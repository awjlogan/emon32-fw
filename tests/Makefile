CC = cc
CFLAGS = -Wall -Wpedantic
INCLUDES = ../src/ -I./
DEFINES = HOSTED

util: OBJS = test_util.c ../src/util.c
cm: OBJS = test_cm.c ../src/emon_CM.c
data: OBJS = test_data.c ../src/data_pack.c ../src/util.c
configuration: OBJS = test_configuration.c ../src/configuration.c ../src/util.c
eeprom: OBJS = test_eeprom.c ../src/eeprom.c
adc: OBJS = test_adc.c

.PHONY: clean

all: util cm data configuration eeprom adc

adc:
	${CC} ${CFLAGS} -D$(DEFINES) -I$(INCLUDES) -o $@.test $(OBJS) ${LIBS}
eeprom:
	${CC} ${CFLAGS} -D$(DEFINES) -I$(INCLUDES) -o $@.test $(OBJS) ${LIBS}

configuration:
	${CC} ${CFLAGS} -D$(DEFINES) -I$(INCLUDES) -o $@.test $(OBJS) ${LIBS}

util:
	${CC} ${CFLAGS} -D$(DEFINES) -I$(INCLUDES) -o $@.test $(OBJS) ${LIBS}
	./util.test
cm:
	${CC} ${CFLAGS} -D$(DEFINES) -I$(INCLUDES) -o $@.test $(OBJS) ${LIBS}
	./cm.test
data:
	${CC} ${CFLAGS} -D$(DEFINES) -I$(INCLUDES) -o $@.test $(OBJS) ${LIBS}
	./data.test
clean:
	rm *.test
