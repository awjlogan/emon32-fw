CC = cc
CFLAGS = -Wall -Wpedantic -g
INCLUDES = ../src/ -I./
DEFINES = -DHOSTED

cm: OBJS = test_cm.c ../src/emon_CM.c ../src/board_def.c
eeprom: OBJS = test_eeprom.c ../src/eeprom.c

.PHONY: clean all

all: cm eeprom

eeprom:
	${CC} ${CFLAGS} $(DEFINES) -I$(INCLUDES) -o $@.test $(OBJS) ${LIBS}
cm:
	${CC} ${CFLAGS} $(DEFINES) -I$(INCLUDES) -o $@.test $(OBJS) ${LIBS}
clean:
	rm *.test
	rm -rf *dSYM/

