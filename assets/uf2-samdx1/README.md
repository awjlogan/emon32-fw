# UF2 Bootloader Configuration

This folder contains two files:

- `board.mk`
- `board_config.h`

that are used to build the [UF2 Bootloader](https://github.com/adafruit/uf2-samdx1). To build the bootloader:

1. In `<uf2-samdx1>/boards/` make a folder called `emonPi3`.
2. Copy `board.mk` and `board_config.h` into `<uf2-samdx1>/boards/emonPi3`.
3. In `<uf2-samdx1>/` run `make BOARD=emonPi3`.
4. In `<uf2-samdx1>/build/emonPi3` you will find the bootloader and bootloader updater with the file extensions `.bin` and `.elf`.

