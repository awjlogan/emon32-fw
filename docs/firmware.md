# Firmware

```{tip}
Most users do not need to compile their own firmware. Pre-built firmware can be downloaded from GitHub.
```

## Updating firmware

### emonTx6

To update the emonTx6's firmware, the following steps are taken:

- Connect a USB cable to the emonTx6's USB-C socket.
- Open a serial connection using (revisit: steps for screen or similar here, rather than Arduino)
- Enter 'e' and then press Enter.
- You will be prompted to reboot to enter the bootloader. Any unsaved configuration changes at this point will ne lost. Press 'y' to continue, or any other key to cancel.
- The emonTx6's LED will slowly pulse red (revisit: check colour!) and a drive called `EMONBOOT` will appear in the file manager.
- Drag and drop the firmware image ending `.uf2` to the `EMONBOOT` folder.
- The emonTx6 will reboot and enter the application.

### emonPi3

revisit: steps with serial bootloader

## How to compile firmware

There is a single unified firmware for the emonPi3 and emonTx6. The firmware is self contained, with no external libraries required and does not require any frameworks like Arduino or Platform.io.

Compiling the firmware requires the the [Arm gcc toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain). This also may be available as a package in your distribution. The Makefile is for Arm Cortex-M0+ based microcontrollers, specifically the Microchip ATSAMD21J17 ([datasheet](https://www.microchip.com/en-us/product/ATSAMD21J17), [errata](https://ww1.microchip.com/downloads/en/DeviceDoc/SAMD21-%20Family-Silicon-%20Errata-and-DataSheet-Clarification-DS80000760C.pdf)).

Ensure the toolchain is available on the path by running:

```{bash}
arm-none-eabi-gcc --version
```

Clone the `emon32-fw` repo and compile the firmware:

```{bash}
git clone https://github.com/awjlogan/emon32-fw
cd emon32-fw
make -j
```

Images in `.bin`, `.hex`, `.elf`, and `.uf2` formats will be in the `bin/`. The image names include the version and the git commit hash for traceability.
