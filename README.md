# _emon32-Pi2_ Firmware

This describes the firmware provided for the _emon32_ energy monitoring platform. The software is intended to be modular, and easily portable to other microcontrollers and implementations.

## emonLibCM Comparison

The emonTx does not use a number of the values calculated by the full emonLibCM library. The following features are not currently supported:

  - Assume RMS voltage
  - RMS Current per CT
  - Power factor per CT
  - Apparent power per CT

## Compile Time Configuration Options 🧱

Most compile time options are contained in `firmware/src/emon32.h`. The following options are configurable:

  - **NUM_V**: The number of voltage channels. This can be less than or equal, but not more than, the number of physical channels on the board.
  - **NUM_CT**: The number of CT channels. This can be less than or equal, but not more than, the number of physical channels on the board.
  - **SAMPLE_RATE**: _Per channel_ sample rate. The ADC's overall sample rate is `(NUM_V + NUM_CT) * SAMPLE_RATE`.
  - **DOWNSAMPLE_DSP**: If this is defined, then the digital filter will be used for downsampling, rather than simply discarding samples.
  - **DOWNSAMPLE_TAPS**: The number of taps in the digital filter.
  - **SAMPLES_IN_SET**: Number of full sets (all V + CT channels) to acquire before raising interrupt.
  - **SAMPLE_BUF_DEPTH**: Buffer depth for digital filter front end.
  - **PROC_DEPTH**: Buffer depth for samples in power calculation.

## Compiling

Compiling the firmware requires the correct toolchain. The Makefile is for a Cortex-M0+ based microcontrollers, specifically the Atmel ATSAMD11D14 and ATSAMD21J. You will need the [Arm gcc toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) (may be available as a package in your distribution).

To build the firmware:

  `> make`

This will generate `firmware/build/emon32.elf` which can then be flashed to the microcontroller.

If you do not have the Arm gcc toolchain on $PATH, you can specify the path in `> Makefile`

### Bootloader

The emon32-Pi2 comes preloaded with a UF2 bootloader.

#### Preparing the firmware

The bootloader occupies the first 8KB of flash. The linker must be modified to account for the change of address. In `firmware/linker/samd11d14.ld`, in the `MEMORY` section, uncomment the line with `OFFSET = 0x00000400` and comment the line with `OFFSET = 0x00000000`. Recompile the firmware as normal. Note that the maximum size of the emon32 is now 1 KB less than without the bootloader.

The new `emon32.elf` file must then be converted to a DFU file for upload.

#### Uploading the firmware

  1. Power off the emon32-Pi2
  2. Connect one end of the USB-C cable
  3. While holding down the emon32's `RESET` button, connect the other end of the USB cable
  4. The emon32 will enter the bootloader, as indicated by the **PROG** LED lighting up.

#### Installing the bootloader

If your board does not come with the bootloader preflashed, after cloning the bootloader repository, copy `firmware/helpers/bootloader.patch` to the bootloader's directory. In that directory, run `git am bootloader.patch` to apply the changes. The bootloader


## Modifications 🔧

### Changing CT calibration

Fixed point (Q15) calibration values for a given CT phase shift can be generated using the **phasecal.py** script (*./helpers/phasecal.py*). The usage of this is: `phasecal.py <MAINS FREQENCY> <EFFECTIVE SAMPLE RATE> <PHI_0> .. <PHI_N>` where `PHI_N` is the phase shift of each CT, *N*. Note that *EFFECTIVE SAMPLE RATE* is the final *f* after downsampling.

### Designing a new board

The files `firmware/src/board_def.h` and `firmware/src/board_def.c` contain options for configuring the microcontroller for a given board. For example, different pin mappings may be required.

### Porting to different microcontroller

Within the top level loop, there are no direct calls to low level hardware. You must provide functions that handle the hardware specific to the microcontroller you are using.

All peripheral drivers are in header/source pairs named **driver_\<PERIPHERAL\>**. For example, the ADC driver is in **driver_ADC.\***. If you are porting to a new microcontroller, you will need to provide implementations of all the functions exposed in **driver_\<PERIPHERAL\>.h** and any internal functions within **driver_\<PERIPHERAL\>.c**. If your microcontroller does not support a particular function (for example, it doesn't have a DMA), then either no operation or an alternative must be provided.

You will also need to ensure that the vendor's headers are included and visible to the compiler.

### Digital filter

The base configuration has an oversampling factor of 2X to ease the anti-aliasing requirments. Samples are then low pass filtered and reduced to *f/2* with a half band filter (**ecmFilterSample()**). The half band filter is exposed for testing. Filter coefficients can be generated using the **filter.py** script (*./helpers/filter.py*). It is recommended to use an odd number of taps, as the filter can be made symmetric in this manner. You will need **scipy** and **matplotlib** to use the filter designer,

### Hosted testing

There are tests available to run on local system (tested on macOS and Linux), rather than on a physical device, for some functions. These are in *./tests*. In that folder, run `make all` to build the tests. These allow for development on a faster system with better debug options. The firmware is structured to remove, as far as possible, direct calls to hardware. Do note that some functions will not behave identically. For example, in the configuration menu terminal entry may be different to that through a UART.

## Acknowledgements

### Third party

  -

