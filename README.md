# _emon32_ Firmware

This describes the firmware provided for the [_emon32_ energy monitoring](https://github.com/awjlogan/emon32) system. The software is modular, and should be easily portable to other microcontrollers and implementations.

This firmware is intended to be used with the [OpenEnergyMonitor](https://openenergymonitor.org) platform. Hardware systems are available directly from them.

## Getting in contact

### Problems

Issues can be reported:

- As a GitHub issue
- On the OpenEnergyMonitor forums

Please include as much information as possible (run the `v` command on the serial link), including at least:

- The emon32 hardware that you using (board name and revision, and serial number)
- The firmware version
- All settings (run the `l` command on the serial link)
- A full description, including a reproduction, of the issue

### Contributing

Contributions are welcome! Small PRs can be accepted at any time. Please get in touch before making _large_ changes to see if it's going to fit before spending too much time on things.

> [!NOTE]
> Please bear in mind that this is an open source project and PRs and enhancements may not be addressed quickly, or at all. This is no comment on the quality of the contribution, and please feel free to fork as you like!

## Functional Description

### Hardware serial connection

A dedicated UART is used for debug, configuration, and data transmission. It has the following UART configuration:

- 38400 baud
- 8N1

It is available on:

- Raspberry Pi:
  - GPIO 14 (UART TX _from_ Raspberry Pi)
  - GPIO 15 (UART RX _to_ the Raspberry Pi)
- Debug pins:
  - 2 (UART TX _from_ Raspberry Pi)
  - 1 (UART RX _to_ the Raspberry Pi)
- Test harness pads (only when TEST_SENSE is LOW)

### Configuration

The _emon32_ firmware is compatible with the OpenEnergyMonitor [emonPi2 configuration](https://docs.openenergymonitor.org/emonpi2/configuration.html) options, which can be accessed through the debug serial link. In addition, the following options are added:

|Command    |Definition                                             |
|-----------|-------------------------------------------------------|
|o&lt;x&gt; |Auto calibrate CT lead for channel _x_                 |
|t          |Trigger a data set processing event                    |
|v          |Print firmware and board information                   |
|w&lt;n&gt; |Minimum energy difference, _n_ Wh, before saving       |

All options can be listed by entering `?`.

### Data acquisition

The ADC is triggered by a dedicated timer (`TIMER_ADC`) with no intervention from the processor. Data are accumulated by DMA into a ping-pong buffer - when one sample set is being processed, another is being captured in the background.

Raw data from the ADC are downsampled (if configured) and then injected into the energy and power calculation routines. As there is a single ADC, CT values are interpolated between the appropriate voltage samples.

### Data transmission

When a full report is ready, the following actions take place:

- 1 s _before_ the report is due, any temperature sensors present are triggered to record a value.
  - The DS18B20 temperature sensor takes 750 ms to take a measurement in the default 12bit mode.
  - A report can also be triggered with the command `t` on the serial link.
- At the report time, the following values are calculated:
  - Power for each CT.
  - Accumulated energy for each CT.
  - Mains frequency.
  - Power factor
- Data are packed into two formats:
  - Comma separated values for serial transmission.
  - Packed structure for transmission by the RFM module.
- Data are sent over the configured interface.
  - It is configurable whether data are always echoed on the debug console.

## Compiling and uploading

### Compiling

Compiling the firmware requires the correct toolchain. The Makefile is for a Cortex-M0+ based microcontrollers, specifically the [Atmel ATSAMD21J](https://www.microchip.com/en-us/product/ATSAMD21J17). You will need the [Arm gcc toolchain](https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain) (may be available as a package in your distribution).

To build the firmware:

  `> make -j`

In `build/`, the following binary files will be generated:

- `emon32.bin`
- `emon32.elf`
- `emon32.uf2`

### Uploading

The emon32-Pi3 comes preloaded with a [UF2 bootloader](https://microsoft.github.io/uf2/). This allows the firmware to be updated without any specialised hardware.

The bootloader binary is included in `bin/bootloader.elf`

  1. Connect to the host computer through the USB-C port.
  2. Double press the `RESET` button.
    - The **PROG** LED will pulse slowly to indicate it has entered bootloader mode,
    - The drive `EMONBOOT` will appear on the host.
  3. Copy `emon32.uf` to the `EMONBOOT` drive. The board will reset and enter the main program.

## Modifications

### Compile Time Configuration Options

Most compile time options are contained in `/src/emon32.h`. The following options are configurable:

- **NUM_V**: The number of voltage channels. This can be less than or equal, but not more than, the number of physical channels on the board.
- **NUM_CT**: The number of CT channels. This can be less than or equal, but not more than, the number of physical channels on the board.
- **SAMPLE_RATE**: _Per channel_ sample rate. The ADC's overall sample rate is `(NUM_V + NUM_CT) * SAMPLE_RATE`.
- **DOWNSAMPLE_DSP**: If this is defined, then the digital filter will be used for downsampling, rather than simply discarding samples.
- **DOWNSAMPLE_TAPS**: The number of taps in the digital filter.
- **SAMPLES_IN_SET**: Number of full sets (all V + CT channels and analog channels) to acquire before raising interrupt.
- **SAMPLE_BUF_DEPTH**: Buffer depth for digital filter front end.
- **PROC_DEPTH**: Buffer depth for samples in power calculation.

### Digital filter

The base configuration has an oversampling factor of 2X to ease the anti-aliasing requirments. Samples are then low pass filtered and reduced to _f/2_ with a half band filter (**ecmFilterSample()**). The half band filter is exposed for testing. Filter coefficients can be generated using the **filter.py** script (_./helpers/filter.py_). It is recommended to use an odd number of taps, as the filter can be made symmetric in this manner. You will need **scipy** and **matplotlib** to use the filter designer,

### Tests

A test program is available for the `emon_CM` module. This is the energy monitoring system and is completely abstracted from the underlying hardware.

There are tests available to run on local system (tested on macOS and Linux), rather than on a physical device, for some functions. These are in _./tests_. In that folder, run `make all` to build the tests. These allow for development on a faster system with better debug options. The firmware is structured to remove, as far as possible, direct calls to hardware. Do note that some functions will not behave identically. For example, in the configuration menu terminal entry may be different to that through a UART.

## Hardware Description

### Peripherals (SAMD21)

The following table lists the peripherals used in the SAMD21.

|Peripheral       | Alias           | Description                   | Usage                             |
|-----------------|-----------------|-------------------------------|-----------------------------------|
|ADC              |ADC              |Analog-to-digital converter    |Acquire analog signals             |
|DMAC             |                 |DMA Controller                 |ADC->buffer and UART TX            |
|EIC              |                 |External interrupt controller  |Input from zero-crossing detector  |
|EVSYS            |                 |Event System                   |Asynchronous event handling        |
|PORT             |                 |GPIO handling                  |                                   |
|SERCOM2          |SERCOM_UART_DBG  |UART (Debug)                   |Configuration and debug UART       |
|SERCOM3          |SERCOM_I2CM      |I2C (internal)                 |I2C for internal peripherals       |
|SERCOM4          |SERCOM_SPI_DATA  |SPI                            |Drive RFM module                   |
|SERCOM5          |SERCOM_I2M_EXT   |I2C (external)                 |Drive display module               |
|TC3              |TIMER_ADC        |Timer/Counter (16bit)          |ADC sample trigger                 |
|TC4+5            |TIMER_DELAY      |Timer/Counter (32bit)          |Delay timer                        |
|TC6+7            |TIMER_TICK       |Timer/Counter (32bit)          |Global time (micro/millisecond)    |

### Designing a new board

The files `/src/board_def.h` and `/src/board_def.c` contain options for configuring the microcontroller for a given board. For example, different pin mappings may be required.

### Porting to different microcontroller

Within the top level loop, there are no direct calls to low level hardware. You must provide functions that handle the hardware specific to the microcontroller you are using.

All peripheral drivers are in header/source pairs named **driver_\<PERIPHERAL\>**. For example, the ADC driver is in **driver_ADC.\***. If you are porting to a new microcontroller, you will need to provide implementations of all the functions exposed in **driver_\<PERIPHERAL\>.h** and any internal functions within **driver_\<PERIPHERAL\>.c**. If your microcontroller does not support a particular function (for example, it doesn't have a DMA), then either no operation or an alternative must be provided.

You will also need to ensure that the vendor's headers are included and visible to the compiler.

## Acknowledgements

### Third party libraries and tools

- [printf](https://github.com/eyalroz/printf) - embedded `printf` implementation.
- [Qfplib](https://www.quinapalus.com/qfplib.html) - soft floating point library for Arm Cortex-M.
- [SSD1306 library](https://github.com/Matiasus/SSD1306/tree/master) - used
- [Wintertools](https://github.com/https://github.com/wntrblm/wintertools) - various build scripts from Winterbloom.

### Others

- Rob Wall
