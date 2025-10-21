# Other sensors

## Temperature sensing

The emonPi3 supports temperature sensing with [DS18B20 temperature sensors](../electricity-monitoring/temperature/DS18B20-temperature-sensing.md). These are small temperature sensors with a 12-bit ADC and a digital output all in the sensor itself. Communication is over a OneWire bus and requires no additional components. The sensors have a quoted accuracy of ±0.5°C in the range -10°C to +85°C.

```{tip}
We have tested up to 6 temperature sensors connected at once, the OneWire bus should however support more than this.
```

DS18B20 temperature sensors are connected via the labelled pluggable terminal blocks (multiple sensors can be connected to each temperature input, e.g using a 6x sensor breakout board). *Please note that temperature sensing is not broken out on the RJ45 connector which is for voltage sensing and power only.*

**Pluggable terminal block connections are:**<br>
GND (black), DATA (yellow), 3.3V (red), left to right, repeated for each of the three blocks:

![emonpi2_temperature_sensing.JPG](img/emonpi2_temperature_sensing.JPG)

The function of the emonPi3 terminal blocks is configured in software (see the [configuration information](./configuration.md)). The default configuration is for both inputs to be temperature sensor imputs.

![emonPi2_temperature_inputs.png](img/emonPi2_temperature_inputs.png)

The DS18B20 input is connected to GPIO17 on the RaspberryPi via the GPIO connection header. It is also connected to the microcontroller for use in transmitter mode as part of an emonTx6.

## Pulse counting

The OpenEnergyMonitor pulse counter can connected like this:

![emonPi2_pulse_input_analog.jpg](img/emonPi2_pulse_input_analog.jpg)

This pulse input will appear alongside the energy monitoring data from the emonPi3 on the emoncms inputs page. Note that the voltage sensor is required for this firmware to work.

---

**Pulse sensing using the terminals labelled 'Temperature'**:

While the left hand side terminal block inputs are configured for temperature sensing as standard, it's possible to change the function of the 'Data' pin on each of the terminal block inputs by software.

```{note}
<b>T1</b> can be either a pulse or OneWire input (<b>Pi GPIO27 PIN13</b>).

<b>T2</b> can be either a pulse or OneWire input (<b>Pi GPIO22 PIN15</b>).
```

Add the following emonhub.conf interfacer configuration. This will enable direct readings from the applicable pulse inputs on the Pi:

```
[[pulse1]]
    Type = EmonHubPulseCounterInterfacer
    [[[init_settings]]]
        pulse_pin = 13
        # bouncetime = 2
        # rate_limit = 2
    [[[runtimesettings]]]
        pubchannels = ToEmonCMS,
        nodeoffset = 1

[[pulse2]]
    Type = EmonHubPulseCounterInterfacer
    [[[init_settings]]]
        pulse_pin = 15
        # bouncetime = 2
        # rate_limit = 2
    [[[runtimesettings]]]
        pubchannels = ToEmonCMS,
        nodeoffset = 2
```

On pulse detection, the pulse inputs will appear on the Emoncms inputs page:

![emoncms_emonhub_pulseinputs.png](img/emoncms_emonhub_pulseinputs.png)

---

## Analog input

It's possible to use the right-most terminal block as an analog input as shown here. An example application is measuring flow rate using a Sika VFS which has an analog voltage output.

![emonPi3_analog_input.png](img/emonPi3_analog_input.png)

```{warning}
The analog input voltage must be in the range **0 - 1.024 V**; anything above 1.024 V will saturate to the maximum value. The maximum voltage must not exceed 3.3 V.
```

The following set of screenshots gives an example of configuring this analog input for use in reading the flow rate from a Sika VFS flow sensor. The flow rate is then used together with measurement of flow and return temperature to calculate heat.

![sika_inputs.png](img/sika_inputs.png)

![analog_input_processors.png](img/analog_input_processors.png)

![dt_heat_processors.png](img/dt_heat_processors.png)

![sika_feeds.png](img/sika_feeds.png)
