![Header image](images/header.jpg)

# esphome-p1mini
Based on esphome-p1reader, which is an ESPHome custom component for reading P1 data from electricity meters. Designed for Swedish meters that implements the specification defined in the [Swedish Energy Industry Recommendation For Customer Interfaces](https://www.energiforetagen.se/forlag/elnat/branschrekommendation-for-lokalt-kundgranssnitt-for-elmatare/) version 1.3 and above.

The component can be used [by itself from any config file](docs/component_only.md) or with one of the config files included in the project, which matches the suggested hardware configurations and is kept up to date with any updates to the component.

Notable differences from esphome-p1reader are:
* More frequent update of sensors with configurable update period.
* No additional components needed. RJ12 cable connects directly to ESP module. (A resistor may be needed in some cases)
* Code rewritten to not spend excessive amounts of time in calls to the `loop` function. This should ensure stable operation of ESPHome and might help prevent some serial communication issues.

## ESPHome version
The current version is tested with ESPHome version `2025.5.0` and the yaml *will not work with versions earlier than `2024.6.0`*.

## Verified meter hardware / supplier
* [Sagemcom T211](https://www.ellevio.se/globalassets/content/el/elmatare-produktblad-b2c/ellevio_produktblad_fas3_t211_web2.pdf) / Ellevio, Skånska Energi
* [Aidon 6534](https://jonkopingenergi.se/storage/B9A468B538E9CF48DF5E276BDA7D2D12727D152110286963E9D603D67B849242/5009da534dbc44b6a34cb0bed31cfd5c/pdf/media/b53a4057862646cbb22702a847a291a2/Aidon%206534%20bruksansvisning.pdf) with RJ12/P1-port module (*not* RJ45/NVE module) / SEVAB
* [Landis+Gyr E360](https://eu.landisgyr.com/blog-se/e360-en-smart-matare-som-optimerarden-totala-agandekostnaden) / E.ON - [But read this](docs/NO-RTS.md#landisgyr-e360)
* [S34U18 (Sanxing SX631)](https://www.vattenfalleldistribution.se/matarbyte/nya-elmataren/) / Vattenfall - [But read this](docs/NO-RTS.md#s34u18-sanxing-sx631)
* Kamstrup OMNIPOWER
* [KAIFA MA304H4E](https://reko.nackaenergi.se/elmatarbyte/) (and MA304T4E) / Nacka Energi - [But read this](docs/NO-RTS.md#kaifa-ma304t4e--ma304h4e)
* [SWEMET / Shenzhen Star - STZ351](https://www.veab.se/globalassets/dokumentarkiv/manualer-och-skotselrad/anvandarmanual-elmatare-3-fas.pdf) - Some meters are working fine while other seems to have an incorrectly formatted message and incorrectly calculated checksum. *If* you are having problems, look at [this discussion](https://github.com/Beaky2000/esphome-p1mini/issues/26) for a possible workaround.

## Meters verified with esphome-p1reader, which should work too...
* [Itron A300](https://boraselnat.se/elnat/elmatarbyte-2020-2021/sa-har-fungerar-din-nya-elmatare/) / Borås Elnät
* [KAIFA CL109](https://www.oresundskraft.se/dags-for-matarbyte/) / Öresundskraft

## Hardware
### Wemos D1 Mini
This project is named after the Wemos D1 mini board, which is based on the ESP8266 processor. D1 mini boards (or clones) are very cheap and still work well.

[The build instructions for the D1 mini](docs/build_d1_mini.md) match the `p1mini.yaml` configuration.

### Waveshare ESP32-C3-Zero
However, the ESP8266 is now over 10 years old and [no longer recommended](https://esphome.io/guides/faq.html) for ESPHome projects. As a result I have moved to using a Waveshare ESP32-C3-Zero board, with a more powerfull processor that does not require more power than the ESP8266.

[The build instructions for the C3-Zero](docs/build_c3_zero.md) match the `p1mini32.yaml` configuration.

### ... or anything else
It is also fairly easy to take any board that ESPHome supports and modifying one of the configurations to work with that. It is mostly a question of figuring out what pins to use for what. If you have pre built hardware which does not connect the RTS signal to a GPIO, [read this](docs/NO-RTS.md#rts-not-attached-to-a-gpio). Also, if your pre built hardware inverts the signal in hardware, make sure to remove the inversion in the configration!

Note that ESP32 based boards other than the ESP32-C3 draw more power, which may cause a problem with the supply from the meter and generally offer no advantage. The P1 port on the meter provides 5V up to 250mA.

## P1 Passthrough
[It is possible to attach another P1 reading device in case you need to connect a car charger (or a second p1-mini...) etc.](docs/passthrough.md).

## Installation
The component can be used by itself from any config file, or with one of the included config files, which are kept up to date with any updates and matches one of the hardware configurations.

### Standalone
If you are making substantial changes to the config it may make more sense to [use the component only](docs/component_only.md) in your config file. 

### With one of the included yaml files
Clone the repository and create a companion `secrets.yaml` file with the following fields:
```
wifi_ssid: <your wifi SSID>
wifi_password: <your wifi password>
p1mini_password: <Your p1mini password (for OTA, etc)>
p1mini_api_key: <Home Assistant API key>
```
The `p1mini_password` field can be set to any password before doing the initial upload of the firmware. A new API key can be generated on [this page](https://esphome.io/components/api.html).

The file structure should include these files:

```
|- p1mini.yaml          (or p1mini32.yaml)
|- secrets.yaml
|- components
   |- p1_mini
      |- __init__.py
      |- p1_mini.cpp
      |- p1_mini.h
      |- sensor
         |- __init__.py
         |- p1_mini_sensor.cpp
         |- p1_mini_sensor.h
      |- text_sensor
         |- __init__.py
         |- p1_mini_text_sensor.cpp
         |- p1_mini_text_sensor.h
```

Flash ESPHome as usual, with the relevant files in place. *Don't* connect USB and the P1 port at the same time!

If everything works, Home Assistant will autodetect the new integration after you plug it into the P1 port:

![In Home Assistant](images/inHA.png)

## Troubleshooting
[Things to try if you are having problems](docs/troubleshooting.md). (Ideally before opening a GitHub Issue)
