# The RTS signal
p1mini uses the RTS signal to request updates from the meter at the desired intervals. Once an update has been received, RTS is set low while the data is processed, which insures that the meter will not send new updates before the p1mini is ready to receive.

Another advantage is that some meters can send updates more frequently when prompted by the RTS signal than they do when RTS is left always high.

## RTS always high
If the RTS signal is always high, the meter will send updates at some interval which is specific to that meter (1 second, 5 seconds and 10 seconds have been observerd). The meter will send the update regardless if the p1mini is ready to receive or not (but a typical p1mini can receive and process more than two updates per second, so usually not a problem)

Generally, the RTS signal is a good thing, but there are some cases when it is better to set it high all the time: If the meter works unreliably with the RTS signal or if your p1mini was build without attaching the RTS signal to a GPIO pin.

### Meters that do not work well with RTS
The following meters have been known to have issues with RTS.

#### S34U18 (Sanxing SX631)
The S34U18 seems buggy and may stop working intermittently when using the RTS signal. If you are having issues, try keeping RTS high all the time in the config.

#### KAIFA MA304T4E / MA304H4E
May not work at all wihtout setting RTS constantly high.

#### Landis+Gyr E360
Most meters accepth 3.3 V on the RTS signal, but the E360 may need the full 5 volts, which is according to specifications, so there is nothing wrong with the meter. This can be solved with level shifting circuitry or by simply attaching RTS directly to 5 V all the time. (And setting the minimum period to 0s)

### RTS not attached to a GPIO
If you built your ESP based on the original esphome-p1reader or if you bought a Slimmeleser etc which does not connect the RTS to a GPIO, you need to set the minimum update period to zero.

## Disabling the RTS signal
By changing this, the RTS signal (if connected) will be set constantly high.

In the yaml file, never set p1_rts low. Simply find and remove this line (in two places):

```
- switch.turn_off: p1_rts
```

If RTS is not connected to a GPIO, p1_rts and all references to it can be removed from the yaml.

Also, the minimum update period needs to be set to zero:

```
minimum_period: 0s
```
