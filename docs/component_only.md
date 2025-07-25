# Using the p1_mini component in your own config file
The `p1_mini` component and sensor can be used from any ESPHome config file by adding the Github repository as source for external components:

```
external_components:
  - source: github://Beaky2000/esphome-p1mini@main
```

One drawback of using this method, instead of using one of the config files included in this project, is that your config can stop working at any time because of updates to the component that you need to account for in your config. Therefore this makes most sense if you are making large modifications to the config, such as using some other hardware etc.

For an example of how to set up UARTs, the p1_mini component and sensors, look at the included config files. Some day I might document all parameters here, but that day is not yet here.
