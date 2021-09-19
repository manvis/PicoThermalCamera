# Raspberry Pi Pico Thermal Camera

A bit slow, but usable enough for what I want.

Based on a combination of these components:

* [MLX90640 Thermal Camera Breakout from Pimoroni](https://shop.pimoroni.com/products/mlx90640-thermal-camera-breakout?variant=12536948654163)
* [Pico Display Pack from Pimoroni](https://shop.pimoroni.com/products/pico-display-pack)
* [LiPo SHIM for Pico from Pimoroni](https://shop.pimoroni.com/products/pico-lipo-shim)
* A generic 3.7V LiPo battery for power

I ended up using the original [Melexis code](https://github.com/melexis/mlx90640-library) and
not the Pimoroni fork, since it was more up to date and I didn't care about the Python version.
If you do, porting the *MLX90640_I2C_PicoDriver.cpp* shouldn't be too difficult.

## TODO
* Adjustable emissivity for higher precision (question - how to measure it?)
* Move the camera data reading to the second core for (probably) slightly better performance
