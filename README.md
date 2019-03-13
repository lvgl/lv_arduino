# LittlevGL Arduino library

This library allows to use LittlevGL as an Arduino library. Library can be installed via Arduino IDE Library Manager or as an .ZIP library.

## Example

There is simple example which uses https://github.com/Bodmer/TFT_eSPI library as an TFT driver to simplify testing. To get all this to work you have to setup TFT_eSPI to work with your TFT display type via editing the `User_Setup.h` file in TFT_eSPI library folder, or by selecting your own configurtion in the `User_Setup_Select.h` file in TFT_eSPI library folder.

LittlevGL library has its own configuration file in `lv_conf.h` file, which is locatd in LittlevGL library folder. Please get in mind to check that corresponding resolutions in LVGL configuration match the ones in TFT_eSPI and with physical resolution of your display.
