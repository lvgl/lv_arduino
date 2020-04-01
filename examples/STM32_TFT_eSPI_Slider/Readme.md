Littlevgl on STM32F103C BluePill and ILI9341 in 8-Bit Parallel with XPT2046 Touch-pad on SPI2 port.
Library dependencies:
1.  https://github.com/Bodmer/TFT_eSPI
  and choosing the right setup inside User_Setup_Select.h and also set right pins on selected user file.
2.  TFT_eTouch.h    https://github.com/achillhasler/TFT_eTouch
  and set the right pins inside TFT_eTouchUser.h and also run calibrate.ino to get the calibration value and store it inside
  TFT_eTouchUser.h like #define TOUCH_DEFAULT_CALIBRATION { 294, 3913, 339, 3869, 2 }
  Note that TFT_eSPI touch include extension does nether support second SPI port nor touch in parallel mode. 
3. Installing the last master Arduino_Core_STM32 on https://github.com/stm32duino/Arduino_Core_STM32
  because the Hardwaretimer definitions changed a bit in 1.9.0 version which about to release.

  Created by Hamid Saffari @ Apr 2020. https://github.com/HamidSaffari/
  Released into the public domain.
