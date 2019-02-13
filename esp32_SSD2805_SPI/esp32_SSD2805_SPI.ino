/**
 * @brief This is a primitive demo to drive a mipi display with ESP32. For further details please read README.md in the same root as this sketch.
 *        Image "rain.jpg" obtained from pixabay : https://pixabay.com/en/rain-drops-rainy-wet-droplets-455124/, trimmed to 240*240 to fit LCD.
 * @Programmer  John Leung @ TechToys Hong Kong
 * @Date        2019-02-13
 */

#include "SSD2805_VSPI_drv.h"
#include "rain.h"

void setup() {
  // put your setup code here, to run once:
  SSD2805_begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  SSD2805_dispFlush(0,0,rain.width-1, rain.height-1,(const uint16_t *)rain.data); 
  vTaskDelay(1000/portTICK_PERIOD_MS); 
  SSD2805_clearLCD(0xf800);
  vTaskDelay(1000/portTICK_PERIOD_MS);
  SSD2805_clearLCD(0x07e0);
  vTaskDelay(1000/portTICK_PERIOD_MS);
  SSD2805_clearLCD(0x001f);
  vTaskDelay(1000/portTICK_PERIOD_MS);
}
