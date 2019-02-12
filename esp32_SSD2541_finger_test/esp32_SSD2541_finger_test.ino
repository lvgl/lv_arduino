/**
 * @brief This example demonstrates `SSD2541_getPoint(args)` with coordinates of your finger detected with pressure.
 *        Plesae read README.md for further details.
 * @date  10th Feb 19
 */

#include "SSD2541.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  SSD2541_begin();

  uint16_t deviceId = SSD2541_readDeviceId();

  Serial.printf("Device ID is 0x%x\n", deviceId);
}

void loop() {
  // put your main code here, to run repeatedly:
  static int16_t finger_x=0, finger_y=0;
  static uint8_t pressure = 0;
  
  vTaskDelay(50/portTICK_PERIOD_MS);
  SSD2541_getPoint(&finger_x, &finger_y, &pressure);
  Serial.printf("Touch x,y = %d , %d , %d\n", finger_x, finger_y, pressure);
}
