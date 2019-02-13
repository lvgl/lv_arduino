/**
 * @brief     SPI driver for a mipi bridge chip SSD2805
 * @processor esp32 + SSD2805 + 1.54" 240*240 mipi LCD (LG LH154Q01)
 * 
 * @note  Reference on isr: 
 * https://github.com/espressif/esp-idf/blob/97eecfa1b2b95a6f77e8e9ffcb676ded687ab68f/examples/peripherals/gpio/main/gpio_example_main.c
 * https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/gpio.html?highlight=gpio_isr_handler_add
 * Reference on SPI by DMA: 
 * https://github.com/espressif/esp-idf/blob/master/examples/peripherals/spi_master/main/spi_master_example_main.c
 * https://github.com/littlevgl/lv_projects/tree/master/esp32_ili9341
 * 
 * Only 16 bit per pixel is supported in 565 color format R4R3R2R1R0G5G4G3 G2G1G0B4B3B2B1B0
 * Programmer : John Leung @ TechToys Co. Hong Kong (www.TechToys.com.hk)
 * Date: 2018-02-11
 * 
 * Important: Driver works ONLY for Arduino ESP32 core v1.0.0. Tested with core version v1.0.1-rcx but all of them didn't compile
 */

#ifndef SSD2805_SPI_H
#define SSD2805_SPI_H

#include <Arduino.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

//Attaching VSPI/HSPI module to SSD2805
#define SPI_NUM  VSPI

#if (SPI_NUM==VSPI)
  #define PIN_NUM_MOSI 23
  #define PIN_NUM_MISO 19
  #define PIN_NUM_SCK  18
  #define PIN_NUM_CS   5
#else //if not VSPI, it should be a HSPI
  #define PIN_NUM_MOSI 13
  #define PIN_NUM_MISO 12 //should be 12?
  #define PIN_NUM_SCK  14
  #define PIN_NUM_CS   15 //cannot use GPIO15 if dev_config.spics_io_num = -1; i.e. when it is not using automatic CS strobe
#endif

#define PIN_NUM_SDC  25 //data(1)/command(0) selection pin (for 8 Bit 4 Wire only)
#define PIN_NUM_RST  10  //manual reset pin for SSD2805
#define PIN_NUM_TE   37 //input pin for TE signal

//DSC commands
#define DCS_NOP                 0x00
#define DCS_SWRESET             0x01    //soft reset
#define DCS_RDDPM               0x0A    //Read display power mode
#define DCS_RDDMADCTL           0x0B    //Read display MADCTL
#define DCS_RDDCOLMOD           0x0C    //Read Display COLMOD
#define DCS_RDDIM               0x0D    //Read display image mode
#define DCS_RDDSM               0x0E    //Read display signal mode
#define DCS_RDDSDR              0x0F    //Read display self-diagnostic result
#define DCS_SLPIN               0x10
#define DCS_SLPOUT              0x11
#define DCS_NORON               0x13    //Normal display mode on
#define DCS_INVOFF              0x20
#define DCS_INVON               0x21
#define DCS_DISPOFF             0x28
#define DCS_DISPON              0x29

#define DCS_COL_SET             0x2A
#define DCS_PAGE_SET            0x2B
#define DCS_RAMWR               0x2C    //command to transfer data from MPU to frame memory
#define DCS_RAMRD               0x2E
#define DCS_TE_OFF              0x34
#define DCS_TE_ON               0x35
#define DCS_MEM_ACC_CTL         0x36    //memory access control for frame buffer scanning direction etc
#define DCS_IDLE_MODE_OFF       0x38
#define DCS_IDLE_MODE_ON        0x39
#define DCS_COLMOD              0x3A    //interface pixel format set
#define DCS_RAMWRC              0x3C    //continue memory write after DCS_RAMWR
#define DCS_READ_MEMORY_CNTU    0x3E

//Display size (in pixels)
#define DISP_HOR_SIZE 240
#define DISP_VER_SIZE 240

#define MAX_DMA_LEN   4092

#define MAX_TRANS_NUM 1            


#ifdef __cplusplus
extern "C" {
#endif

void SSD2805_begin(void);
void SSD2805_hwReset(void);
void SSD2805_clearLCD(uint16_t color);
void SSD2805_dispFill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color);
void SSD2805_dispFlush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const uint16_t* color_p);

#ifdef	__cplusplus
}
#endif

#endif
