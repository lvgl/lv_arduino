/**
 * @brief	Driver for SSD2541
 * 			  Processor : ESP32 Pico Kit
 * 			  Compiler : ESP32 IDF v3.0.0 or Arduino ESP32 core v1.0.0-rc4
 * @author	John Leung @ TechToys Co.
 * @web		www.TechToys.com.hk
 * @Rev		0.10
 * @date  2019-01-03
 */ 

#ifndef SSD2541_H
#define SSD2541_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "driver/i2c.h"

#ifndef ARDUINO
#include "sdkconfig.h"
#endif

#define ACK_CHECK_EN 	  0x1	/*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 	0x0	/*!< I2C master will not check ack from slave */
#define ACK_VAL 		0x0	/*!< I2C ack value */
#define NACK_VAL 		0x1	/*!< I2C nack value */

#define PIN_CTP_SDA     26 //GPIO26
#define PIN_CTP_SCL     27 //GPIO27
#define PIN_CTP_IRQ     38
#define PIN_CTP_RST     10  //sharing same reset pin of SSD2805

#define CTP_SIZE_X		240	//size of the CTP
#define CTP_SIZE_Y		240

/**
 * @note    I2C slave address of SSD2541 with STYPE pin short GND
 */
#define SSD2541_I2C_ADDR	0x48

/**
 * @note    Register table of SSD2541
 */
#define REG_NOP				0x00
#define REG_SW_RESET		0x01
#define REG_DEVICE_ID		0x02
#define REG_SLEEP_OUT		0x04
#define REG_SLEEP_IN		0x05
#define REG_DRIVE_SENSE_NO	0x06
#define REG_DRIVE_LINE0		0x07
#define REG_DRIVE_LINE1		0x08
#define REG_DRIVE_LINE2		0x09
#define REG_DRIVE_LINE3		0x0A
#define REG_DRIVE_LINE4		0x0B
#define REG_DRIVE_LINE5		0x0C
#define REG_DRIVE_LINE6		0x0D
#define REG_DRIVE_LINE7		0x0E
#define REG_DRIVE_LINE8		0x0F
#define REG_DRIVE_LINE9		0x10
#define REG_DRIVE_LINE10	0x11
#define REG_DRIVE_LINE11	0x12
#define REG_DRIVE_LINE12	0x13
#define REG_DRIVE_LINE13	0x14
#define REG_DRIVE_LINE14	0x15
#define REG_DRIVE_LINE15	0x16
#define REG_DRIVE_LINE16	0x17
#define REG_DRIVE_LINE17	0x18
#define REG_DRIVE_LINE18	0x19
#define REG_DRIVE_LINE19	0x1A
#define REG_DRIVE_LINE20	0x1B
#define REG_DRIVE_LINE21	0x1C
#define REG_DRIVE_LINE22	0x1D
#define REG_OP_MODE			0x25
#define REG_FREQ_HOPPING	0x27
#define REG_SENSE_OFFSET	0x28
#define REG_INT_TIMING		0x30
#define REG_MIN_AREA		0x33
#define REG_MIN_LEVEL		0x34
#define REG_MIN_WEIGHT		0x35
#define REG_MAX_AREA		0x36
#define REG_CG_METHOD		0x3A
#define REG_EDGE_SUPPRESS1	0x47
#define REG_EDGE_SUPPRESS2	0x48
#define REG_EDGE_SUPPRESS3	0x49
#define REG_EDGE_SUPPRESS4	0x4A
#define REG_HOP_LEVEL		0x50
#define REG_PRESS_SCALE		0x57
#define REG_ORIENTATION		0x65
#define REG_X_SCALING		0x66
#define REG_Y_SCALING		0x67
#define REG_X_OFFSET		0x68
#define REG_Y_OFFSET		0x69
#define REG_TOUCH_STATUS	0x79
#define REG_EVENT_MASK		0x7A
#define REG_IRQ_MSK			0x7B
#define REG_FINGER00		0x7C
#define REG_FINGER01		0x7D
#define REG_FINGER02		0x7E
#define REG_FINGER03		0x7F
#define REG_FINGER04		0x80
#define REG_FINGER05		0x81
#define REG_FINGER06		0x82
#define REG_FINGER07		0x83
#define REG_FINGER08		0x84
#define REG_FINGER09		0x85
#define REG_EDGE_REMAP		0x8B
#define REG_INIT_RST		0xA2
#define REG_DRIVE_LEVEL		0xD5
#define REG_ADC_RANGE_SEL	0xD7
#define REG_BIAS_RES		0xD8
#define REG_INTG_CAP		0xDB

typedef struct {
	uint8_t command;	//command byte to send
	uint16_t val;		//16bit value to send
} I2C_InitMap;



/**
 * @brief   touch_t describes the raw data packet to be feteched from SSD2541 with I2C
 */ 
typedef union {
	struct
	 {
	  uint8_t lo_x;   	//data[0], low byte of x-coordinates
	  uint8_t lo_y;   	//data[1], low byte of y-coordinates
	  uint8_t hi_y : 4; 	//lower nibble of data[2], high 4 bits of y
	  uint8_t hi_x : 4; 	//higher nibble of data[2], high 4 bits of x
	  uint8_t pressure;   	//data[3]
	 }finger;

	 uint8_t data[4];
} touch_t;

#ifdef __cplusplus
extern "C" {
#endif
/**
 * *********************** API ***********************
 * ***************************************************
 */ 
void SSD2541_begin(void);
void SSD2541_hwReset(void);
uint16_t SSD2541_readDeviceId(void);
bool SSD2541_getPoint(int16_t *x, int16_t *y, uint8_t *pressure);
bool SSD2541_getTouchFlag(void);
void SSD2541_clearTouchFlag(void);
#ifdef	__cplusplus
}
#endif

#endif
