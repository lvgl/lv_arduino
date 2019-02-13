#include "SSD2541.h"

/**
 * @note  Initialization map to be sent to SSD2541 on POR
 */
const I2C_InitMap SSD2541CfgTable[]=
{
		{REG_SW_RESET, 			0x0000},	 //sw reset
		{REG_NOP,				0x012C},	 //wait for 300ms (0x12C=300), use NO Operation command for a delay
	    {REG_DRIVE_SENSE_NO, 	0x0505},     //Drive/Sense number set 6 for 1.54" CTP (register value = number -1)
	    {REG_SENSE_OFFSET, 		0x0002},     //offset for sense line, hardware dependent
	    {REG_DRIVE_LINE0, 		0x000B},     //Select Drive11 as Tx1 for Drive Line 0
	    {REG_DRIVE_LINE1, 		0x000C},     //Drive 12 > Drive Line 1
	    {REG_DRIVE_LINE2, 		0x000D},     //Drive 13 > Drive Line 2
	    {REG_DRIVE_LINE3, 		0x000E},     //Drive 14 > Drive Line 3
	    {REG_DRIVE_LINE4, 		0x000F},     //Drive 15 > Drive Line 4
	    {REG_DRIVE_LINE5, 		0x0010},     //Drive 16 > Drive Line 5
	    {0xC1, 					0x0002},     //magic number, reserved
	    {REG_DRIVE_LEVEL,		0x0000},     //Drive Voltage Level, set vout = 5.5V
	    {REG_INT_TIMING,		0x0609},
	    {REG_MIN_LEVEL,			0xB664},
	    {REG_ADC_RANGE_SEL,	0x0004},
	    {REG_BIAS_RES,			0x0004},
	    {REG_INTG_CAP,			0x0007},
	    {0x2a,					0x700F},	//magic number
		{0x2b, 					0x17DD},	//magic number
		{0x37,					0x07C4},	//magic number
	    {0x38,					0x0002},
	    {0x56,					0x8010},
	    {0x59,					0xFF31},
	    {REG_EVENT_MASK,		0xffff},
	    {REG_IRQ_MSK,			0xFFE3},	//how many fingers to detect? it was 0x0003 from Joe's code
	    {0x40,					0x10C8},
	    {0x41,					0x0050},
	    {0x42,					0x001C},
	    {0x43,					0x0050},
	    {0x44,					0x001C},
	    {0x45,					0x0050},
	    {0x46,					0x101F},
	    {REG_ORIENTATION,		0x0003},
	    {REG_X_SCALING,			0x2800},	//REG_X_SCALING & REG_Y_SCALING are scaling factors for screen resolution 240x240
	    {REG_Y_SCALING,			0x2800},
	    {0x8a,					0x0002},
	    {REG_OP_MODE,			0x0008},
};

/**
 * ****************** Local members ******************
 * ***************************************************
 */
static int16_t  tp_last_x = 4095, tp_last_y = 4095;
volatile bool   isr_tp_flag = false;
static uint16_t getTouchStatus(void);
static void hal_gpioSetLevel(uint8_t pin, bool level);
static void hal_delay(uint32_t ms);
static void hal_readRegister(uint8_t reg, uint8_t* data, uint32_t cnt);
static void hal_writeRegister(uint8_t reg, uint16_t val);

/**
 * @brief Intialization function for SSD2541. Need to call this function before usage.
 */
void SSD2541_begin(void)
{
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = PIN_CTP_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = PIN_CTP_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000UL;
    i2c_param_config(i2c_master_port, &conf);
    esp_err_t err = i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
    ESP_ERROR_CHECK(err);

    //write initialization table
    for(uint32_t index=0; index<(sizeof(SSD2541CfgTable)/sizeof(I2C_InitMap)); index++)
    {
        if(SSD2541CfgTable[index].command==REG_NOP)
    		  hal_delay(SSD2541CfgTable[index].val);
    	  else
    		  hal_writeRegister(SSD2541CfgTable[index].command, SSD2541CfgTable[index].val);
    }
}

/**
 * @brief Toggle reset input for SSD2541
 */
void SSD2541_hwReset(void)
{
  hal_gpioSetLevel(PIN_CTP_RST, 0);
  hal_delay(10);
  hal_gpioSetLevel(PIN_CTP_RST, 1);
  hal_delay(10);
}


/**
 * @brief Return Device ID of SSD2541
 * @return Device ID as 0x2541
 */ 
uint16_t SSD2541_readDeviceId(void)
{
    uint8_t device_id[2];

    hal_readRegister(REG_DEVICE_ID, (uint8_t *)&device_id, 2);

    return ((uint16_t)device_id[0] << 8 | device_id[1]);
}

/**
 * @brief This function returns the last finger coordinates and the current pressure value.
 * @param *x, *y, & *pressure are pointers to variables to store position and pressure of the finger
 * @return  false if no valid touch
 *          true if a valid finger is detected
 *          
 * @note    Example to use<br>
 *          #include "SSD2541.h"
 *          int16_t finger_x, finger_y;
 *          uint8_t pressure;
 *          void setup(){
 *            Serial.begin(115200);
 *            SSD2541_begin();
 *          }
 *          void loop() {
 *            delay(500);            
 *            SSD2541_getPoint(&finger_x, &finger_y, &pressure);
 *            //Observe Serial Monitor for printout of the current touch coordinates and pressure
 *            Serial.printf("Touch x,y,pressure = %d,%d,%d\n", finger_x, finger_y, pressure);
 *            }
 */
bool SSD2541_getPoint(int16_t *x, int16_t *y, uint8_t *pressure)
{
	if((x==NULL) || (y==NULL)) return false;

	bool valid = false;
  
	touch_t touch;
	memset(&touch, 0, sizeof(touch_t));
	int16_t finger_x=0, finger_y=0;

	if((getTouchStatus()&0x1000) == 0x1000) //a single touch is valid
	{
		//snippet here is executed once on POR therefore tp_last_x/y are set to 0xfff when no finger is detected
		hal_readRegister(REG_FINGER00, (uint8_t *)touch.data, 4);
		finger_x = (int16_t)touch.finger.lo_x + ((int16_t)touch.finger.hi_x<<8);
		finger_y = (int16_t)touch.finger.lo_y + ((int16_t)touch.finger.hi_y<<8);
		if((finger_x <= CTP_SIZE_X) | (finger_y <=CTP_SIZE_Y)){	//check if touch values fit the screen size. Sometimes finger_x/y returns 0xfff!
		tp_last_x = *x = finger_x;
		tp_last_y = *y = finger_y;
		if(pressure!=NULL){*pressure = touch.finger.pressure;}	//check if user is passing a NULL to *pressure argument because pressure is optional
		}
	valid = true;
	}else{
		//when a finger is released this is the place to go...
		if(pressure!=NULL) {*pressure = 0;}
		*x = tp_last_x; //this is to suit LittlevGL as the last finger position is required for porting
		*y = tp_last_y; //it is also possible to return -1 when a finger has been lifted to indicate the current position
	valid = false;
	}

	return valid;
}

/**
 * @brief Returns Touch Flag (isr_tp_flag)
 * @return  true if a finger is detected
 *          false if no finger is detected.
 */
bool SSD2541_getTouchFlag(void)
{ 
  return isr_tp_flag;
}

/**
 * @brief This function disable interrupt first, then clear the Touch Flag (isr_tp_flag), and re-enable interrupt for next touch
 */
void SSD2541_clearTouchFlag(void)
{
  //ESP32 IDF
  //...pending
}

/**
 * @brief	Touch Status return by reading STATUS REGISTER R79h
 * @param	None
 * @return	Register value in 16-bit width with buf[1] shift to higher byte
 *
 * 			Register value R79h contains the bits for Finger00 - Finger09 plus few abnormality bits.
 * 			Example, getTouchStatus() = 0x1000 if Finger00 (the first touch) is detected
 * 			getTouchStatus() = 0x3000 if Finger01 & Finger00 is detected, i.e. two fingers.
 * 			If 1st finger is released getTouchStatus() = 0x2000; else,
 *          if 2nd finger is released getTouchStatus() returns 0x1000.
 *      When a valid finger is detected the interrupt will be set low from high (H->L edge) until
 *      this STATUS register is read, the pin will be reset to high by reading REG_TOUCH_STATUS.
 */
uint16_t getTouchStatus(void)
{
    uint8_t buf[2];
    hal_readRegister(REG_TOUCH_STATUS, (uint8_t *)&buf, 2);
    
    uint16_t sta = (uint16_t)buf[1]<<8 | buf[0];
  
    return sta;
}

/**
 * @brief HAL function to set/clear GPIO level.
 *        Should have set the pin as an output in SSD2541_begin().
 */
static void hal_gpioSetLevel(uint8_t pin, bool level)
{
	gpio_set_level((gpio_num_t)pin, (uint32_t)level);
}

/**
 * @brief   HAL delay function for millisec
 */ 
void hal_delay(uint32_t ms)
{
    vTaskDelay(ms/portTICK_PERIOD_MS);
}

/**
 * @brief   HAL I2C read from SSD2541
 * @param   *data is a pointer to buffer to store bytes returned
 * @param   cnt is the byte count
 * @note    Example : 
 *          uint8_t buffer[2];
 *          hal_readRegister(REG_DEVICE_ID, (uint8_t *)&buf, 2);
 *          //buffer[1][0] = [41h][25h]
 */ 
void hal_readRegister(uint8_t reg, uint8_t* data, uint32_t cnt)
{
	if(cnt==0) return;

	esp_err_t err;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SSD2541_I2C_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (SSD2541_I2C_ADDR << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
	if(cnt > 1){
		i2c_master_read(cmd, data, cnt - 1, ACK_VAL);
	}
	i2c_master_read_byte(cmd, data + cnt - 1, NACK_VAL);
	i2c_master_stop(cmd);
	err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  ESP_ERROR_CHECK(err);
	i2c_cmd_link_delete(cmd);
}

/**
 * @brief	HAL I2C write to SSD2541
 * @param	reg is the register to write to
 * @param	val is a 16 bit value to write
 */ 
void hal_writeRegister(uint8_t reg, uint16_t val)
{
    esp_err_t err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SSD2541_I2C_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, (uint8_t)(val>>8), ACK_CHECK_EN);
	i2c_master_write_byte(cmd, (uint8_t)val&0xff, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    ESP_ERROR_CHECK(err);
    i2c_cmd_link_delete(cmd);
}
