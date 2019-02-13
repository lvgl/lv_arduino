#include "SSD2805_VSPI_drv.h"

#ifndef max
#define max(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef min
#define min(a,b)            (((a) < (b)) ? (a) : (b))
#endif

/**
 * @brief SPI pre transmission callback function to set DC pin for command(0) or data(1)
 */ 
static void spiPreTxCallback(spi_transaction_t *t)
{
  int dc = (int)t->user;
  gpio_set_level((gpio_num_t)PIN_NUM_SDC, dc);
}

static inline void setDC(bool v){(v==1)?(gpio_set_level((gpio_num_t)PIN_NUM_SDC,1)):(gpio_set_level((gpio_num_t)PIN_NUM_SDC,0));}
static inline void setCS(bool v){(v==1)?(gpio_set_level((gpio_num_t)PIN_NUM_CS,1)):(gpio_set_level((gpio_num_t)PIN_NUM_CS,0));}
static void setRST(bool v){(v==1)?(gpio_set_level((gpio_num_t)PIN_NUM_RST,1)):(gpio_set_level((gpio_num_t)PIN_NUM_RST,0));}
    
static void writeRegister(uint8_t reg, const uint8_t *data8, uint32_t len);
static void writeCmd(uint8_t cmd);
static void setTDCSize(uint32_t byteCount);
static bool setRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

static spi_device_handle_t ssd2805;
static spi_transaction_t trans[MAX_TRANS_NUM];

/**
 * @brief Like conventional Arduino driver, this begin() method should be called before using D2805
 */
void SSD2805_begin(void)
{
    esp_err_t err;
    spi_bus_config_t  bus_config;
    spi_host_device_t host;
    spi_device_interface_config_t dev_config;
    
    gpio_set_direction((gpio_num_t)PIN_NUM_SDC, (gpio_mode_t)GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)PIN_NUM_RST, (gpio_mode_t)GPIO_MODE_OUTPUT);

    SSD2805_hwReset();

    (SPI_NUM==VSPI)?(host=VSPI_HOST):(host=HSPI_HOST);
    
    {
      bus_config.mosi_io_num = PIN_NUM_MOSI;
      bus_config.miso_io_num = PIN_NUM_MISO;
      bus_config.sclk_io_num = PIN_NUM_SCK;
      bus_config.quadwp_io_num = -1;
      bus_config.quadhd_io_num = -1;
      bus_config.max_transfer_sz = MAX_DMA_LEN; // in byte count
    }

    dev_config.flags = SPI_DEVICE_HALFDUPLEX;
    dev_config.mode = 0;
    dev_config.clock_speed_hz = 2000000UL;  //40MHz works with SPI_DEVICE_HALFDUPLEX, useful for speeding up block write after PLL enabled
    dev_config.spics_io_num = PIN_NUM_CS;   //use GPIO for chip select
    dev_config.queue_size = MAX_TRANS_NUM;  //only 1 transaction at any time. 
    dev_config.pre_cb = spiPreTxCallback;   //pre transmission callback to clear/set SDC line for cmd/data

    err=spi_bus_initialize(host, &bus_config, 1);
    ESP_ERROR_CHECK(err);
    err = spi_bus_add_device(host, &dev_config, &ssd2805);
    ESP_ERROR_CHECK(err);

    //Step 1: set PLL
    uint8_t txbuffer[12];
    txbuffer[0]=0x0f; txbuffer[1]=0x00;

    writeRegister(0xba, (const uint8_t *)&txbuffer, 2); //PLL 	= clock*MUL/(PDIV*DIV)
                                                        //		= clock*(BAh[7:0]+1)/((BAh[15:12]+1)*(BAh[11:8]+1))
                                                        //		= 20*(0x0f+1)/1*1 = 20*16 = 320MHz
                                                        //Remark: 350MHz >= fvco >= 225MHz for SSD2805 since the max. speed per lane is 350Mbps

    txbuffer[0]=0x01; txbuffer[1]=0x00;
    writeRegister(0xb9, (const uint8_t *)&txbuffer, 2);       //enable PLL
    
    vTaskDelay(10/portTICK_PERIOD_MS);  ;   //delay 10ms to wait for PLL stablize

    //Step 2: Now it is safe to set SPI up to 40MHz (remove it for now with strip wire prototype )
    //equivalent to calling SPI.setFrequency(40000000UL);   //now we can set SPI clock up to 40MHz (PLL clock=320/8)
    spi_bus_remove_device(ssd2805); //has to remove device before switching to a faster SPI clock
    spi_bus_free(host);
    dev_config.clock_speed_hz = 40000000UL;
    err=spi_bus_initialize(host, &bus_config, 1); ESP_ERROR_CHECK(err);
    err = spi_bus_add_device(host, &dev_config, &ssd2805); ESP_ERROR_CHECK(err);
  
    //Step 3: set clock control register for SYS_CLK & LP clock speed
    //SYS_CLK = TX_CLK/(BBh[7:6]+1), TX_CLK = external oscillator clock speed
    //In this case, SYS_CLK = 20MHz/(1+1)=10MHz. Measure SYS_CLK pin to verify it.
    //LP clock = PLL/(8*(BBh[5:0]+1)) = 320/(8*(4+1)) = 8MHz, conform to AUO panel's spec, default LP = 8Mbps
    //S6D04D2 is the controller of AUO 1.54" panel.
    txbuffer[0]=0x44; txbuffer[1]=0x00;
    writeRegister(0xbb, (const uint8_t *)&txbuffer, 2);

    txbuffer[0]=0x00; txbuffer[1]=0x01;
    writeRegister(0xd6, (const uint8_t *)&txbuffer, 2);   //output sys_clk for debug. Now check sys_clk pin for 10MHz signal

    //Step 4: Set MIPI packet format
    txbuffer[0]=0x43; txbuffer[1]=0x02;
    writeRegister(0xb7, (const uint8_t *)&txbuffer, 2);   //EOT packet enable, write operation, it is a DCS packet
                                                    //HS clock is disabled, video mode disabled, in HS mode to send data
    //Step 5: set Virtual Channel (VC) to use
    txbuffer[0]=0x00; txbuffer[1]=0x00;
    writeRegister(0xb8, (uint8_t *)&txbuffer, 2);

    //Step 6: Now write DCS command to AUO panel for system power-on upon reset
    setTDCSize(0);
    writeRegister(DCS_SLPOUT, NULL, 0);

    vTaskDelay(100/portTICK_PERIOD_MS);					//wait for AUO/LG  panel after sleepout(Important:cannot use delay(100)!)

    //Step 7: Now configuration parameters sent to AUO/LG panel for 16-bits/pixel
    setTDCSize(1);
    txbuffer[0] = 0x05;
    writeRegister(DCS_COLMOD, (const uint8_t *)&txbuffer, 1); //Set 0x3a to 0x05 (16-bit color)

    //Step 8: Set TE line and VBP time

    setTDCSize(2);
    txbuffer[0]=0x5A; txbuffer[1]=0x5A;
    writeRegister(0xF1, (const uint8_t *)&txbuffer, 2);  //unlock MCS

    //Set MCS register at 0xF2 to set VBP, VFP, and TE time
    //Setup here establishes a TE time sync. with VBP = 5.64ms with display time = 14.40ms
    //leading to a frame rate of 50fps. Calculation can be seen from an Excel sheet
    //Result verified with a DSO. This is a significant result as we may now sync with a slower mcu for flicker-free display.
    // *** Add support for TE and VBP time set here ***
    //Trying larger NVBP and NVFP values for slower framerate with blank time =25.56ms, display time =14.40ms
 
    setTDCSize(12);
    txbuffer[0]=0x00;     //NL=0x00 (default)
    txbuffer[1]=0xF0;     //NHW=0xF0 (default for 240 clocks per line)
    txbuffer[2]=0x03;     //frame inversion etc. leave it default
    txbuffer[3]=0x56;     //NVBP=0x56 (.86) for a vertical back porch in normal mode
    txbuffer[4]=0x08;     //NVFP=8 for a vertical front porch in normal mode
    txbuffer[5]=0x00;     //RSEL[1:0]=00 for 240x240 resolution
    txbuffer[6]=0x01;     //TE = ON
    txbuffer[7]=0x00;     //TE_ST set zero for TE signal starts on internal VS
    txbuffer[8]=0x00;
    txbuffer[9]=0x00;     //TE_ED set the same value as NVBP to synchronize with NVBP
    txbuffer[10]=0x56;
    txbuffer[11]=0x03;    //set TE line as active high
    writeRegister(0xF2, (const uint8_t *)&txbuffer, 12);  //Set DISPCTL for porch and TE values
  
    setTDCSize(2);
    txbuffer[0]=0xA5; txbuffer[1]=0xA5;
    writeRegister(0xF1, (const uint8_t *)&txbuffer, 2);  //lock MCS

    SSD2805_clearLCD(0x0000); //clearLCD after DCS_DISPON if TE pin is to sync with MCU write

    setTDCSize(0);
    writeRegister(DCS_DISPON, NULL, 0);                  //display ON DCS command to AUO panel    
}

void writeCmd(uint8_t cmd)
{
    memset(&trans[0], 0, sizeof(spi_transaction_t));
    trans[0].length = 8;
    trans[0].user = (void*)0; //SDC pin low for command
    trans[0].flags = SPI_TRANS_USE_TXDATA;
    trans[0].tx_data[0] = cmd;
    esp_err_t err = spi_device_transmit(ssd2805, &trans[0]); 
    ESP_ERROR_CHECK(err);
}

void writeRegister(uint8_t reg, const uint8_t *data8, uint32_t len)
{
    memset(&trans[0], 0, sizeof(spi_transaction_t));

    trans[0].length = 8;
    trans[0].user = (void*)0; //SDC pin low for command
    trans[0].flags = SPI_TRANS_USE_TXDATA;
    trans[0].tx_data[0] = reg;

    //setCS(0);   //chip select manually not required because spi_device_transmit will do it automatically
    esp_err_t err = spi_device_transmit(ssd2805, &trans[0]); 
    ESP_ERROR_CHECK(err);

    if(len!=0)
    {
    trans[0].length = 8*len;
    trans[0].user = (void*)1; //SDC pin high for data
    trans[0].flags = 0;
    trans[0].tx_buffer = data8;
    err = spi_device_transmit(ssd2805, &trans[0]); 
    ESP_ERROR_CHECK(err);
    }
}

/**
 * @brief Set packet size sending to mipi display in byte count
 */ 
void setTDCSize(uint32_t byteCount)
{
    uint8_t txbuffer[2];
    txbuffer[0] = (uint8_t)byteCount;
    txbuffer[1] = (uint8_t)(byteCount>>8);

    writeRegister(0xbc, (const uint8_t *)&txbuffer, 2); //Set packet size TDC[15:0]

    txbuffer[0] = (uint8_t)(byteCount>>16);
    txbuffer[1] = (uint8_t)(byteCount>>24);   

    writeRegister(0xbd, (const uint8_t *)&txbuffer, 2); //Set packet size TDC[31:16]
}

/**
 * @brief Set the area to draw on LCD
 * @param (x2,y2) is the top left corner coordinates, (x1,y1) is the lower right corner
 * @return 0 if failed, 1 if successful
 */
bool setRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
  if((x1>x2) || (y1>y2)) return 0;

  uint8_t txbuffer[4];
  
  uint16_t sc = min(x1, DISP_HOR_SIZE-1);
  uint16_t sp = min(y1, DISP_VER_SIZE-1);
  uint16_t ec = min(x2, DISP_HOR_SIZE-1);
  uint16_t ep = min(y2, DISP_VER_SIZE-1);
 
  setTDCSize(4);

  txbuffer[0] = (uint8_t)(sc>>8);  //SC[15:8]  start column
  txbuffer[1] = (uint8_t)sc;       //SC[7:0]
  txbuffer[2] = (uint8_t)(ec>>8);  //EC[15:8] end column
  txbuffer[3] = (uint8_t)ec;       //EC[7:0] 
  writeRegister(DCS_COL_SET, (const uint8_t *)&txbuffer, 4);

  txbuffer[0] = (uint8_t)(sp>>8); //SP[15:8]  start page
  txbuffer[1] = (uint8_t)sp;      //SP[7:0]
  txbuffer[2] = (uint8_t)(ep>>8); //EP[15:8] end page
  txbuffer[3] = (uint8_t)ep;      //EP[7:0]
  writeRegister(DCS_PAGE_SET, (const uint8_t *)&txbuffer, 4);

  return 1;
}

/**
 * @brief Toggle reset input for SSD2805
 */
void SSD2805_hwReset(void)
{
  setRST(0);
  vTaskDelay(10/portTICK_PERIOD_MS);
  setRST(1);
  vTaskDelay(10/portTICK_PERIOD_MS);
}

/**
 * @brief Clear the whole display with a color
 * @param color in 2 bytes per pixel (R4R3R2R1R0G5G4G3 G2G1G0B4B3B2B1B0)
 */ 
void SSD2805_clearLCD(uint16_t color)
{  
  SSD2805_dispFill(0,0,DISP_HOR_SIZE-1, DISP_VER_SIZE-1, color);
}

/**
 * @brief Fill display for a certain area with a color
 * @param (x2,y2) is the top left corner coordinates, (x1,y1) is the lower right corner 
 * @param color in 2 bytes per pixel (R4R3R2R1R0G5G4G3 G2G1G0B4B3B2B1B0)
 */ 
void SSD2805_dispFill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, uint16_t color)
{
  esp_err_t err;
  
  if(!setRectangle(x1,y1,x2,y2)) return;
  
  uint32_t byteCount2Flush = (x2-x1+1)*(y2-y1+1)*2;
  //printf("dispFlush::byteCount2Flush = %d\n", byteCount2Flush);
  setTDCSize(byteCount2Flush);
  
  uint32_t dmaBlockCount = 0;
  if(byteCount2Flush >= MAX_DMA_LEN)
  {
    dmaBlockCount = byteCount2Flush/MAX_DMA_LEN; 
    //printf("dispFlush::byteCount2Flush > MAX_DMA_LEN so, dmaBlockCount = %d\n", dmaBlockCount);
  }
  
  byteCount2Flush%=MAX_DMA_LEN;  //byteCount2Flush becomes the remainder
  writeCmd(DCS_RAMWR);

    uint16_t *vdb = (uint16_t *)heap_caps_malloc(MAX_DMA_LEN, MALLOC_CAP_DMA);
    assert(vdb!=NULL);

  for(uint32_t i=0; i<MAX_DMA_LEN/2; i++)
    vdb[i] = color;

  memset(&trans[0], 0, sizeof(spi_transaction_t));
/*
  err = gpio_intr_enable((gpio_num_t)PIN_NUM_TE); assert(err==ESP_OK);

  while(!TE_TriggerFlag)
  ;

  err = gpio_intr_disable((gpio_num_t)PIN_NUM_TE); assert(err==ESP_OK);
  TE_TriggerFlag = false;
*/
  //sync with TE pin pending...
  while(dmaBlockCount--){
    trans[0].length = MAX_DMA_LEN*8;
    trans[0].user = (void*)1; //SDC pin high for data
    trans[0].tx_buffer = vdb;
    err = spi_device_transmit(ssd2805, &trans[0]); 
    ESP_ERROR_CHECK(err);
  }

  if(byteCount2Flush){
    trans[0].length = byteCount2Flush*8;
    //trans[0].user = (void*)1; //SDC pin high for data
    //trans[0].tx_buffer = vdb;
    err = spi_device_transmit(ssd2805, &trans[0]); 
    ESP_ERROR_CHECK(err);
  }

  free(vdb);

}

/**
 * @brief Fill display for a certain area with data from flash
 * @param (x2,y2) is the top left corner coordinates, (x1,y1) is the lower right corner
 * @param *color_p is a pointer to data from Flash
 * @note  SSD2805 configure in 64k color
 *        1st write G2G1G0B4B3B2B1B0  green_l:3 blue:5
 *        2nd write R4R3R2R1R0G5G4G3  red:5 green_h:3
 *        e.g. uint16_t vdb[4] = {0xABCD, 0x1234, 0x5678, 0xaab6};  //color arranged in R4R3R2R1R0G5G4G3 G2G1G0B4B3B2B1B0
 *        SPI send in 8-bit in this order
 *        CDh->ABh->34h->12h->78h->56h->B6h->AAh
 */
void SSD2805_dispFlush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const uint16_t* color_p)
{
  esp_err_t err;

  if(!setRectangle(x1,y1,x2,y2)) return;
  
  uint32_t byteCount2Flush = (x2-x1+1)*(y2-y1+1)*2;

  setTDCSize(byteCount2Flush);
  
  uint32_t dmaBlockCount = 0;
  if(byteCount2Flush >= MAX_DMA_LEN)
  {
    dmaBlockCount = byteCount2Flush/MAX_DMA_LEN; 
  }
  
  byteCount2Flush%=MAX_DMA_LEN;  //byteCount2Flush becomes the remainder
  writeCmd(DCS_RAMWR);

  memset(&trans[0], 0, sizeof(spi_transaction_t));
/*
  err = gpio_intr_enable((gpio_num_t)PIN_NUM_TE); assert(err==ESP_OK);

  while(!TE_TriggerFlag)
  ;

  err = gpio_intr_disable((gpio_num_t)PIN_NUM_TE); assert(err==ESP_OK);
  TE_TriggerFlag = false;
*/
  //sync with TE pin pending...
  while(dmaBlockCount--){
    trans[0].length = MAX_DMA_LEN*8;
    trans[0].user = (void*)1; //SDC pin high for data
    trans[0].tx_buffer=(uint16_t *)color_p;
    err = spi_device_transmit(ssd2805, &trans[0]); 
    ESP_ERROR_CHECK(err);
    color_p+=(MAX_DMA_LEN/2);
  }

  if(byteCount2Flush){
    trans[0].length = byteCount2Flush*8;
    trans[0].user = (void*)1; //SDC pin high for data
    trans[0].tx_buffer=(uint16_t *)color_p;
    err = spi_device_transmit(ssd2805, &trans[0]); 
    ESP_ERROR_CHECK(err);
  }
}
