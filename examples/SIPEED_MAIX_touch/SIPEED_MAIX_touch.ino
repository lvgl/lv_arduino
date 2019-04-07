#include <lvgl.h>
#include <Ticker.h>
#include <Sipeed_ST7789.h>
#include <touchscreen.h>

#define LVGL_TICK_PERIOD 20

SPIClass spi_(SPI0);// MUST be SPI0 for Maix series on board LCD
Ticker tick; /* timer for interrupt handler */
Sipeed_ST7789 lcd(320, 240, spi_);
TouchScreen touchscreen;
int ledstate = 1;
#if USE_LV_LOG != 0
/* Serial debugging */
void my_print(lv_log_level_t level, const char * file, uint32_t line, const char * dsc)
{

  Serial.printf("%s@%d->%s\r\n", file, line, dsc);
  delay(100);
}
#endif

/* Display flushing */
void disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_p) {
  int32_t w = x2-x1+1;
  int32_t h = y2-y1+1;
  int32_t x,y;
  int32_t i=0;
  uint16_t* data = (uint16_t*)malloc( w*h*2 );
  uint16_t* pixels = data;

  for(y=y1; y<=y2; ++y)
  {
    for(x=x1; x<=x2; ++x)
    {
      pixels[i++]= (color_p->red<<3) | (color_p->blue<<8) | (color_p->green>>3&0x07 | color_p->green<<13);
      // or LV_COLOR_16_SWAP = 1
       ++color_p;
    }
  }
  lcd.drawImage((uint16_t)x1, (uint16_t)y1, (uint16_t)w, (uint16_t)h, data);
  free(data);
  lv_flush_ready(); /* tell lvgl that flushing is done */
}

/* Interrupt driven periodic handler */
static void lv_tick_handler(void)
{

  lv_tick_inc(LVGL_TICK_PERIOD);
}

/* Reading input device  */
bool read_touchscreen(lv_indev_data_t * data)
{
  int status, x, y;
  touchscreen.read();
  status = touchscreen.getStatus();
  x = touchscreen.getX();
  y = touchscreen.getY();
  switch(status)
  {
    case TOUCHSCREEN_STATUS_RELEASE:
        data->state =  LV_INDEV_STATE_REL;
      break;
    case TOUCHSCREEN_STATUS_PRESS:
    case TOUCHSCREEN_STATUS_MOVE:
      data->state = LV_INDEV_STATE_PR;
      break;
    default:
      return false;
  }
  data->point.x = x;
  data->point.y = y;
  return false;
}

static lv_res_t btn_click_action(lv_obj_t * btn)
{
    uint8_t id = lv_obj_get_free_num(btn);
    
    ledstate = !ledstate;
    digitalWrite(LED_BUILTIN,ledstate);

    /* The button is released.
     * Make something here */

    return LV_RES_OK; /*Return OK if the button is not deleted*/
}

void setup() {

  Serial.begin(115200); /* prepare for possible serial debug */
  lcd.begin(15000000, COLOR_WHITE);
  touchscreen.begin();
  lv_init();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN,ledstate);//power off led
#if USE_LV_LOG != 0
  lv_log_register_print(my_print); /* register print function for debugging */
#endif

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.disp_flush = disp_flush;
  lv_disp_drv_register(&disp_drv);


  /*Initialize the touch pad*/
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read = read_touchscreen;
  lv_indev_drv_register(&indev_drv);

  /*Initialize the graphics library's tick*/
  tick.attach_ms(LVGL_TICK_PERIOD, lv_tick_handler);

  /* Create simple label */
  /*Create a title label*/
  lv_obj_t * label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(label, "Default buttons");
  lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);

  /*Create a normal button*/
  lv_obj_t * btn1 = lv_btn_create(lv_scr_act(), NULL);
  lv_cont_set_fit(btn1, true, true); /*Enable resizing horizontally and vertically*/
  lv_obj_align(btn1, label, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
  lv_obj_set_free_num(btn1, 1);   /*Set a unique number for the button*/
  lv_btn_set_action(btn1, LV_BTN_ACTION_CLICK, btn_click_action);

  /*Add a label to the button*/
  label = lv_label_create(btn1, NULL);
  lv_label_set_text(label, "Normal");

  /*Copy the button and set toggled state. (The release action is copied too)*/
  lv_obj_t * btn2 = lv_btn_create(lv_scr_act(), btn1);
  lv_obj_align(btn2, btn1, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
  lv_btn_set_state(btn2, LV_BTN_STATE_TGL_REL);  /*Set toggled state*/
  lv_obj_set_free_num(btn2, 2);               /*Set a unique number for the button*/

  /*Add a label to the toggled button*/
  label = lv_label_create(btn2, NULL);
  lv_label_set_text(label, "Toggled");

  /*Copy the button and set inactive state.*/
  lv_obj_t * btn3 = lv_btn_create(lv_scr_act(), btn1);
  lv_obj_align(btn3, btn2, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
  lv_btn_set_state(btn3, LV_BTN_STATE_INA);   /*Set inactive state*/
  lv_obj_set_free_num(btn3, 3);               /*Set a unique number for the button*/

  /*Add a label to the inactive button*/
  label = lv_label_create(btn3, NULL);
  lv_label_set_text(label, "Inactive");
}


void loop() {

  lv_task_handler(); /* let the GUI do its work */
  delay(5);
}
