#include <lvgl.h>
#include <Ticker.h>
#include <TFT_eSPI.h>
#include "demo.h"

#define LVGL_TICK_PERIOD 20

Ticker tick; /* timer for interrupt handler */
TFT_eSPI tft = TFT_eSPI(); /* TFT instance */
static lv_disp_buf_t disp_buf;
static lv_color_t buf[LV_HOR_RES_MAX * 10];

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint16_t c;

  tft.startWrite(); /* Start new TFT transaction */
  tft.setAddrWindow(area->x1, area->y1, (area->x2 - area->x1 + 1), (area->y2 - area->y1 + 1)); /* set the working window */
  for (int y = area->y1; y <= area->y2; y++) {
    for (int x = area->x1; x <= area->x2; x++) {
      c = color_p->full;
      tft.writeColor(c, 1);
      color_p++;
    }
  }
  tft.endWrite(); /* terminate TFT transaction */
  lv_disp_flush_ready(disp); /* tell lvgl that flushing is done */
}

static void lv_tick_handler(void)
{

  lv_tick_inc(LVGL_TICK_PERIOD);
}

/* Reading input device (simulated encoder here) */
bool read_touchscreen(lv_indev_drv_t * indev, lv_indev_data_t * data)
{

  uint16_t x = 0, y = 0; // To store the touch coordinates

  // Pressed will be set true is there is a valid touch on the screen
  boolean pressed = tft.getTouch(&x, &y);

  data->point.x = x;
  data->point.y = y;
  data->state = pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
  return false; /*No buffering now so no more data read*/
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); /* prepare for possible serial debug */

  lv_init();

  tft.begin(); /* TFT init */
  tft.setRotation(3); /* Landscape orientation */

  lv_disp_buf_init(&disp_buf, buf, NULL, LV_HOR_RES_MAX * 10);

  /*Initialize the display*/
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = 320;
  disp_drv.ver_res = 240;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.buffer = &disp_buf;
  lv_disp_drv_register(&disp_drv);


  /*Initialize the touch pad*/
  lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = read_touchscreen;
  lv_indev_drv_register(&indev_drv);

  /*Initialize the graphics library's tick*/
  tick.attach_ms(LVGL_TICK_PERIOD, lv_tick_handler);

  //  lv_tutorial_hello_world();
  demo_create();

  //  /* Create simple label */
  //  lv_obj_t *label = lv_label_create(lv_scr_act(), NULL);
  //  lv_label_set_text(label, "Hello Arduino! (V6.0)");
  //  lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);
  //
  //  lv_obj_t * btn = lv_btn_create(lv_scr_act(), NULL);     /*Add a button the current screen*/
  //  lv_obj_set_pos(btn, 10, 10);                            /*Set its position*/
  //  lv_obj_set_size(btn, 100, 50);                          /*Set its size*/
  //  //lv_obj_set_event_cb(btn, btn_event_cb);                 /*Assign a callback to the button*/
  //
  //  label = lv_label_create(btn, NULL);          /*Add a label to the button*/
  //  lv_label_set_text(label, "Button");                     /*Set the labels text*/

}

void loop() {
  // put your main code here, to run repeatedly:
  lv_task_handler(); /* let the GUI do its work */
  delay(5);
}
