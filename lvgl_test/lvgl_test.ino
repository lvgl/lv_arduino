#include <lvgl.h>

void my_disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_array)
{

  /*TODO copy 'color_array' to the specifed coordinats of your disaply.*/


  /*Tell the flushing is ready*/
  lv_flush_ready();
}


bool my_tp_read(lv_indev_data_t *data)
{
  bool tp_is_pressed = false; /*TODO read here the state of toush pad*/
  int16_t last_x = 0;
  int16_t last_y = 0;
  
  if(tp_is_pressed) {
    /*Touch pad is being pressed now*/
    last_x = 0;       /*TODO save the current X coordinate*/
    last_y = 0;       /*TODO save the current Y coordinate*/  
  }

  
  data->point.x = last_x;
  data->point.y = last_y;
  data->state = tp_is_pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;

  return false;       /*Return false because no moare to be read*/
}

void hal_init(void)
{
    /*Initialize the display*/
    lv_disp_drv_t disp_drv;
    disp_drv.disp_flush = my_disp_flush;
    lv_disp_drv_register(&disp_drv);

    
    /*Initialize the touch pad*/
    lv_indev_drv_t indev_drv;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read = my_tp_read;
    lv_indev_drv_register(&indev_drv);

    /*Initialize the graphics library's tick*/
    /*In a Timer call 'lv_tick_inc(1)' in every milliseconds
      Or no other option call it loop */
}

void setup() {
  lv_init();

  hal_init();

  lv_obj_t *label = lv_label_create(lv_scr_act(), NULL);
  lv_label_set_text(label, "Hello Arduino!");
  lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);

}

void loop() {
  // put your main code here, to run repeatedly:

  lv_task_handler();
  /*If 'lv_tick_inc(5)' is not called in a Timer then call it here*/
  delay(5);             /*Wait a little*/
}
