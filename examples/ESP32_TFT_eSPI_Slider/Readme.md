# Example for lv_arduino using a slider

This example has the screen set to 320x480, change this by altering the following lines in the main ino file:

int screenWidth = 480;
int screenHeight = 320;

## Getting Started

Change pin 32 t your preferred backlight or remove the following:

  ledcSetup(10, 5000/*freq*/, 10 /*resolution*/);
  ledcAttachPin(32, 10);
  analogReadResolution(10);
  ledcWrite(10,768);

##Theme selection

Change the following to change the theme:

lv_theme_t * th = lv_theme_night_init(210, NULL);     //Set a HUE value and a Font for the Night Theme
   lv_theme_set_current(th);
