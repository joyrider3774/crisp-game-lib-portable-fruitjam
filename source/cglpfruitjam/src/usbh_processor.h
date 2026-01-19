#ifndef USBH_PROCESSOR_H
#define USBH_PROCESSOR_H

#include "Adafruit_TinyUSB.h"

typedef void (*onKeyboardKeyDownUpCallback)(uint8_t scancode,  bool keydown); 

#define GAMEPAD_NONE  0
#define GAMEPAD_LEFT  (1 << 0)
#define GAMEPAD_RIGHT (1 << 1)
#define GAMEPAD_UP    (1 << 2)
#define GAMEPAD_DOWN  (1 << 3)
#define GAMEPAD_A     (1 << 4)
#define GAMEPAD_B     (1 << 5)
#define GAMEPAD_X     (1 << 6)
#define GAMEPAD_Y     (1 << 7)
#define GAMEPAD_LEFT_SHOULDER  (1 << 8)
#define GAMEPAD_RIGHT_SHOULDER (1 << 9)
#define GAMEPAD_SELECT (1 << 10)
#define GAMEPAD_START (1 << 11)




bool gamepadButtonPressed(uint32_t button);
void setKeyDownUpCallBack(onKeyboardKeyDownUpCallback callback);
void setMouseRange(int16_t minx, int16_t miny, int16_t w, int16_t h);
int16_t getMouseX();
int16_t getMouseY();
bool mouseButtonPressed(uint8_t button);
bool keyPressed(uint8_t key);
void USBHidUpdate(Adafruit_USBH_Host *host) ;
const char* getKeyName(uint8_t key);
void setMouse(int16_t x, int16_t y);

#endif