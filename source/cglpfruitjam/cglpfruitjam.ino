#include <stdlib.h>
#include <float.h>
#include <math.h>
#include <string.h>
#include <Adafruit_dvhstx.h>
#include <Adafruit_TinyUSB.h>
#include <pio_usb.h>

#include "src/lib/menu.h"
#include "src/lib/machineDependent.h"
#include "src/lib/menuGameList.h"
#include "src/lib/particle.h"
#include "src/lib/random.h"
#include "src/lib/sound.h"
#include "src/lib/textPattern.h"
#include "src/lib/vector.h"
#include "src/lib/cglp.h"
#include "src/i2stones.h"
#include "src/glcdfont.h"
#include "src/usbh_processor.h"

Adafruit_USBH_Host USBHost;

#if defined(ADAFRUIT_FEATHER_RP2350_HSTX)
DVHSTXPinout pinConfig = ADAFRUIT_FEATHER_RP2350_CFG;
#elif defined(ADAFRUIT_METRO_RP2350)
DVHSTXPinout pinConfig = ADAFRUIT_METRO_RP2350_CFG;
#elif defined(ARDUINO_ADAFRUIT_FRUITJAM_RP2350)
DVHSTXPinout pinConfig = ADAFRUIT_FRUIT_JAM_CFG;
#elif (defined(ARDUINO_RASPBERRY_PI_PICO_2) || defined(ARDUINO_RASPBERRY_PI_PICO_2W))
DVHSTXPinout pinConfig = ADAFRUIT_HSTXDVIBELL_CFG;
#else
// If your board definition has PIN_CKP and related defines,
// DVHSTX_PINOUT_DEFAULT is available
DVHSTXPinout pinConfig = DVHSTX_PINOUT_DEFAULT;
#endif

#define AUDIO_BUFFER_SIZE 4096
#define PIN_USB_HOST_VBUS (11u)

#define RIGHTKEY 0x4F
#define LEFTKEY 0x50
#define DOWNKEY 0x51
#define UPKEY  0x52

#define F1KEY 0x3A
#define F2KEY 0x3B
#define DKEY 0x07

// BUTTON3 = B Ingame
// BUTTON2 = back to menu ingame
// BUTTON1 = A Ingame

#define BUTTON1_KEY 0x06 //key C
#define BUTTON2_KEY 0x29 //key ESC
#define BUTTON3_KEY 0x1B //key X
#define BUTTON3_ALTERNATE 0x2C //key SPACE

#define BUTTON_1_PIN 5
#define BUTTON_2_PIN 4
#define BUTTON_3_PIN 0

#define BUTTON_1_MASK (1 << 0)
#define BUTTON_2_MASK (1 << 1)
#define BUTTON_3_MASK (1 << 2)

#define SAMPLERATE 44100

DVHSTX16 tft(pinConfig, DVHSTX_RESOLUTION_320x240, true);

uint16_t *fb;
static int8_t vol = 5;
static unsigned char clearColorR = 0;
static unsigned char clearColorG = 0;
static unsigned char clearColorB = 0;
const uint16_t timePerFrame =  1000000 / FPS; 
static float frameRate = 0;
static uint32_t currentTime = 0, lastTime = 0, frameTime = 0;
static int WINDOW_WIDTH = 0;
static int WINDOW_HEIGHT = 0;
static float scale = 1.0f;
static int viewW = 0;
static int viewH = 0;
static int origViewW = 0;
static int origViewH = 0;
static int offsetX = 0;
static int offsetY = 0;
static float wscale = 1.0f;
static int debugMode = 0;
static float mouseX = 0, mouseY = 0;
static float prevRealMouseX = 0, prevRealMouseY = 0;
static int debounce = 0;
static int clipx = 0;
static int clipy = 0;
static int clipw = 0;
static int cliph = 0;
static bool endFrame = true;
static int screenOffsetX = 0;
static int screenOffsetY = 0;

extern "C" char* sbrk(int incr);
uint16_t getFreeRam() {	
	char top;
	return &top - reinterpret_cast<char*>(sbrk(0));
}


uint8_t readButtons()
{
  uint8_t ret = 0;
  if (!digitalRead(BUTTON_1_PIN))
    ret |= BUTTON_1_MASK;
  if (!digitalRead(BUTTON_2_PIN))
    ret |= BUTTON_2_MASK;
  if (!digitalRead(BUTTON_3_PIN))
    ret |= BUTTON_3_MASK;
  return ret;
}


inline void setClipRect(int16_t x, int16_t y, int16_t w, int16_t h)
{
    clipx = x < 0 ? 0 : x;
    clipy = y < 0 ? 0 : y;
    
    int16_t right = x + w;
    int16_t bottom = y + h;
    
    right = right > WINDOW_WIDTH ? WINDOW_WIDTH : right;
    bottom = bottom > WINDOW_HEIGHT ? WINDOW_HEIGHT : bottom;
    
    clipw = right - clipx;   // Width from clipped left to clipped right
    cliph = bottom - clipy;  // Height from clipped top to clipped bottom
    
    // Ensure non-negative dimensions
    if (clipw < 0) clipw = 0;
    if (cliph < 0) cliph = 0;
}

inline void memset16(uint16_t *dest, uint16_t value, size_t num_words) {
    size_t i;
    for (i = 0; i < num_words; i++) {
        dest[i] = value;
    }
}

inline void fillRectBuffer(uint16_t *buffer, uint16_t stride, int16_t clipx, int16_t clipy, int16_t clipw, int16_t cliph, float posx, float posy, float w, float h, uint16_t col)
{
    // Fast early exits
    if (w <= 0 || h <= 0 || !buffer) return;
    
    const int16_t clip_right = clipx + clipw;
    const int16_t clip_bottom = clipy + cliph;
    const int16_t rect_right = posx + w;
    const int16_t rect_bottom = posy + h;
    
    const int16_t minx = posx > clipx ? posx : clipx;
    const int16_t miny = posy > clipy ? posy : clipy;
    const int16_t maxx = rect_right < clip_right ? rect_right : clip_right;
    const int16_t maxy = rect_bottom < clip_bottom ? rect_bottom : clip_bottom;
    
    const int16_t fill_width = maxx - minx;
    const int16_t fill_height = maxy - miny;
    if (fill_width <= 0 || fill_height <= 0) return;
    
    const uint32_t col32 = ((uint32_t)col << 16) | col;
    
    // for (int y = miny; y< maxy; y++)
    //     memset16(&buffer[y*stride + minx], col, fill_width);
    // return;

    // Single scanline optimization - use memset if beneficial
    if (fill_height == 1) {
        uint16_t* ptr = &buffer[miny * stride + minx];
        
        // For small fills, direct writes are faster than memset overhead
        if (fill_width <= 4) {
            for (int16_t i = 0; i < fill_width; i++) {
                *ptr++ = col;
            }
            return;
        }
        
        // Handle alignment
        if ((uintptr_t)ptr & 2) {
            *ptr++ = col;
            int16_t remaining = fill_width - 1;
            
            // Write 32-bit pairs
            uint32_t* ptr32 = (uint32_t*)ptr;
            while (remaining >= 2) {
                *ptr32++ = col32;
                remaining -= 2;
            }
            if (remaining) {
                *((uint16_t*)ptr32) = col;
            }
        } else {
            // Aligned path
            uint32_t* ptr32 = (uint32_t*)ptr;
            int16_t pairs = fill_width >> 1;
            
            while (pairs >= 4) {
                *ptr32++ = col32; *ptr32++ = col32; *ptr32++ = col32; *ptr32++ = col32;
                pairs -= 4;
            }
            while (pairs--) {
                *ptr32++ = col32;
            }
            if (fill_width & 1) {
                *((uint16_t*)ptr32) = col;
            }
        }
        return;
    }
    
    // Multi-scanline: Optimize for common case (aligned, even width)
    if (!(minx & 1) && !(fill_width & 1)) {
        // Fully aligned fast path
        for (int16_t y = miny; y < maxy; y++) {
            uint32_t* ptr32 = (uint32_t*)&buffer[y * stride + minx];
            int16_t pairs = fill_width >> 1;
            
            while (pairs >= 4) {
                *ptr32++ = col32; *ptr32++ = col32; *ptr32++ = col32; *ptr32++ = col32;
                pairs -= 4;
            }
            while (pairs--) {
                *ptr32++ = col32;
            }
        }
        return;
    }
    
    // General case (unaligned or odd width)
    for (int16_t y = miny; y < maxy; y++) {
        uint16_t* ptr = &buffer[y * stride + minx]; 
        int16_t remaining = fill_width;
        
        // Handle alignment
        if ((uintptr_t)ptr & 2) {
            *ptr++ = col;
            remaining--;
        }
        
        // Write pairs of pixels
        uint32_t* ptr32 = (uint32_t*)ptr;
        while (remaining >= 8) {
            *ptr32++ = col32; *ptr32++ = col32; *ptr32++ = col32; *ptr32++ = col32;
            remaining -= 8;
        }
        while (remaining >= 2) {
            *ptr32++ = col32;
            remaining -= 2;
        }
        
        if (remaining) {
            *((uint16_t*)ptr32) = col;
        }
    }
}

static void loadHighScores()
{
    //read_save(saveData);
}

static void saveHighScores()
{
//    write_save(saveData);
}

void md_playTone(float freq, float duration, float when)
{ 
    float now = (millis() - getAudioStartTime()) / 1000.0;
    float delay_sec = when - now;
    
    if (delay_sec < 0) delay_sec = 0;  // Play immediately if in the past
    
    playTone(freq, 255/10 * vol, duration, delay_sec); 
}

void md_stopTone()
{
    stopAllTones();
}

float md_getAudioTime()
{
    return (millis() - getAudioStartTime()) / 1000.0;
}
    
void md_drawCharacter(unsigned char grid[CHARACTER_HEIGHT][CHARACTER_WIDTH][3],
                     float x, float y, int hash) 
{
    unsigned char r,g,b;
    for (int yy = 0; yy < CHARACTER_HEIGHT; yy++) {
        for (int xx = 0; xx < CHARACTER_WIDTH; xx++) {
            r = grid[yy][xx][0];
            g = grid[yy][xx][1];
            b = grid[yy][xx][2];
            
            if ((r == 0) && (g == 0) && (b == 0)) continue;

            fillRectBuffer(fb, WINDOW_WIDTH,clipx,clipy,clipw,cliph,
                (offsetX + x*scale + (float)xx * scale),
                (offsetY + y*scale + (float)yy * scale),
                ceilf(scale),
                ceilf(scale),
                tft.color565(r, g, b)
            );
        }
    }
}

 void md_drawRect(float x, float y, float w, float h, unsigned char r,
                  unsigned char g, unsigned char b) {
    //adjust for different behaviour between sdl and js in case of negative width / height
    if(w < 0.0f) {
        x += w;
        w *= -1.0f;
    }
    if(h < 0.0f) {
        y += h;
        h *= -1.0f;
    }

    fillRectBuffer(fb, WINDOW_WIDTH,clipx, clipy, clipw, cliph, 
        (int16_t)(offsetX + x * scale), 
        (int16_t)(offsetY + y * scale),
        (int16_t)ceilf(w * scale),
        (int16_t)ceilf(h * scale),
        tft.color565(r, g, b)
    );
}

void md_clearView(unsigned char r, unsigned char g, unsigned char b) 
{
    setClipRect(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
    fillRectBuffer(fb, WINDOW_WIDTH,clipx, clipy, clipw, cliph, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, tft.color565(clearColorR,clearColorG,clearColorB));
    setClipRect(offsetX, offsetY, viewW, viewH);

    fillRectBuffer(fb, WINDOW_WIDTH,clipx, clipy, clipw, cliph, 
        offsetX,
        offsetY,
        viewW,
        viewH,
        tft.color565(r, g, b)
    );
}

void md_clearScreen(unsigned char r, unsigned char g, unsigned char b)
{
    clearColorR = r;
    clearColorG = g;
    clearColorB = b;
    //window width & height can be smaller than actual screen width & height if hardcoded
    setClipRect(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);
    fillRectBuffer(fb, WINDOW_WIDTH,clipx, clipy, clipw, cliph, 0, 0, WINDOW_WIDTH, WINDOW_HEIGHT, tft.color565(r,g,b));
    setClipRect(offsetX, offsetY, viewW, viewH);
}

void md_initView(int w, int h) 
{   

    WINDOW_WIDTH = tft.width();
    WINDOW_HEIGHT = tft.height();
    screenOffsetX = (tft.width() -WINDOW_WIDTH) >> 1;
    screenOffsetY = (tft.height() -WINDOW_HEIGHT) >> 1;
    
    float wscalex = (float)WINDOW_WIDTH / (float)tft.width();
    float wscaley = (float)WINDOW_HEIGHT / (float)tft.height();
    wscale = (wscaley < wscalex) ? wscaley : wscalex;

    origViewW = w;
    origViewH = h;
    float xScale = (float)WINDOW_WIDTH / w;
    float yScale = (float)WINDOW_HEIGHT / h;
    if (yScale < xScale)
        scale = yScale;
    else
        scale = xScale;
    viewW = (int)floorf((float)w * scale);
    viewH = (int)floorf((float)h * scale);
    offsetX = (int)(WINDOW_WIDTH - viewW) >> 1;
    offsetY = (int)(WINDOW_HEIGHT - viewH) >> 1;
    mouseX = viewW >> 1;
    mouseY = viewH >> 1; 
    setMouseRange(0,0,WINDOW_WIDTH, WINDOW_HEIGHT);  
    setMousePos(offsetX + mouseX, offsetY + mouseY);
    setClipRect(offsetX, offsetY, viewW, viewH);


}

void md_consoleLog(char* msg) 
{ 
    const char* tmp = msg;
    Serial.println(tmp); 
}

// Draw a single pixel
inline void bufferDrawPixel(uint16_t* fb, int16_t stride, int16_t x, int16_t y, uint16_t color) {
    if (!fb) return;
    if(x >= 0 && x < WINDOW_WIDTH && y >= 0 && y < WINDOW_HEIGHT)
        *(fb + y * stride + x) = color;
}

// Draw a character
void bufferDrawChar(uint16_t* fb, int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size, const uint8_t* font) {
    if (!fb || !font) return;
    
    if (c >= 176) c++;
    
    for (int8_t i = 0; i < 5; i++) {
        uint8_t line = pgm_read_byte(&font[c * 5 + i]);
        
        for (int8_t j = 0; j < 8; j++) {
            if (line & 0x1) {
                if (size == 1) {
                    bufferDrawPixel(fb, WINDOW_WIDTH, x + i, y + j, color);
                } else {
                    fillRectBuffer(fb,WINDOW_WIDTH, 0,0,WINDOW_WIDTH,WINDOW_HEIGHT, x + i * size, y + j * size, size, size, color);
                }
            } else if (bg != color) {
                if (size == 1) {
                    bufferDrawPixel(fb, WINDOW_WIDTH, x + i, y + j, bg);
                } else {
                    fillRectBuffer(fb,WINDOW_WIDTH, 0,0,WINDOW_WIDTH,WINDOW_HEIGHT, x + i * size, y + j * size, size, size, bg);
                }
            }
            line >>= 1;
        }
    }
    
    if (bg != color) {
        if (size == 1) {
            for (int8_t j = 0; j < 8; j++) {
                bufferDrawPixel(fb, WINDOW_WIDTH, x + 5, y + j, bg);
            }
        } else {
            fillRectBuffer(fb, WINDOW_WIDTH,  0,0,WINDOW_WIDTH,WINDOW_HEIGHT, x + 5 * size, y, size, 8 * size, bg);
        }
    }
}

// Print a string
void bufferPrint(uint16_t* fb, int16_t x, int16_t y, const char* str, uint16_t color, uint16_t bg, uint8_t size, const uint8_t* font) {
    if (!fb || !str || !font) return;
    
    int16_t cursorX = x;
    
    while (*str) {
        bufferDrawChar(fb, cursorX, y, *str, color, bg, size, font);
        cursorX += (6 * size);
        str++;
    }
}

void printDebugCpuRamLoad()    
{
static int lastFPS = 0;
    static uint32_t lastPrint = 0;
    if(debugMode)
    {
        int currentFPS = (int)frameRate;
        char debuginfo[80];
        
        int fps_int = (int)frameRate;
        int fps_frac = (int)((frameRate - fps_int) * 100);

        sprintf(debuginfo, "F:%3d R:%3d A:%d B:%d%% O:%d U:%d", 
            fps_int, getFreeRam(), 
            getActiveChannelCount(), 
            (getBufferAvailable()*100)/getActualBufferSize(),
            getBufferSkipCount(),
            getBufferUnderrunCount());
        //Serial.println(debuginfo); 
        bufferPrint(fb, 0, 0, debuginfo, tft.color565(255,255,255), tft.color565(0,0,0), 1, font);
        lastFPS = currentFPS;
    } 
}

static void setupBoardAudio()
{
    if (!setupI2SAudio(SAMPLERATE, AUDIO_OUT_BOTH, AUDIO_BUFFER_SIZE)) {
        Serial.println("Failed to initialize I2S audio!");
        while (1) delay(100);
    }
    
    Serial.println("I2S Audio initialized successfully!");
}

void setupButtons()
{
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
}

void setup()
{

#ifdef PIN_5V_EN
    pinMode(PIN_5V_EN, OUTPUT);
    digitalWrite(PIN_5V_EN, PIN_5V_EN_STATE);
#endif

#ifdef PIN_USB_HOST_VBUS
    //not sure if required, doom port did this as well
    Serial.printf("Enabling USB host VBUS power on GP%d\r\n", PIN_USB_HOST_VBUS);
    gpio_init(PIN_USB_HOST_VBUS);
    gpio_set_dir(PIN_USB_HOST_VBUS, GPIO_OUT);
    gpio_put(PIN_USB_HOST_VBUS, 1);
#endif

    //same doom port did this as well i guess this fixes usb host stability
    irq_set_priority(USBCTRL_IRQ, 0xc0);
    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    pio_cfg.pin_dp = PIN_USB_HOST_DP;
    pio_cfg.tx_ch = 9; //added this otherwise would not work in combination with display saw the doom port setting this also
    USBHost.configure_pio_usb(1, &pio_cfg); 
    if(!USBHost.begin(1))
    {
        pinMode(LED_BUILTIN, OUTPUT);
        for (;;)
        digitalWrite(LED_BUILTIN, (millis() / 500) & 1);
    }
    delay(4000); //needs to be high enough or also does not work
}

void loop()
{
   USBHost.task();
   delayMicroseconds(100);
}

void setup1()
{   
    Serial.begin(9600);
    Serial.println("Crisp Game Lib Portable");

    if (!tft.begin()) { // Blink LED if insufficient RAM
        Serial.printf("failed to setup display\n");
        pinMode(LED_BUILTIN, OUTPUT);
        for (;;)
        digitalWrite(LED_BUILTIN, (millis() / 500) & 1);
    }

    setupButtons();
    setupBoardAudio();
    //when double buffer getbuffer returns backbuffer
    fb = tft.getBuffer();

    //no way to save highscores in this way
    //onSaveData = saveHighScores;

    initGame();
    loadHighScores();
    currentTime = micros();
    lastTime = 0; 
}

void loop1()
{
    updateI2SAudio();
    
    currentTime = micros();
    frameTime  = currentTime - lastTime;  
    if((frameTime < timePerFrame) || !endFrame)
       return;     
    endFrame = false;
    frameRate = 1000000.0 / frameTime;
    lastTime = currentTime;
    bool mouseUsed = getGame(currentGameIndex).usesMouse;
    uint8_t pressed_buttons = readButtons();
    
    setButtonState(
        !mouseUsed && (keyPressed(LEFTKEY) || gamepadButtonPressed(GAMEPAD_LEFT)), 
        !mouseUsed && (keyPressed(RIGHTKEY) || gamepadButtonPressed(GAMEPAD_RIGHT)), 
        !mouseUsed && (keyPressed(UPKEY) || gamepadButtonPressed(GAMEPAD_UP)), 
        !mouseUsed && (keyPressed(DOWNKEY) || gamepadButtonPressed(GAMEPAD_DOWN)), 
        (pressed_buttons & BUTTON_1_MASK) || keyPressed(BUTTON1_KEY) || gamepadButtonPressed(GAMEPAD_B) || mouseButtonPressed(1), 
        (pressed_buttons & BUTTON_3_MASK) || keyPressed(BUTTON3_KEY) || gamepadButtonPressed(GAMEPAD_A) || keyPressed(BUTTON3_ALTERNATE)|| mouseButtonPressed(0) 
    ); 

    if (mouseUsed)
    {
        if((pressed_buttons & BUTTON_3_MASK) || keyPressed(RIGHTKEY) || gamepadButtonPressed(GAMEPAD_RIGHT))
            mouseX += WINDOW_WIDTH /100.0f;
        
        if((pressed_buttons & BUTTON_1_MASK) || keyPressed(LEFTKEY)|| gamepadButtonPressed(GAMEPAD_LEFT))
            mouseX -= WINDOW_WIDTH /100.0f;
            
        if(keyPressed(UPKEY) || gamepadButtonPressed(GAMEPAD_UP))
            mouseY -= WINDOW_HEIGHT /100.0f;
    
        if(keyPressed(DOWNKEY) || gamepadButtonPressed(GAMEPAD_DOWN))
            mouseY += WINDOW_HEIGHT /100.0f;

        mouseX = clamp(mouseX, 0, WINDOW_WIDTH - 2*offsetX -1);
        mouseY = clamp(mouseY, 0, WINDOW_HEIGHT - 2*offsetY -1);

        if((prevRealMouseX != getMouseX()) || (prevRealMouseY != getMouseY()))
        {
            mouseX = getMouseX() - offsetX;
            mouseY = getMouseY() - offsetY;
            prevRealMouseX = getMouseX();
            prevRealMouseY = getMouseY();
        }

        setMousePos(mouseX / scale, mouseY / scale);
    }

    updateFrame();
    

    if(mouseUsed && !isInGameOver)
    {
        uint16_t col = tft.color565(255, 105, 180);
        fillRectBuffer(fb, WINDOW_WIDTH,clipx, clipy, clipw, cliph, offsetX + (mouseX-3*wscale), offsetY + (mouseY-1*wscale), (7.0f*wscale),(3.0f*wscale),col);
        fillRectBuffer(fb, WINDOW_WIDTH,clipx, clipy, clipw, cliph, offsetX + (mouseX-1*wscale), offsetY + (mouseY-3*wscale), (3.0f*wscale),(7.0f*wscale),col);
    }

    if(!isInMenu)
        if((pressed_buttons & BUTTON_2_MASK) || keyPressed(BUTTON2_KEY) || mouseButtonPressed(1) || gamepadButtonPressed(GAMEPAD_B))
            goToMenu();
    
    if(debounce < millis())
    {
        if(keyPressed(F1KEY) || gamepadButtonPressed(GAMEPAD_LEFT_SHOULDER))
        {
            vol--;
            if(vol < 0)
                vol = 10;
            debounce = millis() + 200;
        }

        if(keyPressed(F2KEY) || gamepadButtonPressed(GAMEPAD_RIGHT_SHOULDER))
        {
            vol++;
            if(vol > 10)
                vol = 0;
            debounce = millis() + 200;
        }

        if(keyPressed(DKEY) || gamepadButtonPressed(GAMEPAD_SELECT))
        {
            debugMode = !debugMode;
            debounce = millis() + 200;
        }
    }

    printDebugCpuRamLoad();
    tft.swap();
    //need to grab the new buffer
    fb = tft.getBuffer();
    endFrame = true;
}
