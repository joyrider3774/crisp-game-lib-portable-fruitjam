#include "Adafruit_TinyUSB.h"
#include "tusb.h"
#include "usbh_processor.h"

#undef DEBUG_USB

#define REPORT_NONE 0xFF

typedef struct 
{
    uint16_t productid;
    uint16_t vendorid;
    uint8_t buttonLeftReport;
    uint8_t buttonLeftPressedValue;
    bool    buttonLeftIsMask;
    uint8_t buttonRightReport;
    uint8_t buttonRightPressedValue;
    bool    buttonRightIsMask;
    uint8_t buttonUpReport;
    uint8_t buttonUpPressedValue;
    bool    buttonUpIsMask;
    uint8_t buttonDownReport;
    uint8_t buttonDownPressedValue;
    bool    buttonDownIsMask;
    uint8_t buttonAReport;
    uint8_t buttonAPressedValue;
    bool    buttonAIsMask;
    uint8_t buttonBReport;
    uint8_t buttonBPressedValue;
    bool    buttonBIsMask;
    uint8_t buttonXReport;
    uint8_t buttonXPressedValue;
    bool    buttonXIsMask;
    uint8_t buttonYReport;
    uint8_t buttonYPressedValue;
    bool    buttonYIsMask;
    uint8_t buttonLeftShoulderReport;
    uint8_t buttonLeftShoulderPressedValue;
    bool    buttonLeftShoulderIsMask;
    uint8_t buttonRightShoulderReport;
    uint8_t buttonRightShoulderPressedValue;
    bool    buttonRightShoulderIsMask;
    uint8_t buttonSelectReport;
    uint8_t buttonSelectPressedValue;
    bool    buttonSelectIsMask;
    uint8_t buttonStartReport;
    uint8_t buttonStartPressedValue;
    bool    buttonStartIsMask;
} GamePadReport;

GamePadReport GamePadConfigs[] = {
    //{productid, vendorid, L,LV,LM,R,RV,RM,U,UV,UM,D,DV,DM,A,AV,AM,B,BV,BM,X,XV,XM,Y,YV,YM,LS,LSV,LSM,RS,RSV,RSM,SELECT,SELECTV,SELECTM,START,STARTV,STARTM}
    //snes padd from adafruit (tested thanks Lord Rybec)
    {58369,2079,0,0x00,false,0,0xFF,false,1,0x00,false,1,0xFF,false,5,0x2F,true,5,0x4F,true, 5,0x1F,true, 5,0x8F,true,6,0x01,true,6,0x02,true,6,0x10,true,6,0x20,true},
    //ps1 none dualshock using my adaptor
    {34918,2341,2,0x00,false,2,0xFF,false,3,0x00,false,3,0xFF,false,0,0x04,true,0,0x02,true,0,0x08,true,0,0x01,true,0,0x40,true,0,0x80,true,1,0xf2,false,1,0xf1,false},
};

volatile bool keyboardKeys[0xFF];
volatile uint8_t mouseButtons;
volatile int16_t mousex = 0;
volatile int16_t mousey = 0;
volatile int16_t mouseRangeMinX = 0;
volatile int16_t mouseRangeMinY = 0;
volatile int16_t mouseRangeMaxX = 0;
volatile int16_t mouseRangeMaxY = 0;
volatile uint32_t joystickButtons = 0;

onKeyboardKeyDownUpCallback keyboardUpDownCallback = NULL;

void setKeyDownUpCallBack(onKeyboardKeyDownUpCallback callback)
{
    keyboardUpDownCallback = callback;
}

void setMouseRange(int16_t minx, int16_t miny, int16_t w, int16_t h)
{
    mouseRangeMinX = minx;
    mouseRangeMinX = miny;
    mouseRangeMaxX = minx + w;
    mouseRangeMaxY = miny + h;
}

void setMouse(int16_t x, int16_t y)
{
    mousex = x;
    mousey = y;
    if (mousex < mouseRangeMinX) 
        mousex = mouseRangeMinX;
    if (mousex > mouseRangeMaxX) 
        mousex = mouseRangeMaxX;
    if (mousey < mouseRangeMinY) 
        mousey = mouseRangeMinY;
    if (mousey > mouseRangeMaxY) 
        mousey = mouseRangeMaxY;
}

int16_t getMouseX()
{
    return mousex;
}

int16_t getMouseY()
{
    return mousey;
}

bool gamepadButtonPressed(uint32_t button)
{
    return joystickButtons & button;
}


bool mouseButtonPressed(uint8_t button)
{
    return mouseButtons & (1 << button);
}

bool keyPressed(uint8_t key)
{
    return keyboardKeys[key];
}

void USBHidUpdate(Adafruit_USBH_Host *host) 
{
    host->task();
}

// Helper function to print key names

const char* getKeyName(uint8_t key) 
{
  // This is a simplified list. Full HID keyboard has many more key codes
  switch (key) {
    case 0x04: return "A"; break;
    case 0x05: return "B"; break;    
    case 0x06: return "C"; break;
    case 0x07: return "D"; break;
    case 0x08: return "E"; break;
    case 0x09: return "F"; break;
    case 0x0A: return "G"; break;
    case 0x0B: return "H"; break;
    case 0x0C: return "I"; break;
    case 0x0D: return "J"; break;
    case 0x0E: return "K"; break;
    case 0x0F: return "L"; break;
    case 0x10: return "M"; break;
    case 0x11: return "N"; break;
    case 0x12: return "O"; break;
    case 0x13: return "P"; break;
    case 0x14: return "Q"; break;
    case 0x15: return "R"; break;
    case 0x16: return "S"; break;
    case 0x17: return "T"; break;
    case 0x18: return "U"; break;
    case 0x19: return "V"; break;
    case 0x1A: return "W"; break;
    case 0x1B: return "X"; break;
    case 0x1C: return "Y"; break;
    case 0x1D: return "Z"; break;
    case 0x1E: return "1"; break;
    case 0x1F: return "2"; break;
    case 0x20: return "3"; break;
    case 0x21: return "4"; break;
    case 0x22: return "5"; break;
    case 0x23: return "6"; break;
    case 0x24: return "7"; break;
    case 0x25: return "8"; break;
    case 0x26: return "9"; break;
    case 0x27: return "0"; break;
    case 0x28: return "ENTER"; break;
    case 0x29: return "ESC"; break;
    case 0x2A: return "BACKSPACE"; break;
    case 0x2B: return "TAB"; break;
    case 0x2C: return "SPACE"; break;
    case 0x2D: return "MINUS"; break;
    case 0x2E: return "EQUAL"; break;
    case 0x2F: return "LBRACKET"; break;
    case 0x30: return "RBRACKET"; break;
    case 0x31: return "BACKSLASH"; break;
    case 0x33: return "SEMICOLON"; break;
    case 0x34: return "QUOTE"; break;
    case 0x35: return "GRAVE"; break;
    case 0x36: return "COMMA"; break;
    case 0x37: return "PERIOD"; break;
    case 0x38: return "SLASH"; break;
    case 0x39: return "CAPS_LOCK"; break;
    case 0x4F: return "RIGHT_ARROW"; break;
    case 0x50: return "LEFT_ARROW"; break;
    case 0x51: return "DOWN_ARROW"; break;
    case 0x52: return "UP_ARROW"; break;
    // default:
    //   if (key >= 0x3A && key <= 0x45) { // F1-F12
    //     return "F";
    //     Serial.print(key - 0x3A + 1);
    //   } else {
    //     // For keys not handled above, just print the HID code
    //     return "0x";
    //     Serial.print(key, HEX);
    //   }
    //   break;
  }
  return "UNKNOWN KEY";
}

//----------------------------------------------------------------------------------------------------------

//This code below is based on the code found in the doom port here https://github.com/adafruit/fruitjam-doom

//--------------------------------------------------------------------+
// HID Host Callback Functions
//--------------------------------------------------------------------+

#define MAX_REPORT  4

#ifdef DEBUG_USB
#define debug_printf Serial.printf
#else  
#define debug_printf(fmt,...) ((void)0)
#endif

// Each HID instance can has multiple reports
static struct
{
    uint16_t productId;
    uint16_t vendorId;
    uint8_t report_count;
    tuh_hid_report_info_t report_info[MAX_REPORT];
}hid_info[CFG_TUH_HID];

static void process_kbd_report(hid_keyboard_report_t const *report);
static void process_mouse_report(hid_mouse_report_t const * report);
static void process_joystick_report(size_t len, const uint8_t *report, uint16_t productid, uint16_t vendorid);
static void process_generic_report(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len);

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* desc_report, uint16_t desc_len)
{
    debug_printf("HID device address = %d, instance = %d is mounted\r\n", dev_addr, instance);

    // Interface protocol (hid_interface_protocol_enum_t)
    const char* protocol_str[] = { "None", "Keyboard", "Mouse" };
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);
    debug_printf("HID Interface Protocol = %s\r\n", protocol_str[itf_protocol]);

    // By default host stack will use activate boot protocol on supported interface.
    // Therefore for this simple example, we only need to parse generic report descriptor (with built-in parser)
    if ( itf_protocol == HID_ITF_PROTOCOL_NONE )
    {
        tusb_desc_device_t desc_device;
        if(tuh_descriptor_get_device(dev_addr, &desc_device, sizeof(tusb_desc_device_t), NULL, 0))
        {
            hid_info[instance].productId = desc_device.idProduct;
            hid_info[instance].vendorId = desc_device.idVendor;
        }
        hid_info[instance].report_count = tuh_hid_parse_report_descriptor(hid_info[instance].report_info, MAX_REPORT, desc_report, desc_len);
        debug_printf("HID has %u reports \r\n", hid_info[instance].report_count);
    }

    // request to receive report
    // tuh_hid_report_received_cb() will be invoked when report is available
    if ( !tuh_hid_receive_report(dev_addr, instance) )
    {
        debug_printf("Error: cannot request to receive report\r\n");
    }
}

// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    debug_printf("HID device address = %d, instance = %d is unmounted\r\n", dev_addr, instance);
}

// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
    uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, instance);

    switch (itf_protocol)
    {
        case HID_ITF_PROTOCOL_KEYBOARD:
            process_kbd_report( (hid_keyboard_report_t const*) report );
            break;

        case HID_ITF_PROTOCOL_MOUSE:
            process_mouse_report( (hid_mouse_report_t const*) report );
            break;

        default:
            // Generic report requires matching ReportID and contents with previous parsed report info
            process_generic_report(dev_addr, instance, report, len);
            break;
    }

    // continue to request to receive report
    if ( !tuh_hid_receive_report(dev_addr, instance) )
    {
        debug_printf("Error: cannot request to receive report\r\n");
    }
}

//--------------------------------------------------------------------+
// Keyboard
//--------------------------------------------------------------------+

// look up new key in previous keys
static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
    for(uint8_t i=0; i<6; i++)
    {
        if (report->keycode[i] == keycode)  return true;
    }

    return false;
}

static void check_mod(int mod, int prev_mod, int mask, int scancode) {
    if ((mod^prev_mod)&mask) {
        if (mod & mask)
        {
            debug_printf("key down: %s\n", getKeyName(scancode));
            if((scancode < 0xFF) && (scancode > 0))
            {
                if(keyboardUpDownCallback)
                    keyboardUpDownCallback(scancode, true);
                keyboardKeys[scancode] = true;
            }
        }
        else
        {
            debug_printf("key up: %s\n", getKeyName(scancode));
            if((scancode < 0xFF) && (scancode > 0))
            {
                if(keyboardUpDownCallback)
                    keyboardUpDownCallback(scancode, false);
                keyboardKeys[scancode] = false;
            }
        }
    }
}

static void process_kbd_report(hid_keyboard_report_t const *report)
{
    static hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released

    //------------- example code ignore control (non-printable) key affects -------------//
    for(uint8_t i=0; i<6; i++)
    {
        if ( report->keycode[i] )
        {
            if ( find_key_in_report(&prev_report, report->keycode[i]) )
            {
                // exist in previous report means the current key is holding
            }
            else
            {
                // not existed in previous report means the current key is pressed                
                debug_printf("key down: %s\n", getKeyName(report->keycode[i]));
                if((report->keycode[i] < 0xFF) && (report->keycode[i] > 0))
                {
                    if(keyboardUpDownCallback)
                        keyboardUpDownCallback(report->keycode[i], true);
                    keyboardKeys[report->keycode[i]] = true;
                }
            }
        }
        // Check for key depresses (i.e. was present in prev report but not here)
        if (prev_report.keycode[i]) {
            // If not present in the current report then depressed
            if (!find_key_in_report(report, prev_report.keycode[i]))
            {                
                debug_printf("key up: %s\n", getKeyName(prev_report.keycode[i]));
                if((prev_report.keycode[i] < 0xFF) && (prev_report.keycode[i] > 0))
                {
                    if(keyboardUpDownCallback)
                        keyboardUpDownCallback(prev_report.keycode[i], false);
                    keyboardKeys[prev_report.keycode[i]] = false;
                }
            }
        }
    }

    prev_report = *report;
}

//--------------------------------------------------------------------+
// Mouse
//--------------------------------------------------------------------+


static void process_mouse_report(hid_mouse_report_t const * report)
{
    debug_printf("Mouse report buttons: %d, x:%d y:%d\n", report->buttons, report->x, report->y);
    if((report->buttons >= 0) && (report->buttons < 0xFF))
        mouseButtons = report->buttons;
    mousex += report->x;
    mousey += report->y;
    if (mousex < mouseRangeMinX) 
        mousex = mouseRangeMinX;
    if (mousex > mouseRangeMaxX) 
        mousex = mouseRangeMaxX;
    if (mousey < mouseRangeMinY) 
        mousey = mouseRangeMinY;
    if (mousey > mouseRangeMaxY) 
        mousey = mouseRangeMaxY;
}

//--------------------------------------------------------------------+
// Joystick / Joypad
//--------------------------------------------------------------------+

inline uint32_t processButton(uint8_t buttonReport, uint8_t buttonValue, bool buttonIsMask, const uint8_t *report, uint32_t buttonReturnValue) 
{
    if(buttonReport == REPORT_NONE)
        return 0;
    if (buttonIsMask)
        return (report[buttonReport] & buttonValue) == buttonValue ? buttonReturnValue: 0;
    else
        return report[buttonReport] == buttonValue ? buttonReturnValue: 0;

}

static void process_joystic_gamepadconfig(uint8_t configindex, const uint8_t *report)
{
    uint32_t val = 0;
    GamePadReport * c = &GamePadConfigs[configindex];
    val |= processButton(c->buttonAReport, c->buttonAPressedValue, c->buttonAIsMask, report, GAMEPAD_A);
    val |= processButton(c->buttonBReport, c->buttonBPressedValue, c->buttonBIsMask, report, GAMEPAD_B);
    val |= processButton(c->buttonXReport, c->buttonXPressedValue, c->buttonXIsMask, report, GAMEPAD_X);
    val |= processButton(c->buttonYReport, c->buttonYPressedValue, c->buttonYIsMask, report, GAMEPAD_Y);
    val |= processButton(c->buttonLeftShoulderReport, c->buttonLeftShoulderPressedValue, c->buttonLeftShoulderIsMask, report, GAMEPAD_LEFT_SHOULDER);
    val |= processButton(c->buttonRightShoulderReport, c->buttonRightShoulderPressedValue, c->buttonRightShoulderIsMask, report, GAMEPAD_RIGHT_SHOULDER);
    val |= processButton(c->buttonSelectReport, c->buttonSelectPressedValue, c->buttonSelectIsMask, report, GAMEPAD_SELECT);
    val |= processButton(c->buttonStartReport, c->buttonStartPressedValue, c->buttonStartIsMask, report, GAMEPAD_START);
    val |= processButton(c->buttonUpReport, c->buttonUpPressedValue, c->buttonUpIsMask, report, GAMEPAD_UP);
    val |= processButton(c->buttonDownReport, c->buttonDownPressedValue, c->buttonDownIsMask, report, GAMEPAD_DOWN);
    val |= processButton(c->buttonLeftReport, c->buttonLeftPressedValue, c->buttonLeftIsMask, report, GAMEPAD_LEFT);
    val |= processButton(c->buttonRightReport, c->buttonRightPressedValue, c->buttonRightIsMask, report, GAMEPAD_RIGHT);
    joystickButtons = val;
}

static void process_joystick_report(size_t len, const uint8_t *report, uint16_t productId, uint16_t vendorId)
{
    //debug print report data
    debug_printf("joyrstick report productid %d vendorid %d: ", productId, vendorId);
    for (int i = 0; i < len; i++) 
    {
      debug_printf("%#02x",report[i]);
      debug_printf(" ");
          
    }
    debug_printf("\n");

    for (int i = 0; i < sizeof(GamePadConfigs) / sizeof(GamePadConfigs[0]); i++)
    {
        if( (GamePadConfigs[i].vendorid == vendorId) && (GamePadConfigs[i].productid == productId))
        {
            process_joystic_gamepadconfig(i, report);
            return;
        }
    }
    //default config
    process_joystic_gamepadconfig(0, report);
}

//--------------------------------------------------------------------+
// Generic Report
//--------------------------------------------------------------------+
static void process_generic_report(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
    (void) dev_addr;

    uint8_t const rpt_count = hid_info[instance].report_count;
    tuh_hid_report_info_t* rpt_info_arr = hid_info[instance].report_info;
    tuh_hid_report_info_t* rpt_info = NULL;
 
    if ( rpt_count == 1 && rpt_info_arr[0].report_id == 0)
    {
        // Simple report without report ID as 1st byte
        rpt_info = &rpt_info_arr[0];
    }else
    {
        // Composite report, 1st byte is report ID, data starts from 2nd byte
        uint8_t const rpt_id = report[0];

        // Find report id in the arrray
        for(uint8_t i=0; i<rpt_count; i++)
        {
            if (rpt_id == rpt_info_arr[i].report_id )
            {
                rpt_info = &rpt_info_arr[i];
                break;
            }
        }

        report++;
        len--;
    }

    if (!rpt_info)
    {
        debug_printf("Couldn't find the report info for this report !\r\n");
        return;
    }

    // For complete list of Usage Page & Usage checkout src/class/hid/hid.h. For examples:
    // - Keyboard                     : Desktop, Keyboard
    // - Mouse                        : Desktop, Mouse
    // - Gamepad                      : Desktop, Gamepad
    // - Consumer Control (Media Key) : Consumer, Consumer Control
    // - System Control (Power key)   : Desktop, System Control
    // - Generic (vendor)             : 0xFFxx, xx
    if ( rpt_info->usage_page == HID_USAGE_PAGE_DESKTOP )
    {
        switch (rpt_info->usage)
        {
            case HID_USAGE_DESKTOP_KEYBOARD:
                // Assume keyboard follow boot report layout
                process_kbd_report( (hid_keyboard_report_t const*) report );
                break;


            case HID_USAGE_DESKTOP_MOUSE:
                // Assume mouse follow boot report layout
                process_mouse_report( (hid_mouse_report_t const*) report );
                break;

            case HID_USAGE_DESKTOP_GAMEPAD:
            case HID_USAGE_DESKTOP_JOYSTICK:
                process_joystick_report( (size_t) len, report, hid_info[instance].productId, hid_info[instance].vendorId );
                break;
            
            

            default:
                debug_printf("HID receive report usage=%d\r\n", rpt_info->usage);
                break;
        }
    }
}
