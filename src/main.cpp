#include <Arduino.h>
#include "lvgl.h"
#include "TouchDrvCHSC5816.hpp"
#include "ui\ui.h"
#include "custom.h"
#include "wifi_mqtt.h"
#include <math.h>
#include <Adafruit_SleepyDog.h>
#include <elapsedMillis.h>

#define t_max 480
#define knob_increment 60

enum KNOB_State
{
    KNOB_NULL,
    KNOB_INCREMENT,
    KNOB_DECREMENT,
};

int32_t KNOB_Data = 0;
bool KNOB_Trigger_Flag = false;
uint8_t KNOB_State_Flag = KNOB_State::KNOB_NULL;
uint8_t KNOB_Previous_Logical = 0B00000000; //  0B000000[KNOB_DATA_A][KNOB_DATA_B]
elapsedMillis elapsed_knob_moved_ms;
constexpr int32_t timeout_knob_moved_ms = 2000;
bool knob_moving = false;

static const uint16_t screenWidth  = 390;
static const uint16_t screenHeight = 390;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *disp_draw_buf;
static lv_disp_drv_t disp_drv;

Lvgl_CIT_UI CIT_UI;

// DXQ120MYB2416A
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */,
    LCD_SDIO1 /* SDIO1 */, LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

Arduino_GFX *gfx = new Arduino_SH8601(bus, LCD_RST /* RST */, 0 /* rotation */,
                                      false /* IPS */, LCD_WIDTH, LCD_HEIGHT);

/*TouchDrvCHSC5816 touch;
TouchDrvInterface *pTouch;

void CHSC5816_Initialization(void)
{
    //TouchDrvCHSC5816 *pd1 = static_cast<TouchDrvCHSC5816 *>(pTouch);

    touch.setPins(TOUCH_RST, TOUCH_INT);
    if (!touch.begin(Wire, CHSC5816_SLAVE_ADDRESS, IIC_SDA, IIC_SCL))
    {
        Serial.println("Failed to find CHSC5816 - check your wiring!");
        while (1)
        {
            delay(1000);
        }
    }

    Serial.println("Init CHSC5816 Touch device success!");
}*/

/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
    gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

    lv_disp_flush_ready(disp);
}

/*Read the touchpad*/
/*void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
    int16_t Touch_x[2], Touch_y[2];
    uint8_t touchpad = touch.getPoint(Touch_x, Touch_y);
    // TODO: add debounce here I guess?
    if (touchpad > 0)
    {
        data->state = LV_INDEV_STATE_PR;

        //Set the coordinates
        data->point.x = Touch_x[0];
        data->point.y = Touch_y[0];
    }
    else
    {
        data->state = LV_INDEV_STATE_REL;
    }
}*/

void lvgl_initialization(void)
{
    lv_init();

    CIT_UI.LCD_Width = gfx->width();
    CIT_UI.LCD_Height = gfx->height();

    disp_draw_buf = (lv_color_t *)heap_caps_malloc(sizeof(lv_color_t) * CIT_UI.LCD_Width * 40, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    while (!disp_draw_buf)
    {
        Serial.println("LVGL disp_draw_buf allocate failed!");
        delay(1000);
    }

    lv_disp_draw_buf_init(&draw_buf, disp_draw_buf, NULL, CIT_UI.LCD_Width * 40);

    /* Initialize the display */
    lv_disp_drv_init(&disp_drv);
    /* Change the following line to your display resolution */
    disp_drv.hor_res = CIT_UI.LCD_Width;
    disp_drv.ver_res = CIT_UI.LCD_Height;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    /*Initialize the (dummy) input device driver*/
    /*static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);*/
}

bool dry = false;
/*static void ui_Switch1_event_handler(lv_event_t *e)
{
    Serial.println("switchdry clicked");
    dry = !dry;
    if(dry)
        lv_obj_add_state(ui_switchdry, LV_STATE_CHECKED);
    else
        lv_obj_clear_state(ui_switchdry, LV_STATE_CHECKED);
}*/

void KNOB_Logical_Scan_Loop(void)
{
    uint8_t KNOB_Logical_Scan = 0B00000000;

    if (digitalRead(KNOB_DATA_A) == 1)
    {
        KNOB_Logical_Scan |= 0B00000010;
    }
    else
    {
        KNOB_Logical_Scan &= 0B11111101;
    }

    if (digitalRead(KNOB_DATA_B) == 1)
    {
        KNOB_Logical_Scan |= 0B00000001;
    }
    else
    {
        KNOB_Logical_Scan &= 0B11111110;
    }

    if (KNOB_Previous_Logical != KNOB_Logical_Scan)
    {
        if (KNOB_Logical_Scan == 0B00000000 || KNOB_Logical_Scan == 0B00000011)
        {
            KNOB_Previous_Logical = KNOB_Logical_Scan;
            KNOB_Trigger_Flag = true;
        }
        else
        {
            if (KNOB_Logical_Scan == 0B00000010)
            {
                switch (KNOB_Previous_Logical)
                {
                case 0B00000000:
                    KNOB_State_Flag = KNOB_State::KNOB_INCREMENT;
                    break;
                case 0B00000011:
                    KNOB_State_Flag = KNOB_State::KNOB_DECREMENT;
                    break;

                default:
                    break;
                }
            }
            if (KNOB_Logical_Scan == 0B00000001)
            {
                switch (KNOB_Previous_Logical)
                {
                case 0B00000000:
                    KNOB_State_Flag = KNOB_State::KNOB_DECREMENT;
                    break;
                case 0B00000011:
                    KNOB_State_Flag = KNOB_State::KNOB_INCREMENT;
                    break;

                default:
                    break;
                }
            }
        }
    }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Init");

  pinMode(KNOB_DATA_A, INPUT_PULLUP);
  pinMode(KNOB_DATA_B, INPUT_PULLUP);
  pinMode(BUZZER_DATA, OUTPUT);

  ledcAttachPin(BUZZER_DATA, 1);
  ledcSetup(1, 2000, 8);
  ledcWrite(1, 0); // 0 - 255

  pinMode(LCD_VCI_EN, OUTPUT);
  digitalWrite(LCD_VCI_EN, HIGH);

  //CHSC5816_Initialization();

  gfx->begin(40000000);
  gfx->fillScreen(BLACK);
  
  for (int i = 0; i <= 255; i++)
  {
      gfx->Display_Brightness(i);
      delay(3);
  }

  lvgl_initialization();

  ui_init();

  initWifi();
  initOta();
  initMqtt();

  Watchdog.enable(30000);

  ledcWrite(1, 127);
  delay(1000);
  ledcWrite(1, 0);

  //lv_obj_add_event_cb(ui_switchdry, ui_Switch1_event_handler, LV_EVENT_CLICKED, NULL);

  lv_arc_set_range(ui_arctrem,0,t_max);

  KNOB_Logical_Scan_Loop();
  KNOB_Trigger_Flag = false;

  lv_label_set_text(ui_labeltrem, "?:??");
}

char buffer[8];
int time_remaining_min = 0;

void set_time_display(int32_t time_min, bool alt_color) {
    lv_arc_set_value(ui_arctrem, time_min);
    uint32_t color_hex = alt_color ? 0xe2e73b : 0x00C024;
    lv_obj_set_style_arc_color(ui_arctrem, lv_color_hex(color_hex), LV_PART_INDICATOR|LV_STATE_DEFAULT);
    uint8_t th = uint8_t(floor(time_min / 60));
    uint8_t tm = uint8_t(time_min % 60);
    sprintf(buffer, "%d:%02d", th, tm);
    lv_label_set_text(ui_labeltrem, buffer);
}

bool knob_key_last = false;

void set_dry(bool state) {    
    if(state) {
        lv_obj_add_state(ui_switchdry, LV_STATE_CHECKED);
        air_dryer_override.publish("DRY", true);
    } else {
        lv_obj_clear_state(ui_switchdry, LV_STATE_CHECKED);
        air_dryer_override.publish("OVERRIDE", true);
    } 
    dry = state;
}

void toggle_dry(void) { set_dry(!dry); }

void loop() {
    Watchdog.reset();
    ArduinoOTA.handle(); // Handle OTA FW update
    checkWifiConnection(); // Check wifi connection (and try reconnecting if not connected)

    // Handle MQTT subscription
    if (!mqtt.connected()) MQTT_connect();
    subscription = mqtt.readSubscription();

    if (subscription == &Compressor_time_remaining) {
        String str = (char*)Compressor_time_remaining.lastread;
        time_remaining_min = (int)str.toInt();
        if(knob_moving){
            set_time_display(KNOB_Data, true);
        } else {
            set_time_display(time_remaining_min, false);
        }
        Serial.print("Got t_rem = ");
        Serial.print(time_remaining_min);
        Serial.println(" min");        
    } else if (subscription == &air_dryer_controller_status) {
        bool init = !strcmp((char*) air_dryer_controller_status.lastread, "Initialized");
        if(init) { set_dry(dry); }
    }
    mqttKeepAlive(); // Keep MQTT connection alive

    lv_timer_handler(); /* let the GUI do its work */

    if(!digitalRead(KNOB_KEY) && knob_key_last) { // Active low
        toggle_dry();          
    }
    knob_key_last = digitalRead(KNOB_KEY);

    KNOB_Logical_Scan_Loop();
    if (KNOB_Trigger_Flag == true)
    {
        elapsed_knob_moved_ms = 0;
        KNOB_Trigger_Flag = false;
        knob_moving = true;

        switch (KNOB_State_Flag)
        {
        case KNOB_State::KNOB_INCREMENT:
            KNOB_Data += knob_increment;
            if(KNOB_Data > t_max) { KNOB_Data = t_max; }
            Serial.printf("KNOB_Data: %d\n", KNOB_Data);
            break;
        case KNOB_State::KNOB_DECREMENT:
            KNOB_Data -= knob_increment;
            if(KNOB_Data < 0) { KNOB_Data = 0; }
            Serial.printf("KNOB_Data: %d\n", KNOB_Data);
            break;

        default:
            break;
        }
    }

    if(knob_moving){ 
        if( elapsed_knob_moved_ms >= timeout_knob_moved_ms ) {    
            if( KNOB_Data != time_remaining_min) {
                Serial.println("Setting t_rem based on knob");
                String sCmd = "T" + String(KNOB_Data);
                char csCmd[sCmd.length()+1];
                sCmd.toCharArray(csCmd, sCmd.length()+1);
                Compressor_Control.publish(csCmd);
                //Serial.print("sending CCCommand: ");
                //Serial.println(csCmd);                
                time_remaining_min = KNOB_Data;
                //set_time_display(time_remaining_min, false);
            }
            knob_moving = false;
        } else {
            set_time_display(KNOB_Data, true); 
        }             
    } else {
        KNOB_Data = time_remaining_min;
    }
}