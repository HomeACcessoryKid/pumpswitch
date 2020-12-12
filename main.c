/*  (c) 2020 HomeAccessoryKid
 *  This drives a pumpswitch
 *  It uses any ESP8266 with as little as 1MB flash. 
 *  GPIO-0 reads a button for manual instructions
 *  GPIO-12 instructs a relay to drive the motor of thepump
 *  GPIO-13 enables the LED to show the state of the pump
 *  GPIO-2 is used as a single one-wire DS18B20 sensor to measure the watersupply temperature
 *  UDPlogger is used to have remote logging
 *  LCM is enabled in case you want remote updates
 */

#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
//#include <espressif/esp_system.h> //for timestamp report only
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>
#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <string.h>
#include "lwip/api.h"
// #include <wifi_config.h>
#include <udplogger.h>
//#include <adv_button.h>
#include "ds18b20/ds18b20.h"

#ifndef VERSION
 #error You must set VERSION=x.y.z to match github version tag x.y.z
#endif

#ifndef RELAY_PIN
 #error RELAY_PIN is not specified
#endif
#ifndef SENSOR_PIN
 #error SENSOR_PIN is not specified
#endif
#ifndef LED_PIN
 #error LED_PIN is not specified
#endif
#ifndef SETPOINT
 #error SETPOINT is not specified
#endif
#ifndef HYSTERESIS
 #error HYSTERESIS is not specified
#endif

int inhibit=0; //seconds pump will be inhibited

/* ============== BEGIN HOMEKIT CHARACTERISTIC DECLARATIONS =============================================================== */
// add this section to make your device OTA capable
// create the extra characteristic &ota_trigger, at the end of the primary service (before the NULL)
// it can be used in Eve, which will show it, where Home does not
// and apply the four other parameters in the accessories_information section

#include "ota-api.h"
homekit_characteristic_t ota_trigger  = API_OTA_TRIGGER;
homekit_characteristic_t manufacturer = HOMEKIT_CHARACTERISTIC_(MANUFACTURER,  "X");
homekit_characteristic_t serial       = HOMEKIT_CHARACTERISTIC_(SERIAL_NUMBER, "1");
homekit_characteristic_t model        = HOMEKIT_CHARACTERISTIC_(MODEL,         "Z");
homekit_characteristic_t revision     = HOMEKIT_CHARACTERISTIC_(FIRMWARE_REVISION,  "0.0.0");

// next use these two lines before calling homekit_server_init(&config);
//    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
//                                      &model.value.string_value,&revision.value.string_value);
//    config.accessories[0]->config_number=c_hash;
// end of OTA add-in instructions

homekit_value_t active_get();
void active_set(homekit_value_t value);
homekit_characteristic_t active = HOMEKIT_CHARACTERISTIC_(ACTIVE, 1, .getter=active_get, .setter=active_set);
homekit_characteristic_t in_use = HOMEKIT_CHARACTERISTIC_(IN_USE, 1                                        );


homekit_value_t active_get() {
    return HOMEKIT_UINT8(active.value.int_value);
}
void active_set(homekit_value_t value) {
    if (value.format != homekit_format_uint8) {
        UDPLUS("Invalid active-value format: %d\n", value.format);
        return;
    }
    UDPLUS("Active:%3d\n",value.int_value);
    active.value=value;
}

// void identify_task(void *_args) {
//     vTaskDelete(NULL);
// }

void identify(homekit_value_t _value) {
    UDPLUS("Identify\n");
//    xTaskCreate(identify_task, "identify", 256, NULL, 2, NULL);
}

/* ============== END HOMEKIT CHARACTERISTIC DECLARATIONS ================================================================= */


#define BEAT      10 //in seconds
#define REPEAT 14400 //in seconds = 4 hours
#define RUN      120 //in seconds = 2 minutes
#define STOP_FOR 300 //in seconds = 5 minutes, must be multiple of BEAT
void state_task(void *argv) {
    bool on=true;
    int timer=RUN, prev_on=0;
    uint8_t scratchpad[8];
    float temp;
    char status[40];
    
    while (1) {
        temp=99.99;
        if (ds18b20_measure(SENSOR_PIN, DS18B20_ANY, true) && ds18b20_read_scratchpad(SENSOR_PIN, DS18B20_ANY, scratchpad)) {
            temp = ((float)(scratchpad[1] << 8 | scratchpad[0]) * 625.0)/10000;
        }
        if (temp>SETPOINT+HYSTERESIS/2) on=true;
        if (temp<SETPOINT-HYSTERESIS/2) on=false;
        if (inhibit) on=false;
        if (on) prev_on+=BEAT; else prev_on=0;
        timer-=BEAT;
        if (prev_on>RUN || timer<=0) timer=REPEAT;
        status[0]=0;
        if (timer<=RUN) {
            on=true;
            strcpy(status," TIMER activated");
        } else {
            if (inhibit) sprintf(status," inhibited for another %d seconds",inhibit);
        }
        printf("%2.3f C => %d%s\n", temp, on, status);
        gpio_write(RELAY_PIN, on ? 1 : 0);
        gpio_write(  LED_PIN, on ? 0 : 1);
        vTaskDelay((BEAT*1000-750)/portTICK_PERIOD_MS);
    }
}

void inuse_task(void *argv) {
    while (1) {
        if (!active.value.int_value) {
            in_use.value.int_value=0;
            homekit_characteristic_notify(&in_use,HOMEKIT_UINT8(in_use.value.int_value));
            UDPLUS("In_Use %d ... waiting 1 second ... ",in_use.value.int_value);
            vTaskDelay(1000/portTICK_PERIOD_MS);
            inhibit=STOP_FOR;
            in_use.value.int_value=1;
            active.value.int_value=1;
            homekit_characteristic_notify(&in_use,HOMEKIT_UINT8(in_use.value.int_value));
            homekit_characteristic_notify(&active,HOMEKIT_UINT8(active.value.int_value));
            UDPLUS("In_Use %d\n",in_use.value.int_value);
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
        if (inhibit) inhibit--;
    }
}

// void singlepress_callback(uint8_t gpio, void *args) {
//             UDPLUS("single press = stop here\n");
//             active.value.int_value=in_use.value.int_value;
//             homekit_characteristic_notify(&active,HOMEKIT_UINT8(active.value.int_value));
// }
// 
// void doublepress_callback(uint8_t gpio, void *args) {
//             UDPLUS("double press = go open\n");
//             active.value.int_value=100;
//             homekit_characteristic_notify(&active,HOMEKIT_UINT8(active.value.int_value));
// }
// 
// void longpress_callback(uint8_t gpio, void *args) {
//             UDPLUS("long press = go close\n");
//             active.value.int_value=0;
//             homekit_characteristic_notify(&active,HOMEKIT_UINT8(active.value.int_value));
// }


void device_init() {
//     adv_button_set_evaluate_delay(10);
//     adv_button_create(BUTTON_PIN, true, false);
//     adv_button_register_callback_fn(BUTTON_PIN, singlepress_callback, 1, NULL);
//     adv_button_register_callback_fn(BUTTON_PIN, doublepress_callback, 2, NULL);
//     adv_button_register_callback_fn(BUTTON_PIN, longpress_callback, 3, NULL);
    gpio_enable(LED_PIN, GPIO_OUTPUT); gpio_write(LED_PIN, 0);
    gpio_enable( RELAY_PIN, GPIO_OUTPUT); gpio_write( RELAY_PIN, 1);
    gpio_set_pullup(SENSOR_PIN, true, true);
    xTaskCreate(state_task, "State", 512, NULL, 1, NULL);
    xTaskCreate(inuse_task, "InUse", 512, NULL, 1, NULL);
}

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(
        .id=1,
        .category=homekit_accessory_category_other,
        .services=(homekit_service_t*[]){
            HOMEKIT_SERVICE(ACCESSORY_INFORMATION,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "pumpswitch"),
                    &manufacturer,
                    &serial,
                    &model,
                    &revision,
                    HOMEKIT_CHARACTERISTIC(IDENTIFY, identify),
                    NULL
                }),
            HOMEKIT_SERVICE(VALVE, .primary=true,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "Pump"),
                    &active,
                    &in_use,
                    HOMEKIT_CHARACTERISTIC(VALVE_TYPE, 0),
                    &ota_trigger,
                    NULL
                }),
            NULL
        }),
    NULL
};

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "111-11-111"
};


void user_init(void) {
    uart_set_baud(0, 115200);
    udplog_init(3);
    UDPLUS("\n\n\nPumpSwitch " VERSION "\n");

    device_init();
    
    int c_hash=ota_read_sysparam(&manufacturer.value.string_value,&serial.value.string_value,
                                      &model.value.string_value,&revision.value.string_value);
    //c_hash=2; revision.value.string_value="0.0.2"; //cheat line
    config.accessories[0]->config_number=c_hash;
    
    homekit_server_init(&config);
}
