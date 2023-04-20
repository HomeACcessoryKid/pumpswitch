/*  (c) 2020 HomeAccessoryKid
 *  This drives a pumpswitch
 *  It uses any ESP8266 with as little as 1MB flash. 
 *  GPIO-0 reads a button for manual instructions
 *  GPIO-12 instructs a relay to drive the motor of the pump
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
#include <adv_button.h>
#include "ds18b20/ds18b20.h"
#include "math.h"
#include "ping.h"
#include <rboot-api.h>
#include "mqtt-client.h"
#include <sysparam.h>

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
#ifndef BUTTON_PIN
 #error BUTTON_PIN is not specified
#endif
#ifndef SETPOINT
 #error SETPOINT is not specified
#endif
#ifndef HYSTERESIS
 #error HYSTERESIS is not specified
#endif

int idx; //the domoticz base index
#define PUBLISH(name) do {int n=mqtt_client_publish("{\"idx\":%d,\"nvalue\":0,\"svalue\":\"%.1f\"}", idx+name##_ix, name##_fv); \
                            if (n<0) printf("MQTT publish of %s failed because %s\n",#name,MQTT_CLIENT_ERROR(n)); \
                           } while(0)
#define tIN_fv  temp[IN]
#define tIN_ix  0
#define tOUT_fv temp[OUT]
#define tOUT_ix 1
#define tDELTA_fv delta_out
#define tDELTA_ix 3
char    *pinger_target=NULL;

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

homekit_characteristic_t cur_temp = HOMEKIT_CHARACTERISTIC_(CURRENT_TEMPERATURE, 1.0                       );


homekit_value_t active_get() {
    return HOMEKIT_UINT8(active.value.int_value);
}
void active_set(homekit_value_t value) {
    if (value.format != homekit_format_uint8) {
        UDPLUS("Invalid active-value format: %d\n", value.format);
        return;
    }
//     UDPLUS("Active:%3d\n",value.int_value);
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
#define REPEAT  3600 //in seconds = 1 hour
#define RUN  12*BEAT //in seconds = 2 minutes
#define STOP_FOR 180 //in seconds = 3 minutes, must be multiple of BEAT
#define SENSORS    2
#define  IN       14 //incoming water temperature
#define OUT       10 //  return water temperature
void state_task(void *argv) {
    bool on=true;
    bool prev_on=false;
    int  timer=RUN, prev_on_time=0;
    int  sampletimer=0;
    char status[40];
    float sample_out=0,initial_out=0,delta_out=0;
    
    ds18b20_addr_t addrs[SENSORS];
    float temps[SENSORS],temp[16];
    int sensor_count=0,id;
    float old_t;

    while( (sensor_count=ds18b20_scan_devices(SENSOR_PIN, addrs, SENSORS)) != SENSORS) {
        UDPLUS("Only found %d sensors\n",sensor_count);
        vTaskDelay(BEAT*1000/portTICK_PERIOD_MS);
    }

    while(1) {
        ds18b20_measure_and_read_multi(SENSOR_PIN, addrs, SENSORS, temps);
        for (int j = 0; j < SENSORS; j++) {
            // The DS18B20 address 64-bit and my batch turns out family C on https://github.com/cpetrich/counterfeit_DS18B20
            // I have manually selected that I have unique ids using the second hex digit of CRC
            id = (addrs[j]>>56)&0xF;
            temp[id] = temps[j];
//             printf("id=%d %2.4f\n",id,temps[j]);
        } 
        if (isnan(temp[OUT])) temp[OUT]=99.99F;
        if (isnan(temp[IN]))  temp[IN] =99.99F;
        else { //do not change state if broken input
            if (temp[IN]>SETPOINT+HYSTERESIS/2) on=true;
            if (temp[IN]<SETPOINT-HYSTERESIS/2) on=false;
        }
        if (on) prev_on_time+=BEAT; else prev_on_time=0;
        timer-=BEAT;
        if (prev_on_time>RUN || timer<=0) timer=REPEAT;
        status[0]=0;
        if (timer<=RUN) {
            on=true;
            strcpy(status," TIMER activated");
        }
        if (inhibit) {
            on=false;
            sprintf(status," inhibited for another %d seconds",(inhibit/10+1)*10);
        }
        
        //if pump is actived and return temp does not drop by >0.1 degrees in 120s then pump might be broken!
        if ( !prev_on && on ) {
            sample_out=initial_out=temp[OUT];
            sampletimer=RUN; //beats during which we are taking samples
        }
        if (sampletimer) {
            if (temp[OUT]<sample_out) sample_out=temp[OUT];
            sampletimer-=BEAT;
            if (!sampletimer) { //sampling is done
                delta_out=initial_out-sample_out;
                printf("Delta-out= %2.3f\n",delta_out);
                if (delta_out>1.0) delta_out=1.0; //not interested in bigger values
                delta_out*=16.0; //zoom out by 16 for more detail in MQTT. Samples are 1/16th degree granularity
                PUBLISH(tDELTA); //report delta_out to MQTT
            } 
        }
        prev_on=on; //store state for next round
        
        printf("R%2.3f - %2.3f C => %d%s\n", temp[OUT], temp[IN], on, status);
        PUBLISH(tIN);
        PUBLISH(tOUT);
        gpio_write(RELAY_PIN, on ? 1 : 0);
        gpio_write(  LED_PIN, on ? 0 : 1);
        if (on) {
            old_t=cur_temp.value.float_value;
            cur_temp.value.float_value=isnan(temp[OUT])?0.0F:(float)(int)(temp[OUT]*10+0.5)/10;
            if (old_t!=cur_temp.value.float_value) \
                homekit_characteristic_notify(&cur_temp,HOMEKIT_FLOAT(cur_temp.value.float_value));
        }
        vTaskDelay((BEAT*1000-780)/portTICK_PERIOD_MS);
    }
}

void inuse_task(void *argv) {
    while (1) {
        if (!active.value.int_value) {
            in_use.value.int_value=0;
            homekit_characteristic_notify(&in_use,HOMEKIT_UINT8(in_use.value.int_value));
            UDPLUS("In_Use 0 ... waiting 1 second ... In_Use 1\n");
            vTaskDelay(1000/portTICK_PERIOD_MS);
            inhibit=STOP_FOR;
            in_use.value.int_value=1;
            active.value.int_value=1;
            homekit_characteristic_notify(&in_use,HOMEKIT_UINT8(in_use.value.int_value));
            homekit_characteristic_notify(&active,HOMEKIT_UINT8(active.value.int_value));
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
        if (inhibit) inhibit--;
    }
}

void ping_task(void *argv) {
    int count=120,delay=1; //seconds
    ping_result_t res;
    ip_addr_t to_ping;
    inet_aton(pinger_target,&to_ping);
    
    printf("Pinging IP %s\n", ipaddr_ntoa(&to_ping));
    while(1){
        ping_ip(to_ping, &res);

        if (res.result_code == PING_RES_ECHO_REPLY) {
            count+=10; delay+=5;
            if (count>120) count=120; if (delay>60) delay=60;
            printf("good ping from %s %u ms -> count: %d s\n", ipaddr_ntoa(&res.response_ip), res.response_time_ms, count);
        } else {
            count--; delay=1;
            printf("failed ping err %d -> count: %d s\n", res.result_code, count);
        }
        if (count==0) {
            printf("restarting because can't ping home-hub\n");
            sdk_system_restart();  //#include <rboot-api.h>
        }
        vTaskDelay(delay*(1000/portTICK_PERIOD_MS));
    }
}

void singlepress_callback(uint8_t gpio, void *args) {
            UDPLUS("single press = inhibit 15 minutes\n");
            inhibit=900;
}

void doublepress_callback(uint8_t gpio, void *args) {
            UDPLUS("double press = inhibit 1 hour\n");
            inhibit=3600;
}

void longpress_callback(uint8_t gpio, void *args) {
            UDPLUS("long press = stop inhibit\n");
            inhibit=5;
}

mqtt_config_t mqttconf=MQTT_DEFAULT_CONFIG;
char error[]="error";
static void ota_string() {
    char *dmtczbaseidx1=NULL;
    char *otas;
    if (sysparam_get_string("ota_string", &otas) == SYSPARAM_OK) {
        mqttconf.host=strtok(otas,";");
        mqttconf.user=strtok(NULL,";");
        mqttconf.pass=strtok(NULL,";");
        dmtczbaseidx1=strtok(NULL,";");
        pinger_target=strtok(NULL,";");
    }
    if (mqttconf.host==NULL) mqttconf.host=error;
    if (mqttconf.user==NULL) mqttconf.user=error;
    if (mqttconf.pass==NULL) mqttconf.pass=error;
    if (dmtczbaseidx1==NULL) idx=1000; else idx=atoi(dmtczbaseidx1);
    if (pinger_target==NULL) pinger_target=error;
}

void device_init() {
    adv_button_set_evaluate_delay(10);
    adv_button_create(BUTTON_PIN, true, false);
    adv_button_register_callback_fn(BUTTON_PIN, singlepress_callback, 1, NULL);
    adv_button_register_callback_fn(BUTTON_PIN, doublepress_callback, 2, NULL);
    adv_button_register_callback_fn(BUTTON_PIN, longpress_callback, 3, NULL);
    gpio_enable(LED_PIN, GPIO_OUTPUT); gpio_write(LED_PIN, 0);
    gpio_enable( RELAY_PIN, GPIO_OUTPUT); gpio_write( RELAY_PIN, 1);
    gpio_set_pullup(SENSOR_PIN, true, true);

    //sysparam_set_string("ota_string", "192.168.178.5;pumpswitch;fakepassword;89;192.168.178.100"); //can be used if not using LCM
    ota_string();
    mqttconf.queue_size=6;
    mqtt_client_init(&mqttconf);

    xTaskCreate(state_task, "State", 512, NULL, 1, NULL);
    xTaskCreate(inuse_task, "InUse", 512, NULL, 1, NULL);
    xTaskCreate( ping_task, "PingT", 512, NULL, 1, NULL);
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
            HOMEKIT_SERVICE(TEMPERATURE_SENSOR,
                .characteristics=(homekit_characteristic_t*[]){
                    HOMEKIT_CHARACTERISTIC(NAME, "ReturnTemp"),
                    &cur_temp,
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
