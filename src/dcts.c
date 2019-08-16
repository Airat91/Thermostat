#include "dcts.h"
#include "string.h"
#include "cmsis_os.h"

/*========== GLOBAL VARIABLES ==========*/

const uint8_t   id = 0x06;
const char      ver[11] = "1.0";
const char      name[11] = "Thermostat";
uint8_t         address = 0xFF;
rtc_t rtc = {
    .day = 0, 
    .month = 0, 
    .year = 2000, 
    .weekday = 1,
    .hour = 0, 
    .minute = 0, 
    .second = 0
};
float pwr = 0;

#define MEAS_NUM    5
#define ACT_NUM     1

const uint8_t meas_num = MEAS_NUM;
const uint8_t act_num = ACT_NUM;

meas_t meas[MEAS_NUM];
act_t act[ACT_NUM];

void dcts_init () {
    strcpy (meas[0].name, "Floor");
    strcpy (meas[0].unit, "°C");
    meas[0].value = 0;
    
    strcpy (meas[1].name, "Reg");
    strcpy (meas[1].unit, "°C");
    meas[1].value = 0;
    
    strcpy (meas[2].name, "Rh");
    strcpy (meas[2].unit, "%");
    meas[2].value = 0;

    strcpy (meas[3].name, "ADC0");
    strcpy (meas[3].unit, "V");
    meas[3].value = 0;
    strcpy (meas[4].name, "ADC1");
    strcpy (meas[4].unit, "V");
    meas[4].value = 0;

    strcpy (act[0].name, "Floor");
    strcpy (act[0].unit, "°C");
    act[0].set_value = 27.0f;
    act[0].meas_value = 0.0f;
    act[0].state.control = TRUE;
    act[0].state.pin_state = FALSE;
    act[0].state.short_cir = FALSE;
    act[0].state.fall = FALSE;
}

void dcts_write_meas_value (uint8_t meas_channel, float value){
    taskENTER_CRITICAL();
    meas[meas_channel].value = value;
    taskEXIT_CRITICAL();
}
void dcts_write_act_meas_value (uint8_t act_channel, float value){
    taskENTER_CRITICAL();
    act[act_channel].meas_value = value;
    taskEXIT_CRITICAL();
}
void dcts_write_act_set_value (uint8_t act_channel, float value){
    taskENTER_CRITICAL();
    act[act_channel].set_value = value;
    taskEXIT_CRITICAL();
}
