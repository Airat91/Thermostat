#include "dcts.h"
#include "string.h"
#include "cmsis_os.h"

/*========== GLOBAL VARIABLES ==========*/

const uint8_t   id = 0x05;
const char      ver[11] = "1.0";
const char      name[11] = "Thermostat";
uint8_t         address = 0xFF;
rtc_t rtc = {
    .day = 0, 
    .month = 0, 
    .year = 2000, 
    .hour = 0, 
    .minute = 0, 
    .second = 0
};
float pwr = 0;

const uint8_t meas_num = MEAS_NUM;
const uint8_t rele_num = RELE_NUM;

meas_t meas[MEAS_NUM];
rele_t rele[RELE_NUM];

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

    strcpy (rele[0].name, "Heating");
    rele[0].state = 0;
}

void dcts_write_meas_value (uint8_t meas_channel, float value){
    taskENTER_CRITICAL();
    meas[meas_channel].value = value;
    taskEXIT_CRITICAL();
}
