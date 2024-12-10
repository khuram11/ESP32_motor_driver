#ifndef __DEFINE_H__
#define __DEFINE_H__


// Constants
// #ifdef ESP32_S2
#define INA229_CS 10
#define AD7124_CS 38

#define DI1_PIN 6
#define DI2_PIN 7

#define DO1_PIN 8
#define DO2_PIN 9

#define MOT1_PWM_A_PIN 2
#define MOT1_PWM_B_PIN 3


#define MOT2_PWM_A_PIN 4
#define MOT2_PWM_B_PIN 5


#define MOT_ENABLE1_PIN 18
#define STEP1_PIN 14
#define DIR1_PIN 17

#define MOT_ENABLE2_PIN 21
#define STEP2_PIN 19
#define DIR2_PIN 20
// #endif

#ifdef ESP32dev
#define INA229_CS 5
#define AD7124_CS 15

#define DI1_PIN 22
#define DI2_PIN 23

#define DO1_PIN 21
#define DO2_PIN 19

#define MOT1_PWM_A_PIN 17
#define MOT1_PWM_B_PIN 16


#define MOT2_PWM_A_PIN 2
#define MOT2_PWM_B_PIN 4


#define MOT_ENABLE1_PIN 26
#define STEP1_PIN 27
#define DIR1_PIN 25

#define MOT_ENABLE2_PIN 13
#define STEP2_PIN 14
#define DIR2_PIN 12
#endif





#define HEART_BEAT_PIN 34

#define SET_BAT_COMM_LEN 14









#endif //__DEFINE_H__