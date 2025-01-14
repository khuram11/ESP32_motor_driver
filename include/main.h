#ifndef __MAIN_H__
#define __MAIN_H__

#include <Arduino.h>
#include <SPI.h>
#include "define.h"
#include <NHB_AD7124.h>


uint32_t ms1_pwm = 250;
uint8_t ms1_pwm_res = 8;
uint8_t ms1_pwm_ch = 0;
uint8_t ms1_pwm_duty = 50;

uint32_t ms2_pwm = 250;
uint8_t ms2_pwm_res = 8;
uint8_t ms2_pwm_ch = 1;
uint8_t ms2_pwm_duty = 50;



uint32_t md1_pwm = 250;
uint32_t md1_speed = 250;
uint32_t md1_res = 8;
uint8_t  md1_ch1 = 2;
uint8_t  md1_ch2 = 3;

uint32_t md2_pwm = 1000;
uint32_t md2_speed = 50;
uint32_t md2_res = 8;
uint8_t  md2_ch1 = 4;
uint8_t  md2_ch2 = 5;

volatile uint64_t pin1_pulses = 0;
volatile uint64_t pin2_pulses = 0;

volatile uint64_t p1_Hz = 0;
volatile uint64_t p2_Hz = 0;


typedef enum
{
  ERROR,
  OKAY
} Status_t;

typedef enum
{
  OFF,
  ON
} m_dir_t;

typedef enum
{
  PWMA,
  PWMB
} m_chan_t;



void sendResponse(const char *response);
uint32_t BAT_ReadReg(uint8_t regAddress, uint8_t regLength);
bool BAT_WriteReg(const uint8_t reg, const uint16_t value);
uint32_t ADC_readReg(AD7124_regIDs id);
int ADC_writeReg(AD7124_regIDs id, uint32_t value);
bool setRelay(uint8_t relay, bool state);
bool PWM(uint8_t motor_id, uint32_t mPWM, uint8_t speed);
bool processCommand(String command);
void InitSerial(void);
void SetupPins(void);
void ProcessSerialCommand(void);
void DumpRegisters(void);
void reset_pulses(void);
void step2_sig(void);
void step1_sig(void);
void checkWifi(void);
#endif //__MAIN_H__