
#include "main.h"



typedef struct stepper_t
{
  uint8_t id;
  uint8_t usteppping_mode;
  uint8_t decay_mode;
  uint8_t dir;
  uint16_t stepping_freq;
} stepper_t;



uint16_t BAT_ReadReg(uint8_t RegAddr);
uint32_t ADC_readReg(uint8_t regAddress, uint8_t numBytes);

// SPI objects
SPIClass spi1(HSPI);
SPIClass spi2(FSPI);

// Registers
uint16_t ina229_registers[20];
uint32_t ad7124_registers[14];

// Relays state
bool DO1_state = false;
bool DO2_state = false;

const uint8_t BAT_ReadOnlyRegs[] = {0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x3E, 0x03F}; // write attempt to these will result in Error
const uint8_t BAT_AllRegs[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x3E, 0x03F};

const uint8_t ADC_ReadOnlyRegs[] = {0x00, 0x02, 0x05, 0x06, 0x08};
const uint8_t ADC_ReadRegsLen[] = {2, 1, 3, 3, 2, 1, 3, 3, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
const uint8_t ADC_totalRegs = 0x38;



void setup()
{
  InitSerial();
  SetupPins();
}

void loop()
{

  ProcessSerialCommand();
  DumpRegisters();
  reset_pulses();
}


void sendResponse(const char *response)
{
  Serial.println(response);
}



bool setRelay(uint8_t relay, bool state)
{
  if (relay == 1)
  {
    if (DO2_state && state)
      return false; 
    digitalWrite(DO1_PIN, state ? HIGH : LOW);
    DO1_state = state;
  }
  else if (relay == 2)
  {
    if (DO1_state && state)
      return ERROR; 
    digitalWrite(DO2_PIN, state ? HIGH : LOW);
    DO2_state = state;
  }
  else
  {
    return false;
  }
  return true;
}

bool PWM(uint8_t motor_id, uint32_t mPWM, uint8_t speed)
{
  switch (motor_id)
  {
  case 1:
  ledcSetup(md1_ch1,mPWM,md1_res);
  ledcAttachPin(MOT1_PWM_A_PIN,md1_ch1);
  ledcWrite(md1_ch1, (speed * 255 )/100);
  md1_pwm = mPWM;
    break;
  case 2:
  ledcSetup(md2_ch1,mPWM,md2_res);
  ledcAttachPin(MOT2_PWM_A_PIN,md2_ch1);
  ledcWrite(md2_ch1, (speed * 255 )/100);
  md2_pwm = mPWM;
    break;
  default:
    break;
  }

  return OKAY;
}
bool ADC_writeReg(uint8_t regAddress, uint32_t value, uint8_t numBytes)
{
  for (uint8_t i_reg : ADC_ReadOnlyRegs)
  {
    if (i_reg == regAddress)
      return false;
  }
  digitalWrite(AD7124_CS, LOW);
  uint8_t commandByte = (0 << 7) | (regAddress & 0x3F);
  spi2.transfer(commandByte);
  for (int8_t i = numBytes - 1; i >= 0; i--)
  {
    spi2.transfer((value >> (8 * i)) & 0xFF);
  }
  digitalWrite(AD7124_CS, HIGH);
  return value == ADC_readReg(regAddress, numBytes);
}

uint32_t ADC_readReg(uint8_t regAddress, uint8_t numBytes)
{
  digitalWrite(AD7124_CS, LOW);
  uint8_t commandByte = (1 << 7) | (regAddress & 0x3F);
  spi2.transfer(commandByte);
  uint32_t value = 0;
  for (uint8_t i = 0; i < numBytes; i++)
  {
    value = (value << 8) | spi2.transfer(0x00);
  }
  digitalWrite(AD7124_CS, HIGH);
  return value;
}

// Parse and execute commands
bool processCommand(String command)
{

  if (command.startsWith("SET.BAT."))
  {
    uint16_t RegAddr, RegVal;
    int res = sscanf(command.c_str(), "SET.BAT.%u.%u", &RegAddr, &RegVal);
    if (2 != res)
    {
      sendResponse("ERROR");
      return ERROR;
    }
    Serial.println("Valid Command Received for Setting BAT Register");
    Serial.printf("Reg \t %d,\tVal \t %d\n", RegAddr, RegVal);
    return (BAT_WriteReg(RegAddr, RegVal));
  }
  else if (command.startsWith("SET.ADC."))
  {
    uint16_t RegAddr, RegVal;
    int res = sscanf(command.c_str(), "SET.ADC.%u.%u", &RegAddr, &RegVal);
    if (2 != res)
    {
      sendResponse("ERROR");
      return ERROR;
    }
    Serial.println("Valid Command Received for Setting ADC Register");
    Serial.printf("Reg \t %d,\tVal \t %d\n", RegAddr, RegVal);
    return ADC_writeReg(RegAddr, RegVal, ((RegVal / 256) + 1));
  }
  else if (command.startsWith("SET.DO"))
  {
    uint8_t DO_ID, DO_State;
    int res = sscanf(command.c_str(), "SET.DO%u.%u", &DO_ID, &DO_State);
    if (1 == DO_ID)
    {
      if (DO_State && DO2_state)
      {
        return ERROR;
      }
      digitalWrite(DO1_PIN, DO_State);
      DO1_state = DO_State;
      return OKAY;
    }
    else if (2 == DO_ID)
    {
      if (DO_State && DO1_state)
      {
        return ERROR;
      }
      digitalWrite(DO2_PIN, DO_State);
      DO2_state = DO_State;
      return OKAY;
    }
    return ERROR;
  }
  else if (command.startsWith("SET.MD"))
  {
    // assuming motor stepping mode and decay mode is set at initilzation
    uint8_t m_id, m_PWM, m_speed;
    int res = sscanf(command.c_str(), "SET.MD%u.%u.%u", &m_id, &m_PWM, &m_speed);
    if (3 != res)
      return ERROR;
    return PWM(m_id, m_PWM, m_speed);
  }
  else if (command.startsWith("SET.MS"))
  {
    uint8_t m_id, m_dir, m_en;
    int res = sscanf(command.c_str(), "SET.MS%u.%1u.%u", &m_id, &m_en, &m_dir);
    if (3 != res)
      return ERROR;
    switch (m_id)
    {
      // assuming motor stepping mode, stepping frequency and decay mode is set at initilzation
    case 1:
      digitalWrite(MOT_ENABLE1_PIN, m_en);
      digitalWrite(DIR1_PIN, m_dir);
      return OKAY;
    case 2:
      digitalWrite(MOT_ENABLE2_PIN, m_en);
      digitalWrite(DIR2_PIN, m_dir);
      return OKAY;
    default:
      return ERROR;
    }
  }
  return ERROR;
}

void InitSerial(void)
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("---- Starting Motor Driver Application ----\n");
}

void step1_sig(void)
{
  ledcSetup(ms1_pwm_ch,ms1_pwm,ms1_pwm_res);
  ledcAttachPin(STEP1_PIN,ms1_pwm_ch);
  ledcWrite(ms1_pwm_ch,128);
}
void step2_sig(void)
{
  ledcSetup(ms2_pwm_ch,ms2_pwm,ms2_pwm_res);
  ledcAttachPin(STEP1_PIN,ms2_pwm_ch);
  ledcWrite(ms2_pwm_ch,128);
}

void DC1_PWM1(void)
{
  ledcSetup(md1_ch1,md1_pwm,md1_res);
  ledcAttachPin(MOT1_PWM_A_PIN,md1_ch1);
  ledcWrite(md1_ch1,128);
}
void DC2_PWM1(void)
{
  ledcSetup(md2_ch1,md2_pwm,md2_res);
  ledcAttachPin(MOT2_PWM_A_PIN,md2_ch1);
  ledcWrite(md2_ch1,128);
}

void IRAM_ATTR DI1_ISR()
{
  pin1_pulses++;
  p1_Hz++;
}
void IRAM_ATTR DI2_ISR()
{
  pin2_pulses++;
  p2_Hz++;
}

void SetupPins(void)
{

  pinMode(DO1_PIN, OUTPUT);
  pinMode(DO2_PIN, OUTPUT);
  pinMode(STEP1_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(DI1_PIN, INPUT);
  pinMode(DI2_PIN, INPUT);

  digitalWrite(DO1_PIN, LOW);
  digitalWrite(DO2_PIN, LOW);

  step1_sig(); 
  step2_sig();

  // DC1_PWM1();
  // DC2_PWM1();

  attachInterrupt(DI1_PIN, DI1_ISR, RISING);
  attachInterrupt(DI2_PIN, DI2_ISR, RISING);

}

void ProcessSerialCommand(void)
{
  if (Serial.available())
  {
    String command = Serial.readStringUntil('\n');
    if (processCommand(command))
      sendResponse("OK");
    else
      sendResponse("ERROR");
  }
}
uint16_t BAT_ReadReg(uint8_t RegAddr)
{
  digitalWrite(INA229_CS, LOW);             // Enable SPI slave
  spi1.transfer16(RegAddr | 0x8000);        // Send register address (read mode)
  uint16_t value = spi1.transfer16(0x0000); // Receive 16-bit value
  digitalWrite(INA229_CS, HIGH);            // Disable SPI slave
  return value;
}


bool BAT_WriteReg(const uint8_t reg, const uint16_t value)
{
  for (uint8_t i_reg : BAT_ReadOnlyRegs)
  {
    if (reg == i_reg)
      return false;
  }
  // write the value to the register
  digitalWrite(INA229_CS, LOW);  // Enable SPI slave
  spi1.transfer16(reg & 0x7FFF); // Send register address (write mode)
  spi1.transfer16(value);        // Send 16-bit value to write
  digitalWrite(INA229_CS, HIGH); // Disable SPI slave
  // readback to verify
  return value == BAT_ReadReg(reg);
}

void reset_pulses(void)
{
  static uint32_t last_reset_time = 0;
  if(millis() - last_reset_time > 1000)
  {
    p1_Hz = 0; 
    p2_Hz = 0;
    last_reset_time = millis();
  }
}
void DumpRegisters(void)
{
  static unsigned long lastDumpTime = 0;
  if (millis() - lastDumpTime >= 5000)
  {
    Serial.println("");
    for (int i : BAT_AllRegs)
    {
      Serial.printf("BAT.%02d.%d\n", i, BAT_ReadReg(i));
    }
    for (int i = 0; i < ADC_totalRegs; i++)
    {
      Serial.printf("ADC.%02d.%d\n", i, ADC_readReg(i, ADC_ReadRegsLen[i]));
    }
    Serial.printf("DI1.%ld\n", pin1_pulses);
    Serial.printf("DI2.%ld\n", pin2_pulses);
    Serial.printf("DI1S.%ld\n", p1_Hz);
    Serial.printf("DI2S.%ld\n", p2_Hz);
    Serial.printf("DOS.%02d.%02d\n", DO1_state, DO2_state);
    Serial.printf("MS1.%02d.%02d\n", ms1_pwm, ms1_pwm_duty); 
    Serial.printf("MS2.%02d.%02d\n", ms2_pwm, ms2_pwm_duty); 
    Serial.printf("MD1.%02d.%02d\n", md1_pwm, md1_duty); 
    Serial.printf("MD2.%02d.%02d\n", md2_pwm, md2_duty); 
    Serial.println();
    lastDumpTime = millis();
  }
}

