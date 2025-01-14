
#include "main.h"
#include "WiFi.h"


typedef struct stepper_t
{
  uint8_t id;
  uint8_t usteppping_mode;
  uint8_t decay_mode;
  uint8_t dir;
  uint16_t stepping_freq;
} stepper_t;





// SPI objects
SPIClass spi1(HSPI);
SPIClass spi2(FSPI);
SPISettings spiSettings(10000000, MSBFIRST, SPI_MODE1);


// Registers
uint16_t ina229_registers[20];

// Relays state
bool DO1_state = false;
bool DO2_state = false;

const uint8_t BAT_ReadOnlyRegs[] = {0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x3E, 0x03F}; // write attempt to these will result in Error
const uint8_t BAT_AllRegs[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x10, 0x11, 0x3E, 0x03F};

const uint8_t ADC_ReadOnlyRegs[] = {0x00, 0x02, 0x05, 0x06, 0x08};
const uint8_t ADC_ReadRegsLen[] = {2, 1, 3, 3, 2, 1, 3, 3, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
const uint8_t ADC_totalRegs = 0x38;


Ad7124 adc(AD7124_CS, 4000000);


void setup()
{
  InitSerial();
  // checkWifi();
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
  md1_speed = speed;
    break;
  case 2:
  ledcSetup(md2_ch1,mPWM,md2_res);
  ledcAttachPin(MOT2_PWM_A_PIN,md2_ch1);
  ledcWrite(md2_ch1, (speed * 255 )/100);
  md2_pwm = mPWM;
  md2_speed = speed;
    break;
  default:
    break;
  }

  return OKAY;
}
int ADC_writeReg(AD7124_regIDs id, uint32_t value)
{
  adc.regs[id].value = value;
  int ret = adc.writeRegister(id);
  return ret;
}

uint32_t ADC_readReg(AD7124_regIDs id)
{
  int ret = adc.readRegister(id);
  return adc.regs[id].value;
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
    uint32_t RegAddr, RegVal;
    int res = sscanf(command.c_str(), "SET.ADC.%u.%u", &RegAddr, &RegVal);
    if (2 != res)
    {
      sendResponse("ERROR");
      return ERROR;
    }
    Serial.println("Valid Command Received for Setting ADC Register");
    Serial.printf("Reg \t %d,\tVal \t %d\n", RegAddr, RegVal);
    return ADC_writeReg((AD7124_regIDs)RegAddr, RegVal);
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
    int res = sscanf(command.c_str(), "SET.MS%u.%1u%1u", &m_id, &m_en, &m_dir);
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
  }  else if (command.startsWith("SET.SM"))
  {
    uint8_t m_id;
    uint32_t mPWM, mDC;
    int res = sscanf(command.c_str(), "SET.SM%u.%u.%u", &m_id, &mPWM, &mDC);
    if (3 != res)
      return ERROR;
    switch (m_id)
    {
      // assuming motor stepping mode, stepping frequency and decay mode is set at initilzation
    case 1:
      ms1_pwm = mPWM;
      ms1_pwm_duty = mDC;
      step1_sig();
      return OKAY;
    case 2:
      ms2_pwm = mPWM;
      ms2_pwm_duty = mDC;
      step2_sig();
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


void checkWifi(void) 
{
    WiFi.mode(WIFI_STA);
    WiFi.begin("pucca", "1r3n3!!..");
    Serial.println("\nConnecting");

    while(WiFi.status() != WL_CONNECTED){
          int n = WiFi.scanNetworks();  // This will scan for available Wi-Fi networks
  
  // Check if networks are found
  if (n == 0) {
    Serial.println("No networks found");
  } else 
  {
    Serial.print(n);
    Serial.println(" networks found:");
    
    // Print SSID of all available networks
    for (int i = 0; i < n; ++i)
     {
      Serial.print(i + 1);  // Network index (1-based)
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));  // SSID of the network
      Serial.print(" (Signal Strength: ");
      Serial.print(WiFi.RSSI(i));  // Signal strength in dBm
      Serial.println(" dBm)");
      }
    }
        delay(500);
    }

    Serial.println("\nConnected to the WiFi network");
    Serial.print("Local ESP32 IP: ");
    Serial.println(WiFi.localIP());
}

void SetupPins(void)
{

  pinMode(DO1_PIN, OUTPUT);
  pinMode(DO2_PIN, OUTPUT);
  pinMode(STEP1_PIN, OUTPUT);
  pinMode(STEP2_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  pinMode(HEART_BEAT_PIN, OUTPUT);
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
  spi1.begin(12, 13, 11, 10);

  // SPI.begin(36, 37, 35, 38);
  adc.begin(spi2);
  adc.setAdcControl (AD7124_OpMode_SingleConv, AD7124_FullPower, true);
  adc.setChannel(0, 0, AD7124_Input_AIN0, AD7124_Input_AIN1, true);
  adc.setPWRSW(1);

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
uint32_t  BAT_ReadReg(uint8_t regAddress, uint8_t regLength) {
    // Validate register length
    if (regLength < 2 || regLength > 5) {
        Serial.println("Invalid register length!");
        return 0;
    }

    // Prepare the SPI frame
    uint8_t readCommand = (regAddress << 2) | 0x01; // 6-bit regAddress + R/W = 1
    uint8_t dummyBytes = regLength - 1;

    // Allocate buffer for the response
    uint8_t response[5] = {0};

    // Begin SPI transaction
    spi1.beginTransaction(spiSettings);
    digitalWrite(INA229_CS, LOW); // Select the device

    // Send the read command
    spi1.transfer(readCommand);
    // Send dummy bytes and collect response
    for (uint8_t i = 0; i < regLength; i++) {
        response[i] = spi1.transfer(0x00); // Send dummy data (0x00) to clock out the response
    }

    digitalWrite(INA229_CS, HIGH); // Deselect the device
    spi1.endTransaction();

    // Assemble response into a 32-bit value
    uint32_t result = 0;
    for (uint8_t i = 0; i < regLength; i++) {
        result = (result << 8) | response[i];
    }

    return result;

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
  return true;
}

void reset_pulses(void)
{
  static uint32_t last_reset_time = 0;
  if(millis() - last_reset_time > 1000)
  {
    p1_Hz = 0; 
    p2_Hz = 0;
    last_reset_time = millis();
    static bool ledState = 0;
    digitalWrite(HEART_BEAT_PIN, ledState);
    ledState = !ledState; 

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
      int len = 2;
      if(i == 4 || i == 5 || i == 7 || i == 8) len = 3;
      if(i == 9 || i == 10) len = 5;
      Serial.printf("BAT.%02d.%d\n", i, BAT_ReadReg(i, len));
    }
    for (int i = 0 ; i <=  Reg_REG_NO ; i++ )
    {
      Serial.printf("ADC.%02d.%d\n", i, ADC_readReg((AD7124_regIDs)i));
    }
    Serial.printf("DI1.%ld\n", pin1_pulses);
    Serial.printf("DI2.%ld\n", pin2_pulses);
    Serial.printf("DI1S.%ld\n", p1_Hz);
    Serial.printf("DI2S.%ld\n", p2_Hz);
    Serial.printf("DO1.%02d\n", DO1_state);
    Serial.printf("DO2.%02d\n", DO2_state);
    Serial.printf("MS1.%02d.%02d\n", ms1_pwm, ms1_pwm_duty); 
    Serial.printf("MS2.%02d.%02d\n", ms2_pwm, ms2_pwm_duty); 
    Serial.printf("MD1.%02d.%02d\n", md1_pwm, md1_speed); 
    Serial.printf("MD2.%02d.%02d\n", md2_pwm, md2_speed); 
    Serial.println();
    lastDumpTime = millis();
  }
}

