#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
//Constants
  const uint32_t interval = 100; //Display update interval

//Pin definitions
  //Row select and enable
  const int RA0_PIN = D3;
  const int RA1_PIN = D6;
  const int RA2_PIN = D12;
  const int REN_PIN = A5;

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;
  struct {
std::bitset<32> inputs;  
} sysState;



//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);
// const uint32_t AStepSize= (2^32)*440/22000; 85899345
const uint32_t AStepSize= 85899345;
volatile uint32_t currentStepSize;
// float factor= 2^(1/12);

// uint32_t stepSizes [12] = {          };
const uint32_t stepSizes [12] = {     51076056,54113196,57330934,60740009,64351798,68178355,72232451,76527616,81078185,AStepSize,91007185,96418754     };
// void generateStepSize(){
  
//   for (int i=0;i<12;i++){
  
//   stepSizes[i]=AStepSize*(factor^-9)*(factor^i);
//   }
// }
void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  analogWrite(OUTR_PIN, Vout + 128);
}

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
      digitalWrite(REN_PIN,LOW);
      digitalWrite(RA0_PIN, bitIdx & 0x01);
      digitalWrite(RA1_PIN, bitIdx & 0x02);
      digitalWrite(RA2_PIN, bitIdx & 0x04);
      digitalWrite(OUT_PIN,value);
      digitalWrite(REN_PIN,HIGH);
      delayMicroseconds(2);
      digitalWrite(REN_PIN,LOW);
}




std::bitset<4> readCols(){
  // digitalWrite(RA0_PIN, LOW);
  // digitalWrite(RA1_PIN, LOW);
  // digitalWrite(RA2_PIN, LOW);
  // digitalWrite(REN_PIN, HIGH);
  
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  // digitalWrite(REN_PIN, LOW);
  return result;}

void setRow(uint8_t rowIdx){
  digitalWrite(REN_PIN, LOW);
  // uint8_t rowIdxBinary = rowIdx;
  uint8_t bit0 = rowIdx & 0x01;
  uint8_t bit1 = rowIdx & 0x02;
  uint8_t bit2 = rowIdx & 0x04;
  digitalWrite(RA0_PIN, bit0);
  digitalWrite(RA1_PIN, bit1);
  digitalWrite(RA2_PIN, bit2);
  // std::bitset<4> result=readCols();
  digitalWrite(REN_PIN, HIGH);
  

  
}
void DisplayTask(void * pvParameters){
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1){
     vTaskDelayUntil( &xLastWakeTime, xFrequency );
     u8g2.clearBuffer();         // clear the internal memory
     u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  // u8g2.drawStr(2,10,"Helllo World!");// write something to the internal memory
  // std::bitset<4> inputs = readCols();
     u8g2.setCursor(2,10);
     u8g2.print(sysState.inputs.to_ulong(),BIN); 
  // u8g2.setCursor(2,10);
  // u8g2.print(keypressed); 
  // u8g2.setCursor(2,30);
  // u8g2.print(localStep); 
  
  
  // u8g2.print(count++);
  u8g2.sendBuffer();          // transfer internal memory to the display

  // //Toggle LED
  digitalToggle(LED_BUILTIN);

  }
}



void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    uint32_t keypressed=-1;
    // std::bitset<12> inputs;
    uint32_t localStep;
    for (int i=0; i<3;i++){
      setRow(i);
      delayMicroseconds(3);
      std::bitset<4> result=readCols();
    
    // inputs |= (std::bitset<4>(result.to_ulong()) << 12-4*(i+1)); 
      for (int j=0; j<4; j++){
        sysState.inputs[11-(i*4+j)]=result[j];
        if (result[j]==0){
          keypressed=i*4+j;
        }
      }
    
   } 
    if( keypressed==-1){
      localStep=0;
    }
    else{
    localStep=stepSizes[keypressed];
    }
  __atomic_store_n(&currentStepSize, localStep, __ATOMIC_RELAXED);
  }

}



void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  // generateStepSize();
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);
  TIM_TypeDef *Instance = TIM1;
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &scanKeysHandle );	/* Pointer to store the task handle */
  TaskHandle_t DisplayHandle = NULL;
  xTaskCreate(
    DisplayTask,		/* Function that implements the task */
    "Display",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &DisplayHandle );	/* Pointer to store the task handle */
  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  // static uint32_t next = millis();
  // static uint32_t count = 0;

  // while (millis() < next);  //Wait for next interval
  // scanKeysTask(NULL);
  
  // u8g2.setCursor(2,20);
  // uint32_t keypressed=-1;
  // std::bitset<12> inputs;
  // uint32_t localStep;
  // for (int i=0; i<3;i++){
  //   setRow(i);
  //   delayMicroseconds(3);
  //   std::bitset<4> result=readCols();
    
  //   // inputs |= (std::bitset<4>(result.to_ulong()) << 12-4*(i+1)); 
  //   for (int j=0; j<4; j++){
  //     inputs[11-(i*4+j)]=result[j];
  //     if (result[j]==0){
  //       keypressed=i*4+j;
  //     }
  //   }
    
  // }
  // if( keypressed==-1){
  //   localStep=0;
  // }
  // else{
  // localStep=stepSizes[keypressed];
  // currentStepSize=demostepSizes[keypressed];
  // currentStepSize=keypressed;
  // }

  // analogWrite(OUTR_PIN,  208);
  

  // next += interval;

  // //Update display
  // u8g2.clearBuffer();         // clear the internal memory
  // u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  // u8g2.drawStr(2,10,"Helllo World!");// write something to the internal memory
  // std::bitset<4> inputs = readCols();
  // u8g2.setCursor(2,10);
  // u8g2.print(inputs.to_ulong(),BIN); 
  // u8g2.setCursor(2,10);
  // u8g2.print(keypressed); 
  // u8g2.setCursor(2,30);
  // u8g2.print(localStep); 
  
  
  // u8g2.print(count++);
  // u8g2.sendBuffer();          // transfer internal memory to the display

  // //Toggle LED
  // digitalToggle(LED_BUILTIN);
  // __atomic_store_n(&currentStepSize, localStep, __ATOMIC_RELAXED);
  // currentStepSize=localStep;
}