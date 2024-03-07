#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <STM32FreeRTOS.h>
#include <ES_CAN.h>
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
  QueueHandle_t msgInQ;
  QueueHandle_t msgOutQ;
  SemaphoreHandle_t CAN_TX_Semaphore;
  struct {
std::bitset<32> inputs;
int rotationValues[4] ={0,0,0,0} ;
SemaphoreHandle_t mutex;
} sysState;
uint8_t TX_Message[8] = {0};


//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);
// const uint32_t AStepSize= (2^32)*440/22000; 85899345
const uint32_t AStepSize= 85899345;
volatile uint32_t currentStepSize;
// float factor= 2^(1/12);

// uint32_t stepSizes [12] = {          };
const uint32_t stepSizes [12] = {     51076056,54113196,57330934,60740009,64351798,68178355,72232451,76527616,81078185,AStepSize,91007185,96418754     };

void CAN_TX_ISR (void) {
	xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
}

void CAN_RX_ISR (void) {
	uint8_t RX_Message_ISR[8];
	uint32_t ID;
	CAN_RX(ID, RX_Message_ISR);
	xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);

}

void sampleISR() {
  static uint32_t phaseAcc = 0;
  phaseAcc += currentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  int val;
  val=__atomic_load_n(&sysState.rotationValues[2],__ATOMIC_RELAXED);
  Vout = Vout >> (8 - val);
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
  
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  digitalWrite(REN_PIN, LOW);
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
  uint8_t RX_Message[8]={0};



void CAN_TX_Task (void * pvParameters) {
	uint8_t msgOut[8];
	while (1) {
		xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
		xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
		CAN_TX(0x123, msgOut);
	}
}

void DecodeTask(void * pvParameters){

  uint32_t ID;
  int localPress=-1;
  uint32_t localStep;
  while(1){

    xQueueReceive(msgInQ, RX_Message, portMAX_DELAY);
    if (RX_Message[0]=='R'){
        localPress=-1;
        localStep=0;
    }
    else{
      int octave=RX_Message[1];
      localPress=RX_Message[2];
      localStep=stepSizes[localStep]*(pow(2,(-4+octave)));


    }
    

    __atomic_store_n(&currentStepSize, localStep, __ATOMIC_RELAXED);


  }
}
void DisplayTask(void * pvParameters){
  // uint32_t ID;
  // uint8_t RX_Message[8]={0};

  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1){
      
    
	   
     vTaskDelayUntil( &xLastWakeTime, xFrequency );
     u8g2.clearBuffer();         // clear the internal memory
     u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
  // u8g2.drawStr(2,10,"Helllo World!");// write something to the internal memory
  // std::bitset<4> inputs = readCols();
     u8g2.setCursor(2,10);
     xSemaphoreGive(sysState.mutex);
     u8g2.print(sysState.inputs.to_ulong(),BIN); 
     

    u8g2.setCursor(2,30);
    u8g2.print(sysState.rotationValues[0],DEC); 

    u8g2.setCursor(12,30);
    u8g2.print(sysState.rotationValues[1],DEC); 

    u8g2.setCursor(22,30);
    u8g2.print(sysState.rotationValues[2],DEC); 

    u8g2.setCursor(32,30);
    u8g2.print(sysState.rotationValues[3],DEC); 
    xSemaphoreGive(sysState.mutex);
    u8g2.setCursor(66,30);
    u8g2.print((char) TX_Message[0]);
    u8g2.print(TX_Message[1]);
    u8g2.print(TX_Message[2]);
    u8g2.setCursor(66,20);
    u8g2.print((char) RX_Message[0]);
    u8g2.print(RX_Message[1]);
    u8g2.print(RX_Message[2]);
    // u8g2.print(pow(2,(-4+RX_Message[1])));
      // u8g2.setCursor(2,30);
      // u8g2.print(localStep); 
  
  
  // u8g2.print(count++);
  u8g2.sendBuffer();          // transfer internal memory to the display

  // //Toggle LED
  digitalToggle(LED_BUILTIN);

  }
}



void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1){
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    uint32_t keypressed=-1;
    // std::bitset<12> inputs;
    uint32_t localStep;
    // int rotationVariable=0;
    
    for (int i=0; i<5;i++){
      setRow(i);
      delayMicroseconds(3);
      std::bitset<4> result=readCols();
    
      if (i<3){ //keys
        for (int j=0; j<4; j++){
        if (sysState.inputs[11-(i*4+j)]!=result[j]){
            TX_Message[1] = 4;
            TX_Message[2] = (i*4+j);
          if (result[j]==0){
            TX_Message[0] = 'P';
            keypressed=i*4+j;
            localStep=stepSizes[keypressed];
              
          }
          else{
            TX_Message[0] = 'R';
            keypressed=-1;
            localStep=0;
          }
          __atomic_store_n(&currentStepSize, localStep, __ATOMIC_RELAXED);
          // CAN_TX(0x123, TX_Message);
          xQueueSend( msgOutQ, TX_Message, portMAX_DELAY);
        }

          // if (result[j]==0){
            
            
          // }
          xSemaphoreTake(sysState.mutex, portMAX_DELAY);
          sysState.inputs[11-(i*4+j)]=result[j];
          xSemaphoreGive(sysState.mutex);
        }
      }
      else if(i>=3){//knobs
        
        for (int j=0; j<2; j++){
          // rotationVariable=(sysState.inputs[4*i+2*j,4*i+2*j+1]).to_ulong()
          std::bitset<2> knob;
          std::bitset<2> knobnew; 
          knobnew[0]=result[2*j];
          knobnew[1]=result[2*j+1];
          xSemaphoreTake(sysState.mutex, portMAX_DELAY);
          knob[0]=sysState.inputs[4*i+2*j];
          knob[1]=sysState.inputs[4*i+2*j+1];
          xSemaphoreGive(sysState.mutex);
          // knobnew[0,1]=result[2*j,2*j+1];
          // Serial.println(knobnew.to_ulong(),BIN);
          int val=sysState.rotationValues[3-((i-3)*2+j)];
        if (((knob==std::bitset<2>(0b00)) && (knobnew==std::bitset<2>(0b01))) || 
          ((knob==std::bitset<2>(0b11)) && (knobnew==std::bitset<2>(0b00))) ||
         ( (knob==std::bitset<2>(0b11)) && (knobnew==std::bitset<2>(0b10))) ||
          ((knob==std::bitset<2>(0b10)) && (knobnew==std::bitset<2>(0b01)))
        )
          {
          val+=1;
        }
        else if (((knob==std::bitset<2>(0b01)) && (knobnew==std::bitset<2>(0b00))) ||
          ((knob==std::bitset<2>(0b00)) && (knobnew==std::bitset<2>(0b11))) ||
          (((knob==std::bitset<2>(0b10)) && (knobnew==std::bitset<2>(0b11)) )||
        ((knob==std::bitset<2>(0b01)) && (knobnew==std::bitset<2>(0b10))))
        )
        {
          val-=1;
        }

        if (val>8){
          val=8;
        }
        else if (val<0){
          val=0;
        }
        // for (int k=0; k<4; k++){
        // Serial.print(sysState.rotationValues[k]);
        // Serial.print("  ");
        // }
        // Serial.println("");

        xSemaphoreTake(sysState.mutex, portMAX_DELAY);
        // sysState.rotationValues[3-((i-3)*2+j)]=val;
        sysState.inputs[4*i+2*j]=knobnew[0];
        sysState.inputs[4*i+2*j+1]=knobnew[1];
        xSemaphoreGive(sysState.mutex);
        knob=knobnew;
        __atomic_store_n(&sysState.rotationValues[3-((i-3)*2+j)], val, __ATOMIC_RELAXED);

        }
      }
   } 
    // if( keypressed==-1){
    //   localStep=0;
    // }
    // else{
    //   localStep=stepSizes[keypressed];
    // }

  }

}



void setup() {
  // put your setup code here, to run once:

  //Set pin directions
  // generateStepSize();
  CAN_Init(true);
  setCANFilter(0x123,0x7ff);
  CAN_RegisterRX_ISR(CAN_RX_ISR);
  CAN_RegisterTX_ISR(CAN_TX_ISR);
  CAN_Start();
  msgInQ = xQueueCreate(36,8);
  msgOutQ = xQueueCreate(36,8);
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
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3,3);
  

  sysState.mutex = xSemaphoreCreateMutex();
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
  TaskHandle_t DecodeHandle = NULL;
  xTaskCreate(
    DecodeTask,		/* Function that implements the task */
    "Decode",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &DecodeHandle );	/* Pointer to store the task handle */
  TaskHandle_t CAN_TXHandle = NULL;
  xTaskCreate(
    CAN_TX_Task,		/* Function that implements the task */
    "CAN_TX",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &CAN_TXHandle );	/* Pointer to store the task handle */
  
  
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