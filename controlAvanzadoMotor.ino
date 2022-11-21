//    ************      includes
#include <Wire.h>
#include <INA226_WE.h>
#define I2C_ADDRESS1 0x40
#define I2C_ADDRESS2 0x41

//    ************      parameters
#define sampleTime 5000 //(usegs)

//      **Puente H

//Motor 1
#define in1HBridge 15   //pin
#define in2HBridge 2   //pin

//Motor 2
#define in3HBridge 16   //pin
#define in4HBridge 17   //pin

//      **Sensores hall

//Motor 1
#define hallSensorA 18    //pin
//Motor 2
#define hallSensorB 19    //pin

#define timeMotor 1000     //(miliseconds) 
#define VCC       12
#define Ka        1.4048   // N*m/A
//current sensor  
//sda   pin 21
//scl   pin 22
//          **  PWM
// setting PWM properties
const int freq = 2500;
const int PWMChannel1 = 6;
const int PWMChannel2 = 7;
const int PWMChannel3 = 8;
const int PWMChannel4 = 9;

const int resolution = 12;
const int rangePWM = pow(2,resolution);

//    *************     variables

//    interrupt
//these are volatile because the use of ram on interrupts
volatile int pulses1 = 0;
volatile int pulses2 = 0;

//    timer
hw_timer_t * timer = NULL;

volatile int samples=0;
volatile bool flagSample=0;
bool flagStop=0;

//    current sensor
INA226_WE ina226_1 = INA226_WE(I2C_ADDRESS1);
INA226_WE ina226_2 = INA226_WE(I2C_ADDRESS2);

float current_A1 = 0.0;
float current_A2 = 0.0;


// *****    interrupt (isr)
//we use this tipe of variable because we change variables
//that we use in other parts of the code
portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;

// control del motor

// float referenceN = 0.7;    //Newton * meter -Max: 0.84
// float referenceA = referenceN / Ka;
float actualRef1 = 0;
float actualRef2 = 0;

//float period =    10;
//float slopeRef  = 4*referenceA/period;   //[A/s]

int referenceType = 2;   //1.square 2.triangle 3.sin

float sampleTimeSec=sampleTime/1000000.00;
//float stepRef = slopeRef*sampleTimeSec;
float voltMotor1 = 0;
float voltMotor2 = 0;

float errorAct1 = 0;
float errorAct2 = 0;


float q0=   0;
float q1=   1.73875;
float q2=	  -1.59513;
float s0=   -0.9886;

float error1[] ={0,0};
float error2[] ={0,0};

float out1[]   ={0,0};
float out2[]   ={0,0};


float dutyPWM1 = 0;
float dutyPWM2 = 0;


void IRAM_ATTR isr1() {  
  // Interrupt Service Routine
  portENTER_CRITICAL(&synch);
  pulses1 += 1;
  portEXIT_CRITICAL(&synch);
}

void IRAM_ATTR isr2() {  
  // Interrupt Service Routine
  portENTER_CRITICAL(&synch);
  pulses2 += 1;
  portEXIT_CRITICAL(&synch);
}

void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux0);
  samples=samples+1;
  flagSample=1;
  //sampleProcess();
  portEXIT_CRITICAL_ISR(&timerMux0);
}



void updateData(){
  char option=Serial.read();

  if(option=='R'){  //configurar referencia
    //R{reference1},{reference2}
    actualRef1 =Serial.parseFloat() ;
    actualRef2 =Serial.parseFloat() ;
  }
  if(option=='P'){
    flagStop=!flagSample;
  }
  Serial.readString(); //clear buffer
}

void sentData(){
  //{pulses1},{pulses2}
  if (samples%10==0){
    Serial.print(pulses1);
    Serial.print(",");
    Serial.print(pulses2);
    Serial.print("\n");
  }
}

void sampleProcess(){
  //Read sensor
  ina226_1.readAndClearFlags();
  ina226_2.readAndClearFlags();
  current_A1 = ina226_1.getCurrent_mA()/1000.0000;
  current_A2 = ina226_2.getCurrent_mA()/1000.0000;
  //Calculate error
  errorAct1=actualRef1-current_A1;
  errorAct2=actualRef2-current_A2;

  //Diference Ecuation
  voltMotor1=q0*errorAct1+q1*error1[0]+q2*error1[1]-(s0-1)*out1[0]+s0*out1[1];
  voltMotor2=q0*errorAct2+q1*error2[0]+q2*error2[1]-(s0-1)*out2[0]+s0*out2[1];

  //Saturation
  if (voltMotor1>VCC) voltMotor1=VCC;
  if (voltMotor1<-VCC) voltMotor1=-VCC;
  if (voltMotor2>VCC) voltMotor2=VCC;
  if (voltMotor2<-VCC) voltMotor2=-VCC;

  error1[1]=error1[0];
  error1[0]=errorAct1;
  out1[1]=out1[0];
  out1[0]=voltMotor1;

  error2[1]=error2[0];
  error2[0]=errorAct2;
  out2[1]=out2[0];
  out2[0]=voltMotor2;

  dutyPWM1 = abs(voltMotor1)*rangePWM/VCC;
  dutyPWM2 = abs(voltMotor2)*rangePWM/VCC;


  if(voltMotor1>0){
    //when PWMChannel2 = 0 positive current
    ledcWrite(PWMChannel2,0);
    ledcWrite(PWMChannel1,dutyPWM1);
  }
  else{
    ledcWrite(PWMChannel2,dutyPWM1);
    ledcWrite(PWMChannel1,0);
  }

  if(voltMotor2>0){
    //when PWMChannel2 = 0 positive current
    ledcWrite(PWMChannel4,0);
    ledcWrite(PWMChannel3,dutyPWM1);
  }
  else{
    ledcWrite(PWMChannel4,dutyPWM1);
    ledcWrite(PWMChannel3,0);
  }
  //Sent information
  sentData();
  //Reset variables
  pulses1=0;
  pulses2=0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("start setup");
  //current sensor
  Wire.begin();
  ina226_1.init();
  ina226_2.init();

  //if you comment this line the first data might be zero
  ina226_1.waitUntilConversionCompleted(); 
  ina226_2.waitUntilConversionCompleted(); 
  //hall Sensor (configure an interrupt to count pulses1 per sample)
  attachInterrupt(hallSensorA, isr1, RISING);
  attachInterrupt(hallSensorB, isr2, RISING);
  //start timer
  timer = timerBegin(0, 80, true);  
  // timer 0, MWDT clock period = 
  // 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered 
  timerAlarmWrite(timer, sampleTime, true); // in microseconds
  timerAlarmEnable(timer); // enable timer
  //            *******  pwm
  // configure LED PWM functionalitites
  ledcSetup(PWMChannel1, freq, resolution);
  ledcSetup(PWMChannel2, freq, resolution);
  ledcSetup(PWMChannel3, freq, resolution);
  ledcSetup(PWMChannel4, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(in1HBridge, PWMChannel1);
  ledcAttachPin(in2HBridge, PWMChannel2);
  ledcAttachPin(in3HBridge, PWMChannel3);
  ledcAttachPin(in4HBridge, PWMChannel4);
  Serial.flush();
  Serial.println("end setup"); 
}

void loop() {

  if(Serial.available() > 0) {
    updateData();
  }  
  if(flagSample && !flagStop) { //script when a sample is made
    sampleProcess();
    flagSample=0;
  }

}