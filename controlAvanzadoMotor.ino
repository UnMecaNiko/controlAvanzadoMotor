//    ************      includes
#include <Wire.h>
#include <INA226_WE.h>
#define I2C_ADDRESS 0x40

//    ************      parameters
#define sampleTime 5000 //(usegs)

#define in1HBridge 4   //pin
#define in2HBridge 2   //pin

#define hallSensorA 18    //pin
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
const int PWMChannelA = 6;
const int PWMChannelB = 7;
const int resolution = 12;
const int rangePWM = pow(2,resolution);

//    *************     variables

//    interrupt
//these are volatile because the use of ram on interrupts
volatile int pulses = 0;

//    timer
hw_timer_t * timer = NULL;

volatile int samples=0;
volatile bool flagSample=0;

//    current sensor
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);
float current_A = 0.0;

// *****    interrupt (isr)
//we use this tipe of variable because we change variables
//that we use in other parts of the code
portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;

// control del motor

float referenceN = 0.7;    //Newton * meter -Max: 0.84
float referenceA = referenceN / Ka;
float actualRef = 0;
float period =    10;
float slopeRef  = 4*referenceA/period;   //[A/s]

int referenceType = 2;   //1.square 2.triangle 3.sin

float sampleTimeSec=sampleTime/1000000.00;
float stepRef = slopeRef*sampleTimeSec;
float voltMotor = 0;
float errorAct = 0;

float q0=   0;
float q1=   5.4315;
float q2=	  -5.069;
float s0=   -0.9969;

float error[] ={0,0};
float out[]   ={0,0};

float dutyPWM = 0;

void IRAM_ATTR isr() {  
  // Interrupt Service Routine
  portENTER_CRITICAL(&synch);
  pulses += 1;
  portEXIT_CRITICAL(&synch);
}

void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux0);
  samples=samples+1;
  flagSample=1;
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void updateStepRef(){
  slopeRef  = 4*referenceA/period;
  stepRef   = slopeRef*sampleTimeSec;
}

void updateData(){
  char option=Serial.read();

  if(option=='C'){ //Configurar control
    //C{q0},{q1},{q2},{s0}
    q0=Serial.parseFloat();
    q1=Serial.parseFloat();
    q2=Serial.parseFloat();
    s0=Serial.parseFloat();
  }
  if(option=='R'){  //configurar referencia
    //R{referenceType},{referenceN},{period}
    referenceType =int( Serial.parseFloat() );
    referenceN    =Serial.parseFloat();
    period        =Serial.parseFloat();

    referenceA = referenceN / Ka;
    if(referenceType==2) updateStepRef();
  }
  Serial.readString(); //clear buffer
}

void sentData(){
  //{actualRef},{out},{control},{pulses}
  Serial.print(actualRef*Ka,3);
  Serial.print(",");
  Serial.print(current_A*Ka,3);
  Serial.print(",");
  Serial.print(voltMotor,3);
  Serial.print(",");
  Serial.print(pulses);
  Serial.print("\n");
}

void sampleProcess(){
  switch (referenceType){
    case 1: //Step Reference
      if( ( samples%int(period/(sampleTimeSec*2) ) ) ==0 ){
        actualRef=referenceA*-1;
        referenceA=referenceA*-1;
      }
      break;
    case 2: //Triangle Reference
      if(actualRef<referenceA) actualRef+=stepRef;
      if(actualRef>referenceA) actualRef-=stepRef;

      if(abs(actualRef)>abs(referenceA)-0.001) referenceA=referenceA*-1;
      break;
    case 3: //Sin Reference
      actualRef=referenceA*sin(6.2832*sampleTimeSec*samples/period);
      break;
    default:
      Serial.println("errorTypeReference");
      break;
  }
  //Read sensor
  ina226.readAndClearFlags();
  current_A = ina226.getCurrent_mA()/1000.0000;
  //Calculate error
  errorAct=actualRef-current_A;
  //Diference Ecuation
  voltMotor=q0*errorAct+q1*error[0]+q2*error[1]-(s0-1)*out[0]+s0*out[1];
  //Saturation
  if (voltMotor>VCC) voltMotor=VCC;
  if (voltMotor<-VCC) voltMotor=-VCC;

  error[1]=error[0];
  error[0]=errorAct;
  out[1]=out[0];
  out[0]=voltMotor;

  dutyPWM = abs(voltMotor)*rangePWM/VCC;

  if(voltMotor>0){
    //when PWMChannelB = 0 positive current
    ledcWrite(PWMChannelB,0);
    ledcWrite(PWMChannelA,dutyPWM);
  }
  else{
    ledcWrite(PWMChannelB,dutyPWM);
    ledcWrite(PWMChannelA,0);
  }

  //Sent information
  sentData();
  //Reset variables
  pulses=0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("start setup");
  //current sensor
  Wire.begin();
  ina226.init();
  ina226.waitUntilConversionCompleted(); //if you comment this line the first data might be zero
  //hall Sensor (configure an interrupt to count pulses per sample)
  attachInterrupt(hallSensorA, isr, RISING);
  attachInterrupt(hallSensorB, isr, RISING);
  //start timer
  timer = timerBegin(0, 80, true);  
  // timer 0, MWDT clock period = 
  // 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered 
  timerAlarmWrite(timer, sampleTime, true); // in microseconds
  timerAlarmEnable(timer); // enable timer
  //            **    pwm
  // configure LED PWM functionalitites
  ledcSetup(PWMChannelA, freq, resolution);
  ledcSetup(PWMChannelB, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(in1HBridge, PWMChannelA);
  ledcAttachPin(in2HBridge, PWMChannelB);
  Serial.flush();
  Serial.println("end setup"); 
}

void loop() {

  if(Serial.available() > 0) {
    updateData();
  }  
  if(flagSample==1) { //script when a sample is made
    sampleProcess();
    flagSample=0;
  }

}