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
//current sensor  
//sda   pin 21
//scl   pin 22
//          **  PWM
// setting PWM properties
const int freq = 2500;
const int PWMChannelA = 6;
const int PWMChannelB = 7;
const int resolution = 12;
const int rangePWM = pow(2,12);

//    *************     variables

//    interrupt
//these are volatile because the use of ram on interrupts
volatile int pulses = 0;

//    timer
hw_timer_t * timer = NULL;

volatile int samples=0;
volatile bool flagSample=0;

bool direction = false; 

//    current sensor
INA226_WE ina226 = INA226_WE(I2C_ADDRESS);
float current_A = 0.0;

// *****    interrupt (isr)
//we use this tipe of variable because we change variables
//that we use in other parts of the code
portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;


// control del motor

float reference = 0.5;    //Amperes
float actualRef = 0;
float slopeRef  = 0.2;
float stepRef = slopeRef*sampleTime/1000000;
float voltMotor = 0;
float errorAct     = 0;

float q0=   2.12;
float q1=   0.8838;
float q2=	  0;
float s0=   0 ;

float error[] ={0,0};
float out[]   ={0,0};

float dutyPWM = 0;


void IRAM_ATTR isr() {  
  // Interrupt Service Routine
  portENTER_CRITICAL(&synch);
  //
  pulses += 1;
  portEXIT_CRITICAL(&synch);
}

void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux0);

  samples=samples+1;
  flagSample=1;
  //Serial.println(pulses);
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void setup() {
  Serial.begin(115200);
  Serial.println("setup");
  //current sensor
  Wire.begin();
  ina226.init();
  ina226.waitUntilConversionCompleted(); //if you comment this line the first data might be zero

  //hall Sensor (configure an interrupt to count pulses per sample)
  attachInterrupt(hallSensorA, isr, RISING);
  attachInterrupt(hallSensorB, isr, RISING);
 
  //start timer
  timer = timerBegin(0, 80, true);  
  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered 
  timerAlarmWrite(timer, sampleTime, true); // in microseconds
  timerAlarmEnable(timer); // enable

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
    String recievedData = Serial.readString();
    reference = recievedData.toFloat();
  }

  if(flagSample==1)
  { //script when a sample is made
    if(actualRef<reference) actualRef+=stepRef;
    if(actualRef>reference) actualRef-=stepRef;

    ina226.readAndClearFlags();
    current_A = ina226.getCurrent_mA()/1000.0000;

    errorAct=actualRef-current_A;

    voltMotor=q0*errorAct+q1*error[0]+q2*error[1]-(s0-1)*out[0]+s0*out[1];

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
    if (direction==0) pulses=pulses*(-1); 
    Serial.print(actualRef,3);
    Serial.print("\t");
    Serial.print(voltMotor,3);
    Serial.print("\t");
    Serial.println(current_A,3);
    flagSample=0;
    pulses=0;
  }

  // if (samples>=(timeMotor/(sampleTime/1000.00))){
  //   //change direction of rotation

  //   if(direction){
  //     ledcWrite(PWMChannel,2000);
  //     ledcWrite(PWMChannel+1,0);
  //   }
  //   else{
  //     ledcWrite(PWMChannel,0);
  //     ledcWrite(PWMChannel+1,1000);
  //   }
    
  //   direction=!direction;
  //   samples=0;
  // }
}