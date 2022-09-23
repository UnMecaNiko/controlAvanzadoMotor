//    ************      includes
#include <Wire.h>
#include <INA226_WE.h>
#define I2C_ADDRESS 0x40

//    ************      parameters
#define sampleTime 3000 //(usegs)

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
const int PWMChannel = 6;
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
float current_mA = 0.0;

// *****    interrupt (isr)
//we use this tipe of variable because we change variables
//that we use in other parts of the code
portMUX_TYPE synch = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED;


// control del motor

float reference = 300;
float actualRef = 0;
float slopeRef  = 300;
float stepRef = slopeRef*sampleTime/1000000;
float voltMotor = 0;
float errorAct     = 0;

float q0=   -1256.3;
float q1=   2484.8;
float q2=	  -1224.6;
float s0=   0.9932;

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
  ledcSetup(PWMChannel, freq, resolution);
  ledcSetup(PWMChannel+1, freq, resolution);
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(in1HBridge, PWMChannel);
  ledcAttachPin(in2HBridge, PWMChannel+1);
  Serial.println("end setup"); 
}

void loop() {

  if(Serial.available() > 0) {
    String recievedData = Serial.readString();
    reference = recievedData.toFloat();
    
  }

  if(flagSample==1)
  {
    if(actualRef<reference) actualRef+=stepRef;
    if(actualRef>reference) actualRef-=stepRef;

    ina226.readAndClearFlags();
    current_mA = ina226.getCurrent_mA();

    errorAct=actualRef-current_mA;

    voltMotor=q0*errorAct+q1*error[0]+q2*error[1]+(s0+1)*out[0]-s0*out[1];

    if (voltMotor>VCC) voltMotor=VCC;
    if (voltMotor<-VCC) voltMotor=-VCC;

    error[1]=error[0];
	  error[0]=errorAct;

	  out[1]=out[0];
	  out[0]=voltMotor;

    dutyPWM = abs(voltMotor)*rangePWM/VCC;

    if(voltMotor>0){
      ledcWrite(PWMChannel,dutyPWM);
      ledcWrite(PWMChannel+1,0);
    }
    else{
      ledcWrite(PWMChannel,0);
      ledcWrite(PWMChannel+1,dutyPWM);
    }
    
    if (direction==0) pulses=pulses*(-1); 
    Serial.print(actualRef);
    Serial.print("\t");
    Serial.print(voltMotor);
    Serial.print("\t");
    Serial.println(current_mA);
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