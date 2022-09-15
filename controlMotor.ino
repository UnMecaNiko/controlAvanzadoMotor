//    ************      includes
#include <Wire.h>
#include <INA226_WE.h>
#define I2C_ADDRESS 0x40

//    ************      parameters
#define sampleTime 2000 //(usegs)

#define in1HBridge 33   //pin
#define in2HBridge 32   //pin

#define hallSensorA 18    //pin
#define hallSensorB 19    //pin

#define timeMotor 500     //(miliseconds) 
//current sensor
//sda   pin 21
//scl   pin 22

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

  // H-bridge
  pinMode(in1HBridge, OUTPUT);
  pinMode(in2HBridge, OUTPUT);

  //current sensor
  Wire.begin();
  ina226.init();
  ina226.waitUntilConversionCompleted(); //if you comment this line the first data might be zero




  //hall Sensor (configure an interrupt to count pulses per sample)
  attachInterrupt(hallSensorA, isr, RISING);
  attachInterrupt(hallSensorB, isr, RISING);
 
  //start timer
  timer = timerBegin(0, 80, true);  // timer 0, MWDT clock period = 12.5 ns * TIMGn_Tx_WDT_CLK_PRESCALE -> 12.5 ns * 80 -> 1000 ns = 1 us, countUp
  timerAttachInterrupt(timer, &onTimer, true); // edge (not level) triggered 
  timerAlarmWrite(timer, sampleTime, true); // in microseconds
  timerAlarmEnable(timer); // enable

  


}

void loop() {
  //vTaskDelay(portMAX_DELAY); // wait as much as posible ...

  if (samples>=(timeMotor/(sampleTime/1000.00))){

    //change direction of rotation
    digitalWrite(in2HBridge,direction);
    digitalWrite(in1HBridge,!direction);

    direction=!direction;
    samples=0;
  }
  if(flagSample==1)
  {
    ina226.readAndClearFlags();
    current_mA = ina226.getCurrent_mA();

    if (direction==0){
      pulses=pulses*(-1);
    }


    Serial.print(pulses);
    Serial.print(",");
    Serial.println(current_mA);
    flagSample=0;
    pulses=0;
  }

}