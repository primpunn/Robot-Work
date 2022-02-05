#include <SimpleFOC.h>
// software interrupt library
#include <PciManager.h>
#include <PciListenerImp.h>

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(9, 10, 11, 8);
Encoder encoder = Encoder(2, 3, 2048);
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
//void doI1(){encoder1.handleIndex();}

// encoder interrupt init
PciListenerImp listenerA(encoder.pinA, doA);
PciListenerImp listenerB(encoder.pinB, doB);
//PciListenerImp listenerI(encoder1.index_pin, doI1);

//BLDCMotor motor2 =  BLDCMotor( 11);
//BLDCDriver3PWM driver2 = BLDCDriver3PWM(9, 11, 5, 8);
//MagneticSensorI2C sensor2 = MagneticSensorI2C(0x36, 12, 0x0E, 4);

void setup() {

  // initialise encoder hardware
  encoder.init();  
  // interrupt initialization
  PciManager.registerListener(&listenerA);
  PciManager.registerListener(&listenerB);
  //PciManager.registerListener(&listenerI);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  // init driver
  driver.init();
  // link driver
  motor.linkDriver(&driver);

  // set control loop type to be used
  motor.controller = MotionControlType::torque;
  // initialise motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();
  
  // initialise magnetic sensor hardware
  //sensor2.init();
  // link the motor to the sensor
  //motor2.linkSensor(&sensor2);
  // init driver
  //driver2.init();
  // link driver
  //motor2.linkDriver(&driver2);
  // set control loop type to be used
  //motor2.controller = MotionControlType::torque;
  // initialise motor
  //motor2.init();
  // align encoder and start FOC
  //motor2.initFOC();
  
  Serial.println("Steer by wire ready!");
  _delay(1000);
}

void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();
  //motor2.loopFOC();

  // virtual link code
  motor.move( 5*(motor.shaft_angle));
  //motor2.move( 5*(motor1.shaft_angle - motor2.shaft_angle));
  
}
