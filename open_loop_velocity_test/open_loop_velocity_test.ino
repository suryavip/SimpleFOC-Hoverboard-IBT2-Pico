/*
  Use serial monitor (or something simillar).
  
  To change the voltage limit, enter command: L<voltage_limit>.
  Example: L1
  will change the voltage limit to 1V.
  
  To change the velocity target, enter command: T<velocity>.
  Example: T1
  will target the velocity to 1 rad/sec.
  
  Use command P to print current target and voltage limit.
*/

#include <SimpleFOC.h>

// Wiring:
const pin_size_t PA_EN_PIN = 12;  // to R_EN (1st IBT-2 module)
const pin_size_t PA_PIN = 13;     // to RPWM (1st IBT-2 module)
const pin_size_t PB_EN_PIN = 14;  // to L_EN (1st IBT-2 module)
const pin_size_t PB_PIN = 15;     // to LPWM (1st IBT-2 module)
const pin_size_t PC_EN_PIN = 16;  // to R_EN (2nd IBT-2 module)
const pin_size_t PC_PIN = 17;     // to RPWM (2nd IBT-2 module)
// Phase A will be on M+ terminal on 1st IBT-2 module.
// Phase B will be on M- terminal on 1st IBT-2 module.
// Phase C will be on M+ terminal on 2nd IBT-2 module.

const int MOTOR_POLE_PAIRS = 15;

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

BLDCDriver3PWM driver = BLDCDriver3PWM(
  PA_PIN,
  PB_PIN,
  PC_PIN,
  PA_EN_PIN,
  PB_EN_PIN,
  PC_EN_PIN);

Commander command = Commander(Serial);

void doTarget(char* cmd) {
  command.scalar(&motor.target, cmd);
  echoBack(cmd);
}

void doLimit(char* cmd) {
  command.scalar(&motor.voltage_limit, cmd);
  echoBack(cmd);
}

void echoBack(char* cmd) {
  Serial.print("Target: ");
  Serial.print(motor.target);
  Serial.print("; Limit: ");
  Serial.println(motor.voltage_limit);
}

void setup() {
  driver.pwm_frequency = 3000;
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 3;
  driver.init();

  motor.linkDriver(&driver);
  motor.target = 0;
  motor.voltage_limit = 0;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();

  // add target command T
  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");
  command.add('P', echoBack, "print state");

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  motor.move();
  command.run();
}
