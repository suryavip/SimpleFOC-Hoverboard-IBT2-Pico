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
const pin_size_t MAGNETIC_SENSOR_SDA_PIN = 0;
const pin_size_t MAGNETIC_SENSOR_SCL_PIN = 1;

const pin_size_t PA_EN_PIN = 12;  // to R_EN (1st IBT-2 module)
const pin_size_t PA_PIN = 13;     // to RPWM (1st IBT-2 module)
const pin_size_t PB_EN_PIN = 14;  // to L_EN (1st IBT-2 module)
const pin_size_t PB_PIN = 15;     // to LPWM (1st IBT-2 module)
const pin_size_t PC_EN_PIN = 16;  // to R_EN (2nd IBT-2 module)
const pin_size_t PC_PIN = 17;     // to RPWM (2nd IBT-2 module)
// Phase A will be on M+ terminal on 1st IBT-2 module.
// Phase B will be on M- terminal on 1st IBT-2 module.
// Phase C will be on M+ terminal on 2nd IBT-2 module.

const uint8_t MAGNETIC_SENSOR_ADDRESS = 0x36;
const uint8_t MAGNETIC_SENSOR_RESOLUTION = 12;
const uint8_t MAGNETIC_SENSOR_ANGLE_REGISTER_MSB = 0x0C;  // or 0x0E (appear to have deadzone)
const uint8_t MAGNETIC_SENSOR_BITS_USED_MSB = 4;

const int MOTOR_POLE_PAIRS = 15;

MagneticSensorI2C sensor = MagneticSensorI2C(
                             MAGNETIC_SENSOR_ADDRESS,
                             MAGNETIC_SENSOR_RESOLUTION,
                             MAGNETIC_SENSOR_ANGLE_REGISTER_MSB,
                             MAGNETIC_SENSOR_BITS_USED_MSB);

BLDCDriver3PWM driver = BLDCDriver3PWM(
                          PA_PIN,
                          PB_PIN,
                          PC_PIN,
                          PA_EN_PIN,
                          PB_EN_PIN,
                          PC_EN_PIN);

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);

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
  Wire.setSDA(MAGNETIC_SENSOR_SDA_PIN);
  Wire.setSCL(MAGNETIC_SENSOR_SCL_PIN);
  Wire.setClock(400000);
  sensor.init();

  driver.pwm_frequency = 3000;
  driver.voltage_power_supply = 12;
  driver.voltage_limit = 3;
  driver.init();

  motor.linkSensor(&sensor);
  motor.linkDriver(&driver);
  motor.target = 0;
  motor.voltage_limit = 0;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity_openloop;
  motor.init();
  motor.initFOC();

  // add target command T
  command.add('T', doTarget);
  command.add('L', doLimit);
  command.add('P', echoBack);

  Serial.begin(115200);
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move();

  command.run();
}

void loop1() {
  unsigned long lastEchoSensor = millis();

  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());

  while (millis() < lastEchoSensor + 1000) {
  }
}
