#include <SimpleFOC.h>

pin_size_t ENCODER_SDA_PIN = 0;
pin_size_t ENCODER_SCL_PIN = 1;

const uint8_t ENCODER_ADDRESS = 0x36;
const uint8_t ENCODER_RESOLUTION = 12;
const uint8_t ENCODER_ANGLE_REGISTER_MSB = 0x0C;  // or 0x0E (appear to have deadzone)
const uint8_t ENCODER_BITS_USED_MSB = 4;

MagneticSensorI2C sensor = MagneticSensorI2C(
  ENCODER_ADDRESS,
  ENCODER_RESOLUTION,
  ENCODER_ANGLE_REGISTER_MSB,
  ENCODER_BITS_USED_MSB);


void setup() {
  // monitoring port
  Serial.begin(115200);

  Wire.setSDA(ENCODER_SDA_PIN);
  Wire.setSCL(ENCODER_SCL_PIN);
  // configure i2C
  // Wire.setClock(400000);
  // initialise magnetic sensor hardware
  sensor.init();

  Serial.println("Sensor ready");
  _delay(1000);
}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and
  // has to be called before getAngle nad getVelocity
  sensor.update();

  // display the angle and the angular velocity to the terminal
  Serial.print(sensor.getAngle());
  Serial.print("\t");
  Serial.println(sensor.getVelocity());
}
