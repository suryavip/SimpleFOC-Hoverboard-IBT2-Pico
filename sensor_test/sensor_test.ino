#include <SimpleFOC.h>

// Wiring
const pin_size_t MAGNETIC_SENSOR_SDA_PIN = 0;
const pin_size_t MAGNETIC_SENSOR_SCL_PIN = 1;

const uint8_t MAGNETIC_SENSOR_ADDRESS = 0x36;
const uint8_t MAGNETIC_SENSOR_RESOLUTION = 12;
const uint8_t MAGNETIC_SENSOR_ANGLE_REGISTER_MSB = 0x0C;  // or 0x0E (appear to have deadzone)
const uint8_t MAGNETIC_SENSOR_BITS_USED_MSB = 4;

MagneticSensorI2C sensor = MagneticSensorI2C(
                             MAGNETIC_SENSOR_ADDRESS,
                             MAGNETIC_SENSOR_RESOLUTION,
                             MAGNETIC_SENSOR_ANGLE_REGISTER_MSB,
                             MAGNETIC_SENSOR_BITS_USED_MSB);


void setup() {
  Serial.begin(115200);

  Wire.setSDA(MAGNETIC_SENSOR_SDA_PIN);
  Wire.setSCL(MAGNETIC_SENSOR_SCL_PIN);
  Wire.setClock(400000);
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
