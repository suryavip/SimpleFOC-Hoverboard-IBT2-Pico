/*
  DON'T ATTACH THE MOTOR YET!
  Just test the output voltage for each phases with multimeter.
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

BLDCDriver3PWM driver = BLDCDriver3PWM(
                          PA_PIN,
                          PB_PIN,
                          PC_PIN,
                          PA_EN_PIN,
                          PB_EN_PIN,
                          PC_EN_PIN);

void setup() {

  // PWM frequency to be used [Hz]
  driver.pwm_frequency = 3000;

  // Power supply voltage [V]
  driver.voltage_power_supply = 12;

  // Max DC voltage allowed [V]
  driver.voltage_limit = 3;

  // Driver init
  driver.init();

  // Enable driver
  driver.enable();

  _delay(1000);
}

void loop() {
  // Check voltages (relative to supply ground) with multimeter on each phases terminal.
  // If all the voltage readings matched with what value is set here, then it's safe to continue to open-loop test.
  // driver.setPwm(1, 3, 2); means phase A terminal should output 1 volt, phase B should output 3 volt, and phase C should output 2 volt.
  driver.setPwm(1, 3, 2);
}
