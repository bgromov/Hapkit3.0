/*
 * Author: Boris Gromov <boris@idsia.ch>
 *
 * May 11, 2018
 *
 * Based on the original code from Introduction to Haptics -- an online
 * course from Stanford University.
 *
 * Original copyright:
 * --------------------------------------------------------------------------
 * Tania Morimoto and Allison Okamura, Stanford University
 * 11.16.13 - 10.16.14
 * Code to test basic Hapkit functionality (sensing and force output)
 * --------------------------------------------------------------------------
 */

#include "hapkit3.h"

static float g_UpdateRate = 2000.0f;

// #define LOOP_RATE (2000.0) // 0.5 kHz
#define LOOP_PERIOD (1.0 / g_UpdateRate) // [sec]

#if defined(__AVR__)
HapkitSensor::HapkitSensor(uint8_t pin, int16_t flip_threshold)
#elif defined(__MBED__)
HapkitSensor::HapkitSensor(PinName pin, int16_t flip_threshold)
: sensor(pin)
#endif
{
  sensor_pin = pin;
  flipThresh = flip_threshold;
#if defined(__AVR__)
  pinMode(pin, INPUT); // set MR sensor pin to be an input
#endif

  this->reset();
}

void HapkitSensor::reset()
{
  updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
  rawPos = analogRead(sensor_pin);         // current raw reading from MR sensor
  lastRawPos = analogRead(sensor_pin);     // last raw reading from MR sensor
  lastLastRawPos = 0; // last last raw reading from MR sensor
  flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
  tempOffset = 0;
  rawDiff = 0;
  lastRawDiff = 0;
  rawOffset = 0;
  lastRawOffset = 0;
  // flipThresh = 700;     // threshold to determine whether or not a flip over the 180 degree mark occurred
  flipped = false;

  zeroPos = 0;

#if defined(__AVR__)
  // timer_tck.detachInterrupt();
  // timer_tck.attachInterrupt(&HapkitSensor::readSensor, 2000);
#elif defined(__MBED__)
  timer_tck.detach();
  timer_tck.attach_us(callback(this, &HapkitSensor::readSensor), 1000000 / g_UpdateRate);
#endif
}

void HapkitSensor::readSensor()
{
//  __disable_irq();
  // Get voltage output by MR sensor
  rawPos = analogRead(sensor_pin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);

  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;

  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    if(rawOffset > flipThresh) { // check to see if the data was good and the most current offset is above the threshold
      updatedPos = rawPos + flipNumber*rawOffset; // update the pos value to account for flips over 180deg using the most current offset
      tempOffset = rawOffset;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPos = rawPos + flipNumber*lastRawOffset;  // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffset = lastRawOffset;
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    updatedPos = rawPos + flipNumber*tempOffset; // need to update pos based on what most recent offset is
    flipped = false;
  }
//  __enable_irq();
//  printf("%d\r\n", updatedPos);
}

HapkitMotor::HapkitMotor(uint8_t motornum)
{
  #if defined(__AVR__)
    motor_id = motornum;
    // Adafruit Motor Shield V1
    // switch(motor_id)
    // {
    //   case 1:
    //   case 2:
    //     motor = new AF_DCMotor(motor_id, MOTOR12_64KHZ);
    //     break;
    //   case 3:
    //   case 4:
    //     motor = new AF_DCMotor(motor_id, MOTOR34_64KHZ);
    //     break;
    // }
  // Adafruit Motor Shield V2
  AFMS = new Adafruit_MotorShield();
  motor = AFMS->getMotor(motor_id);

  AFMS->begin();

  TWBR = ((F_CPU /400000l) - 16) / 2; // Change the i2c clock to 400KHz

  // Make sure the motor is not running
  motor->setSpeed(0);
  motor->run(FORWARD);

  #elif defined(__MBED__)

    motor_id = motornum - 1;

    motor = new L6206(D2, A4, D5, D4, A0, A1);
    if (motor->init(NULL) != COMPONENT_OK) {
        exit(EXIT_FAILURE);
    }

    switch(motor_id)
    {
      case 0:
      case 1:
        /* Set PWM Frequency of bridge inputs to 20 kHz */
        motor->set_bridge_input_pwm_freq(motor_id, 60000);
        motor->set_speed(motor_id, 0);
        /* start motor */
        motor->run(motor_id, (BDCMotor::direction_t)FORWARD);
        break;
    }
  #endif
}

#if defined(__AVR__)
void HapkitMotor::run(uint8_t direction)
#elif defined(__MBED__)
void HapkitMotor::run(motorDir_t direction)
#endif
{
  if (motor)
  {
  #if defined(__AVR__)
    motor->run(direction);
  #elif defined(__MBED__)
    motor->run(motor_id, (BDCMotor::direction_t)direction);
  #endif
  }
}

void HapkitMotor::setSpeed(float duty)
{
  uint8_t duty_u8;

  // Clamp the duty cycle
  if (duty > 1.0f)
  {
    duty = 1.0f;
  }

  if (duty < 0.0f)
  {
    duty = 0.0f;
  }

  #if defined(__AVR__)
    duty_u8 = duty * 255;
    motor->setSpeed(duty_u8);
  #elif defined(__MBED__)
    duty_u8 = duty * 255;
    motor->set_speed(motor_id, duty_u8);
//    printf("Motor speed: %d\r\n", duty_u8);
  #endif
}

void HapkitMotor::stop()
{
  #if defined(__AVR__)
    motor->run(RELEASE);
  #elif defined(__MBED__)
    motor->hard_hiz(motor_id);
  #endif
}

#if defined(__AVR__)
Hapkit::Hapkit(const hapkit_kinematics_t kin, uint8_t motornum, uint8_t sensor_pin)
#elif defined(__MBED__)
Hapkit::Hapkit(const hapkit_kinematics_t kin, uint8_t motornum, PinName sensor_pin)
#endif
: motornum(motornum), sensor_pin(sensor_pin),
  motor(motornum), sensor(sensor_pin), effects(NULL), effects_len(0),
  duty_th(0.10f),
  // pos_filter(500.0, g_UpdateRate),
  vel_filter(5.0, g_UpdateRate),
  acc_filter(3.0, g_UpdateRate)
{
    // Kinematics variables
    rp = kin.pulley_radius;
    rh = kin.handle_radius;
    rs = kin.sector_radius;
    sec_span = kin.sector_span;

    this->reset();
}

void Hapkit::startLoop()
{
  #if defined(__AVR__)
  #elif defined(__MBED__)
  force_tck.attach_us(callback(this, &Hapkit::update), 1000000 / g_UpdateRate); // 2 kHz
  #endif
}

void Hapkit::stopLoop()
{
  #if defined(__AVR__)
  #elif defined(__MBED__)
  force_tck.detach();
  #endif
}

void Hapkit::reset()
{
     // Position tracking variables
    minPos = INT_MAX;
    maxPos = INT_MIN;
    zeroPos = 0;

    sec_K = 0.0;
    alpha_h = 0.0;
    xh = 0.0;

    // Force output variables
    // force = 0.0;
    Tp = 0.0;
    duty = 0.0;
    output = 0.0;

    lastXh = 0.0;
    vh = 0.0;
    lastVh = 0.0;
    lastLastVh = 0.0;

    ah = 0.0;
    lastAh = 0.0;
    lastLastAh = 0.0;


    calibrated = false;
    calibration_counter = 0;

    vel_filter.reset();
    acc_filter.reset();
}

void Hapkit::configure(float duty_threshold)
{
    duty_th = duty_threshold;
}

//*************************************************************
//*** Automatic Calibration ***********************************
//*************************************************************
void Hapkit::calibrate()
{
  float e = 0.0;
  float e_last = 0.0;
  float e_i = 0.0;
  float e_d = 0.0;

  this->stopLoop(); // Make sure the haptic loop is not running (only on STM32)

  // Slowly move to one end
  motor.setSpeed(0.3);

  while(!calibrated)
  {
    // Read sensor
    int16_t cur_pos = sensor.getPosition();

    // Keep track of the maximum and minimum sensor readings
    minPos = min(cur_pos, minPos);
    maxPos = max(cur_pos, maxPos);
    // Calculate the middle position
    zeroPos = maxPos - (abs(minPos) + abs(maxPos)) / 2.0;

    #if defined(__AVR__)
    delay(1);
    #elif defined(__MBED__)
    wait_us(1500);
    #endif

//    printf("%d\r\n", cur_pos);

    calibration_counter++;

    // Move to one mechanical limit
    if (calibration_counter < 700)
    {
      motor.run(BACKWARD);
    }
    // Move to another mechanical limit
    else if (calibration_counter < 1400)
    {
      motor.run(FORWARD);
    }
    // Move to calculated zero position
    else if (calibration_counter < 2100)
    {
        // A kind of PID controller
        e = (cur_pos - zeroPos) / (float)(abs(minPos) + abs(maxPos));
        e_d = e_last - e;
        e_i += e;
        e_last = e;
        float s = (e * 2.5f + e_i * 0.03f + e_d * 0.20f);

        if (s > 0.0) motor.run(BACKWARD);
        else motor.run(FORWARD);

//        printf("Current speed: %1.5f\r\n", fabs(s));
//        printf("Position error: %1.5f\r\n", e);

        motor.setSpeed(fabs(s));
    }
    else
    {
        // Ouput calibration results
        printf("Calibrated\r\n");

        printf("Min: %d\r\n", minPos);
        printf("Max: %d\r\n", maxPos);
        printf("Zero: %d\r\n", zeroPos);

        printf("Final position error: %1.5f\r\n", e);

        // Set the zero
        sensor.setZero(zeroPos);

        // Calculate the scaling factor for the sector,
        // (transforms sensor readings from ADC units to radians)
        sec_K = sec_span / (fabs((float)minPos) + fabs((float)maxPos));

        calibrated = true;
        calibration_counter = 0;
//        motor.setSpeed(0.0);
        motor.stop();

        this->startLoop();
        return;
    }
  }
}

void Hapkit::update()
{
    alpha_h = sec_K * sensor.getPosition(); // sector current rotation angle
    xh = rh * alpha_h;  // linear position of the handle

    // Calculate and filter velocity
    vh = vel_filter.filter((xh-lastXh) / LOOP_PERIOD); // First derivative
    // Calculate and filter acceleration
    ah = acc_filter.filter((vh-lastVh) / LOOP_PERIOD); // Second derivative

    // Remember values until the next iteration
    lastXh = xh;
    lastLastVh = lastVh;
    lastVh = vh;
    lastLastAh = lastAh;
    lastAh = ah;
}

float Hapkit::calcEffectsForce()
{
    if (this->effects && this->effects_len)
    {
        double k_spring = 100.0; // define the stiffness of a virtual spring in N/m
        double k_damper = 0.05;//sqrt(k_spring * 0.050) * 2; //[Ns/m]
        float force = -k_spring * xh; // virtual spring

        // Interate over the array of haptic effects
        for (int i = 0; i < this->effects_len; i++) {
            // Check if the handle is in the range of current effect
            double dx = xh - this->effects[i].position;
            if (fabs(dx) < this->effects[i].width / 2.0) {
                // If so, generate the force accordingly to the constants
                force = -this->effects[i].k_damper * vh - this->effects[i].k_spring * dx;
                break;
            }
            //  else {
            //   force = 0.0;
            // }
        }
        return force;
    }
}

void Hapkit::setForce(float force)
{
  // Calculate the required motor torque
  Tp = (force * rh / rs) * rp;

  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(fabs(Tp) / 0.0146); // the magic number here is the motor constant

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {
    duty = 1.0;
  } else if (duty < 0) {
    duty = 0.0;
  }

  // If the duty cycle is below the threshold cut off the power
  if (duty < duty_th) {
    duty = 0.0;
    motor.stop();
  }
  else
  {
    // Determine correct direction for motor torque
    if(force < 0) {
      motor.run(BACKWARD);
    } else {
      motor.run(FORWARD);
    }
  }

  // Set the motor speed/duty
  motor.setSpeed(duty);
}

float Hapkit::getUpdateRate()
{
  return g_UpdateRate;
}

void Hapkit::setUpdateRate(float rate)
{
  if (rate > 0.0f)
  {
    g_UpdateRate = rate;
  }
  else
  {
    printf("Invalid update rate: %3.2f. Ignoring...\n", rate);
  }
}
