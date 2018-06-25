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

#ifndef _hapkit3_h_
#define _hapkit3_h_

#include <math.h>
#include <limits.h>

// Define standard constants and functions (some versions of libc do not include these)
#ifndef M_PI
    #define M_PI                      (3.14159265358979323846f)
#endif

#ifndef max
    #define max(a, b)                 ((a > b ? a : b))
#endif
#ifndef min
    #define min(a, b)                 ((a < b ? a : b))
#endif

// Build for Arduino
#if defined(__AVR__)
  #include <Arduino.h>
  // Build for Arduino with Adafruit Motor Shield v1.0
  // #include <AFMotor.h>
  // Build for Arduino with Adafruit Motor Shield v2.0
  #include <Adafruit_MotorShield.h>
// Build for Mbed on STM32
#elif defined(__MBED__)
  // Build for Mbed with X Nucleo IHM04A1
  #include <L6206.h>
  #include "analogin_dma.h"
#endif

// One-pole recursive low-pass filter
class LowPassFilter
{
  float a0, b1, y1;
public:
  LowPassFilter()
  {
    y1 = 0.0;
    b1 = 0.0;
    a0 = 1.0;
  }
  LowPassFilter(float band_freq, float sampling_freq)
  {
    setFc(band_freq / sampling_freq);
  }
  LowPassFilter(float fc)
  {
    setFc(fc);
  }

  inline void reset()
  {
    y1 = 0.0;
  }
  inline void setFc(float fc)
  {
    y1 = 0.0;
    b1 = exp(-2.0 * M_PI * fc);
    a0 = 1.0 - b1;
    printf("b1: %1.5f, a0: %1.5f\n", b1, a0);
  }
  inline float filter(float x)
  {
    return y1 = x * a0 + y1 * b1;
  }
};

// Interface to NXP KMA210 sensor
class HapkitSensor
{
  private:
    int16_t updatedPos;     // keeps track of the latest updated value of the MR sensor reading
    int16_t rawPos;         // current raw reading from MR sensor
    int16_t lastRawPos;     // last raw reading from MR sensor
    int16_t lastLastRawPos; // last last raw reading from MR sensor
    int16_t flipNumber;     // keeps track of the number of flips over the 180deg mark
    int16_t tempOffset;
    int16_t rawDiff;
    int16_t lastRawDiff;
    int16_t rawOffset;
    int16_t lastRawOffset;
    int16_t flipThresh;     // threshold to determine whether or not a flip over the 180 degree mark occurred
    bool flipped;

    int16_t zeroPos;        // mechanical zero of the handle, set during automatic callibration

#if defined(__AVR__)
    uint8_t sensor_pin;     // analog input pin number that is connected to the pin 3 of the sensor
    // There is no simple way to use class method as a callback on Arduino,
    // so the user needs to instantiate the timer object by hand and set
    // the plain C callbacks in their program
    // Timer1 timer_tck;
#elif defined(__MBED__)
    PinName sensor_pin;
    AnalogIn sensor;
    Ticker timer_tck;

    // On Mbed ADC always returns 16-bit value,
    // so we shift it to make it 10-bit, same as on Arduino
    inline int16_t analogRead(PinName)
    {
      return sensor.read_u16() >> 6;
    }
#endif

  public:
#if defined(__AVR__)
    HapkitSensor(uint8_t pin, int16_t flip_threshold = 700);
#elif defined(__MBED__)
    HapkitSensor(PinName pin, int16_t flip_threshold = 700);
#endif
    // Reset the variables to their default values
    void reset();
    // Get calibrated position of the motor shaft (in ADC units)
    inline int16_t getPosition()
    {
      return updatedPos - zeroPos;
    }
    // Get raw position of the motor shaft (in ADC units),
    // the flips of the magnetic field are ignored
    inline int16_t getRawPosition()
    {
      return rawPos;
    }

    // Read the sensor and update the motor position,
    // i.e. keep track of the field flips
#if defined(__AVR__)
    void readSensor();
#elif defined(__MBED__)
    void readSensor();
#endif

    // Set zero's position
    inline void setZero(int16_t zero_pos)
    {
        zeroPos = zero_pos;
    };
};

// Interface to the motor via the Adafruit Motor Shield (on Arduino)
// and to X Nucleo IHM04A1 motor shield on STM32
class HapkitMotor
{
  private:
    // Motor ID (screw terminal output number)
    uint8_t motor_id;
#if defined(__AVR__)
    // Adafruit Motor Shield V1
    // AF_DCMotor* motor;

    // Adafruit Motor Shield V2
    Adafruit_MotorShield* AFMS;
    Adafruit_DCMotor *motor;
#elif defined(__MBED__)
    // X Nucleo IHM04A1
    L6206* motor;
#endif

  public:
    HapkitMotor(uint8_t motornum);

    // Start the motor in the given direction
#if defined(__AVR__)
    void run(uint8_t direction);
#elif defined(__MBED__)
    void run(motorDir_t direction);
#endif
    // Set normalized motor speed (0.0; 1.0]
    void setSpeed(float duty);
    // Stop the motor
    void stop();
};

// Holds the kinematic parameters of the Hapkit
typedef struct hapkit_kinematics
{
  float pulley_radius;
  float handle_radius;
  float sector_radius;
  float sector_span;
} hapkit_kinematics_t;

// Kinematic parameters of the first prototype (yellow breadboard)
static const hapkit_kinematics_t HAPKIT_YELLOW = {
  .pulley_radius = 0.00575,
  .handle_radius = 0.07,
  .sector_radius = 0.075,
  .sector_span = (90.0 / 180.0 * M_PI),
};

// Kinematic parameters of the second prototype (blue breadboard)
static const hapkit_kinematics_t HAPKIT_BLUE = {
  .pulley_radius = 0.00420,
  .handle_radius = 0.07,
  .sector_radius = 0.075,
  .sector_span = (90.0 / 180.0 * M_PI),
};

// Holds haptic effect data
typedef struct hapkit_effect
{
  float position;  // the center of haptic effect
  float width;     // width of the effect, defines the range where the effect is applied
  float k_spring;  // spring coefficient of the effect
  float k_damper;  // damping coefficient of the effect

} hapkit_effect_t;

// High-level interface to the Hapkit functions
class Hapkit
{
  private:
    // Motor number
    uint8_t motornum;

    // Analog input pin number
#if defined(__AVR__)
    uint8_t sensor_pin;
#elif defined(__MBED__)
    PinName sensor_pin;
#endif
    // Motor interface object
    HapkitMotor motor;
    // Sensor interface object
    HapkitSensor sensor;
    // Low-pass position filter
    LowPassFilter pos_filter;
    // Low-pass velocity filter
    LowPassFilter vel_filter;
    // Low-pass acceleration filter
    LowPassFilter acc_filter;

    // Position tracking variables for automatic callibration
    int minPos;
    int maxPos;
    // Mechanical zero
    int zeroPos;

    // Kinematics variables
    float rp;               // pulley radius [m]
    float rh;               // handle radius [m]
    float rs;               // sector radius [m]
    float sec_span;         // sector rotation range [rad]
    float sec_K;            // transmission coefficient from sensor val to radians
    float alpha_h;          // rotation angle of the handle [rad]
    float xh;               // position of the handle [m]

    // Force output variables
    float Tp;              // torque of the motor pulley
    float duty;            // duty cylce (between 0 and 255)
    unsigned int output;    // output command to the motor

    float lastXh;     //last x position of the handle
    float vh;         //velocity of the handle
    float lastVh;     //last velocity of the handle
    float lastLastVh; //last last velocity of the handle

    float ah;         //acceleration of the handle
    float lastAh;     //last acceleration of the handle
    float lastLastAh; //last last acceleration of the handle

    hapkit_effect_t* effects; // array of haptic effects
    uint8_t effects_len;      // number of haptic effects

    bool calibrated;          // automatic calibration is over?
    int calibration_counter;  // internal counter for calibration procedure

    float duty_th; // duty cycle threshold

    // Start/stop haptic loop (only on STM32)
    void startLoop();
    void stopLoop();

  public:
#if defined(__AVR__)
    Hapkit(hapkit_kinematics_t kin, uint8_t motornum, uint8_t sensor_pin);
#elif defined(__MBED__)
    Ticker force_tck;
    Hapkit(hapkit_kinematics_t kin, uint8_t motornum, PinName sensor_pin);
#endif
    // Set the motor duty cycle threshold.
    // If the velocity is too low, switch off the motor
    void configure(float duty_threshold = 0.01f);
    // Configure low-pass filters
    void configureFilters(float velFc = 10.0, float accFc = 5.0)
    {
      vel_filter.setFc(velFc);
      acc_filter.setFc(accFc);
    }
    // Start automatic calibration procedure
    void calibrate();
    // Reset variables
    void reset();
    // Set the desired force at the tip of the Hapkit handle, [N]
    void setForce(float force);
    // Calculate the force based on the array of haptic effects
    float calcEffectsForce();

    // Get linear position of the handle, [m]
    inline float getPosition()
    {
        return xh;
    }

    // Get linear velocity of the handle, [m/s]
    inline float getVelocity()
    {
        return vh;
    }

    // Get linear acceleration of the handle, [m/s^2]
    inline float getAcceleration()
    {
        return ah;
    }

    // Get sensor object (for debug purposes)
    inline HapkitSensor* getSensor()
    {
        return &sensor;
    }

    // Run haptic loop once: read the sensor, calculte and apply forces
    void update();

    // Is Hapkit calibrated?
    inline bool isCalibrated()
    {
      return calibrated;
    }

    // Set the array of haptic effects
    inline void setEffects(hapkit_effect_t effects[], uint8_t len)
    {
      // Sanity check
      if (effects && len)
      {
        this->effects = effects;
        this->effects_len = len;

        printf("Setting effects [%d]: 0x%X\n", len, effects);
      }
    }

    // Set the haptic update loop rate, [Hz]
    void setUpdateRate(float rate);
    // Get the haptic update loop rate, [Hz]
    float getUpdateRate();
};

// Support for multiple Hapkits (not implemented)
class HapkitHub
{
  public:
    HapkitHub();
    friend class Hapkit;
    void addHapkit(Hapkit& obj);

  private:

};
#endif
