#ifndef _hapkit3_h_
#define _hapkit3_h_

#include <math.h>
#include <limits.h>

#ifndef M_PI
    #define M_PI                      (3.14159265358979323846f)
#endif

#ifndef max
    #define max(a, b)                 ((a > b ? a : b))
#endif
#ifndef min
    #define min(a, b)                 ((a < b ? a : b))
#endif

#if defined(__AVR__)
  #include <Arduino.h>
  // Build for Arduino with Adafruit Motor Shield v1.0
  // #include <AFMotor.h>
  // Build for Arduino with Adafruit Motor Shield v2.0
  #include <Adafruit_MotorShield.h>
#elif defined(__MBED__)
  // Build for Mbed with X Nucleo IHM04A1
  #include <L6206.h>
  #include "analogin_dma.h"
#endif

// extern g_UpdateRate;

// One-pole recursive low-pass filter
class LowPassFilter
{
  float a0, b1, y1;
public:
  LowPassFilter()
  {
    this->reset();
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
    b1 = 0.0;
    a0 = 1.0;
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

    int16_t zeroPos;

#if defined(__AVR__)
    uint8_t sensor_pin;
    // Timer1 timer_tck;
#elif defined(__MBED__)
    PinName sensor_pin;
    AnalogIn sensor;
    Ticker timer_tck;

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
    void reset();
    inline int16_t getPosition()
    {
      return updatedPos - zeroPos;
    }
    inline int16_t getRawPosition()
    {
      return rawPos;
    }

#if defined(__AVR__)
    // static void readSensor();
    void readSensor();
#elif defined(__MBED__)
    void readSensor();
#endif

    inline void setZero(int16_t zero_pos)
    {
        zeroPos = zero_pos;
    };
};

class HapkitMotor
{
  private:
    uint8_t motor_id;
#if defined(__AVR__)
    // AF_DCMotor* motor;
    Adafruit_MotorShield* AFMS;
    Adafruit_DCMotor *motor;
#elif defined(__MBED__)
    L6206* motor;
#endif

  public:
    HapkitMotor(uint8_t motornum);
#if defined(__AVR__)
    void run(uint8_t direction);
#elif defined(__MBED__)
    void run(motorDir_t direction);
#endif
    void setSpeed(float duty);
    void stop();
};

typedef struct hapkit_kinematics
{
  float pulley_radius;
  float handle_radius;
  float sector_radius;
  float sector_span;
} hapkit_kinematics_t;

static const hapkit_kinematics_t HAPKIT_YELLOW = {
  .pulley_radius = 0.00575,
  .handle_radius = 0.07,
  .sector_radius = 0.075,
  .sector_span = (90.0 / 180.0 * M_PI),
};

static const hapkit_kinematics_t HAPKIT_BLUE = {
  .pulley_radius = 0.00420,
  .handle_radius = 0.07,
  .sector_radius = 0.075,
  .sector_span = (90.0 / 180.0 * M_PI),
};

typedef struct hapkit_effect
{
  float position;
  float width;
  float k_spring;
  float k_dumper;

} hapkit_effect_t;

class Hapkit
{
  private:
    uint8_t motornum;

#if defined(__AVR__)
    uint8_t sensor_pin;
#elif defined(__MBED__)
    PinName sensor_pin;
#endif

    HapkitMotor motor;
    HapkitSensor sensor;
    LowPassFilter pos_filter;
    LowPassFilter vel_filter;
    LowPassFilter acc_filter;

    // Position tracking variables
    int minPos;
    int maxPos;
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
    // float force;           // force at the handle
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

    hapkit_effect_t* effects;
    uint8_t effects_len;

    bool calibrated;
    int calibration_counter;

    float duty_th; // duty cycle threshold

    void startLoop();
    void stopLoop();

  public:
#if defined(__AVR__)
    Hapkit(hapkit_kinematics_t kin, uint8_t motornum, uint8_t sensor_pin);
#elif defined(__MBED__)
    Ticker force_tck;
    Hapkit(hapkit_kinematics_t kin, uint8_t motornum, PinName sensor_pin);
#endif
    void configure(float duty_threshold = 0.01f);
    void configureFilters(float velFc = 10.0, float accFc = 5.0)
    {
      vel_filter.setFc(velFc);
      acc_filter.setFc(accFc);
    }
    void calibrate();
    void reset();
    void setForce(float force);
    float calcEffectsForce();

    inline float getPosition()
    {
        return xh;
    }

    inline float getVelocity()
    {
        return vh;
    }

    inline float getAcceleration()
    {
        return ah;
    }

    inline HapkitSensor* getSensor()
    {
        return &sensor;
    }

    void update();

    inline bool isCalibrated()
    {
      return calibrated;
    }

    inline void setEffects(hapkit_effect_t effects[], uint8_t len)
    {
      if (effects && len)
      {
        this->effects = effects;
        this->effects_len = len;

        printf("Setting effects [%d]: 0x%X\n", len, effects);
      }
    }

    float getUpdateRate();
    void setUpdateRate(float rate);
};

class HapkitHub
{
  public:
    HapkitHub();
    friend class Hapkit;
    void addHapkit(Hapkit& obj);

  private:

};
#endif
