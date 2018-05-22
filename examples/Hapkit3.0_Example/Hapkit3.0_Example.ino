#include <TimerOne.h>
#include <hapkit3.h>

//#define USE_PRINTF

#if defined(USE_PRINTF)
static FILE uartout = {0};

static int uart_putchar (char c, FILE *stream)
{
    Serial.write(c);
    return 0;
}
#endif

Hapkit* hapkit = NULL;

TimerOne timer_tck;

const hapkit_effect_t potential_well[] = {
  {
    .position = -0.04,
    .width = 0.003,
    .k_spring = 500.0,
    .k_dumper = 0.7,
  },
  {
    .position = -0.02,
    .width = 0.003,
    .k_spring = 500.0,
    .k_dumper = 0.7,
  },
  {
    .position = 0.0,
    .width = 0.003,
    .k_spring = 1500.0,
    .k_dumper = 0.7,
  },
  {
    .position = 0.02,
    .width = 0.003,
    .k_spring = 500.0,
    .k_dumper = 0.7,
  },
  {
    .position = 0.04,
    .width = 0.003,
    .k_spring = 500.0,
    .k_dumper = 0.7,
  },
};

void hapticLoop() {
  hapkit->getSensor()->readSensor();

  if (hapkit->isCalibrated())
  {
    hapkit->update();
    hapkit->setForce(-hapkit->getAcceleration() * 0.500); // 500g
  }
}

void setup() {
  // Set up serial communication
  Serial.begin(2000000);

#if defined(USE_PRINTF)
  ////////// DEBUG stuff
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout;
  //////////////////////
#endif

  hapkit = new Hapkit(HAPKIT_YELLOW, 2, A2);
  hapkit->setUpdateRate(2000.0);

  timer_tck.initialize(1000000 / hapkit->getUpdateRate()); // 2 kHz by default
  timer_tck.attachInterrupt(hapticLoop);

  hapkit->calibrate();

//  hapkit->setEffects(potential_well, sizeof(potential_well) / sizeof(hapkit_effect_t));
}

void loop() {
  if (hapkit->isCalibrated())
  {
  }
//  printf("%d\n", hapkit->getSensor()->getPosition());
  delay(50);
}