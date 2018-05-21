#include <TimerOne.h>

#include <hapkit3.h>

static FILE uartout = {0};

static int uart_putchar (char c, FILE *stream)
{
    Serial.write(c);
    return 0;
}

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

uint8_t generate_pattern(hapkit_effect* arr, float from, float to, float step, float k_spring, float k_dumper)
{
  
}

void hapticLoop() {
  hapkit->getSensor()->readSensor();

  if (hapkit->isCalibrated())
  {
    hapkit->update();
//    hapkit->setForce(-hapkit->getAcceleration() * 0.500); // 500g
  }
}

void setup() {
  // Set up serial communication
  Serial.begin(2000000);
  ////////// DEBUG stuff
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout;
  //////////////////////
  hapkit = new Hapkit(HAPKIT_YELLOW, 2, A2);
//  hapkit.configure(0.00575);   // Hapkit No.1 (yellow)
//  hapkit.configure(0.00420);   // Hapkit No.2 (blue)

  timer_tck.initialize(1000000 / 2000); // 2 kHz
  timer_tck.attachInterrupt(hapticLoop);

  hapkit->calibrate();

  hapkit->setEffects(potential_well, sizeof(potential_well) / sizeof(hapkit_effect_t));
}

void loop() {
  if (hapkit->isCalibrated())
  {
  }
//  printf("%1.5f\n", hapkit.getAcceleration());
  delay(50);
}
