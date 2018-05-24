#include <TimerOne.h>
#include <hapkit3.h>

//#define USE_PRINTF

//#if defined(USE_PRINTF)
static FILE uartout = {0};

static int uart_putchar (char c, FILE *stream)
{
    Serial.write(c);
    return 0;
}
//#endif

Hapkit* hapkit = NULL;

TimerOne timer_tck;

const hapkit_effect_t potential_well[] = {
  {
    .position = -0.04,
    .width = 0.003,
    .k_spring = 700.0,
    .k_dumper = 5.7,
  },
  {
    .position = -0.02,
    .width = 0.003,
    .k_spring = 700.0,
    .k_dumper = 5.7,
  },
//  {
//    .position = 0.0,
//    .width = 0.010,
//    .k_spring = 1500.0,
//    .k_dumper = 5.7,
//  },
  {
    .position = 0.02,
    .width = 0.003,
    .k_spring = 700.0,
    .k_dumper = 5.7,
  },
  {
    .position = 0.04,
    .width = 0.003,
    .k_spring = 700.0,
    .k_dumper = 5.7,
  },
};

uint32_t counter = 0;

uint8_t mode = 0;

uint8_t buttonPin = 13;
uint8_t button = 0;
uint8_t lastButton = 0;

float force = 0.0;

void hapticLoop()
{
  hapkit->getSensor()->readSensor();
  if (hapkit->isCalibrated())
  {
    hapkit->update();
    force = hapkit->calcEffectsForce();

    button = digitalRead(buttonPin);
    if (button - lastButton > 0) // rising edge
    {
      mode++;
    }

    lastButton = button;
  }
}

void setup() {
  // Set up serial communication
  Serial.begin(2000000);

//#if defined(USE_PRINTF)
  ////////// DEBUG stuff
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout;
  //////////////////////
//#endif

  pinMode(buttonPin, OUTPUT);
  digitalWrite(buttonPin, LOW);
  pinMode(buttonPin, INPUT_PULLUP);

  hapkit = new Hapkit(HAPKIT_YELLOW, 1, A2);
  hapkit->setUpdateRate(1000.0);

  timer_tck.initialize(1000000 / hapkit->getUpdateRate()); // 1 kHz by default
  timer_tck.attachInterrupt(hapticLoop);

  hapkit->calibrate();

  hapkit->setEffects(potential_well, sizeof(potential_well) / sizeof(hapkit_effect_t));
  mode = 0;
}

void loop() {
  if (hapkit->isCalibrated())
  {
      switch(mode % 5)
      {
        case 0:
          hapkit->setForce(0.0); // Free space
          break;
        case 1:
          hapkit->setForce(force); // Slingshot band with holds
          break;
        case 2:
          hapkit->setForce(-(hapkit->getPosition() - 0.045) * 50.0 - hapkit->getVelocity() * 2.5); // Slingshot band
          break;
        case 3:
          hapkit->setForce(-hapkit->getAcceleration() * 0.500);// -hapkit->getVelocity() * 0.05); // 500g - Slingshot angle
          break;
        case 4:
          hapkit->setForce(-hapkit->getAcceleration() * 1.500 - hapkit->getVelocity() * 1.0); // 1500g - Rocket dynamics (angle)
          break;
      }
  }
}