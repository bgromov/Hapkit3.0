#include <TimerOne.h>
#include <hapkit3.h>

// Make printf() output to the serial port
static FILE uartout = {0};

static int uart_putchar (char c, FILE *stream)
{
    Serial.write(c);
    return 0;
}

// Hapkit 3.0 object
Hapkit* hapkit = NULL;

// Timer object
TimerOne timer_tck;

// Define the arrau of haptic effects -- click-in positions
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

// Track the button clicks
uint8_t buttonPin = 13;
uint8_t button = 0;
uint8_t lastButton = 0;

float force = 0.0;

// Haptic loop,
void hapticLoop()
{
  // Read the sensor and update internal state of the Hapkit object
  hapkit->getSensor()->readSensor();

  // Is Hapkit calibrated?
  if (hapkit->isCalibrated())
  {
    // Render the forces
    hapkit->update();
    // Calculate the haptic effects w.r.t. the current handle position
    force = hapkit->calcEffectsForce();

    // Handle the button (only available on Hapkit Yellow)
    // Read the button pin
    button = digitalRead(buttonPin);

    // Button down?
    if (button - lastButton > 0) // rising edge
    {
      // Loop over control modes
      mode++;
    }

    // Remember last state
    lastButton = button;
  }
}

void setup() {
  // Set up serial communication
  Serial.begin(2000000);

  // Redirect stdout to serial port
  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout;

  // Set the button to Pull-Up mode,
  // i.e. when the button unpressed the value is 1
  pinMode(buttonPin, OUTPUT);
  digitalWrite(buttonPin, LOW);
  pinMode(buttonPin, INPUT_PULLUP);

  // Instantiate the Hapkit object
  // Use pre-defined kinematic parameters.
  // Motor termina: 1
  // Sensor analog pin: A2
  hapkit = new Hapkit(HAPKIT_YELLOW, 1, A2);
  // Update at 1 kHz rate
  hapkit->setUpdateRate(1000.0);

  // Setup the timer
  timer_tck.initialize(1000000 / hapkit->getUpdateRate()); // 1 kHz by default
  // Run the haptic loop at 1 kHz rate
  timer_tck.attachInterrupt(hapticLoop);

  // Run automatic calibration
  // The handle will bounce to the mechanical limits
  hapkit->calibrate();

  // Set low-pass velocity and acceleration filters
  hapkit->configureFilters(10.0, 5.0);

  // Set the haptic effects
  hapkit->setEffects(potential_well, sizeof(potential_well) / sizeof(hapkit_effect_t));

  // Default mode 0: no forces applied, free space motion
  mode = 0;
}

// Main loop
void loop() {
  // Did calibration finish?
  if (hapkit->isCalibrated())
  {
      // Set the mode
      switch(mode % 5)
      {
        case 0:
          // No forces applied, free space
          hapkit->setForce(0.0);
          break;
        case 1:
          // Virtual spring attached at the zero position,
          // but with the click-in holds every 2 cm
          hapkit->setForce(force); // Slingshot band with holds
          break;
        case 2:
          // Virtual spring attached at the very end of the handle range
          hapkit->setForce(-(hapkit->getPosition() - 0.045) * 50.0 - hapkit->getVelocity() * 2.5); // Slingshot band
          break;
        case 3:
          // Virtual mass, a kind of 0.5 kg
          hapkit->setForce(-hapkit->getAcceleration() * 0.500);// -hapkit->getVelocity() * 0.05); // 500g - Slingshot angle
          break;
        case 4:
          // Virtual mass, a kind of 1.5 kg
          hapkit->setForce(-hapkit->getAcceleration() * 1.500 - hapkit->getVelocity() * 1.0); // 1500g - Rocket dynamics (angle)
          break;
      }
  }
}