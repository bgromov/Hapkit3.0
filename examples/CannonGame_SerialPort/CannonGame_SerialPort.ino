#include <TimerOne.h>
#include <hapkit3.h>

#define BAUD_RATE 2000000 //921600 // The maximum supported by Windows side

#define RECVBUFSIZE 512

static FILE uartout = {0};

static int uart_putchar (char c, FILE *stream)
{
    Serial.write(c);
    return 0;
}

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

struct Packet
{
  float force;
  float position;
  float velocity;
  float acceleration;
  bool  button;
};

char recvBuf[RECVBUFSIZE] = { '\0' };
int  recvBytes = 0;
bool string_complete = false;
TimerOne timer_tck;
TimerOne send_timer;
Hapkit* hapkit = NULL;
Packet *packet = new Packet();

// void serialEvent() {
//   while (Serial.available() > 0) {
//     char chr = Serial.read();
//     if (chr == '\n') {
//       recvBuf[recvBytes++] = '\0';
//       string_complete = true;
//       continue;
//     }
//     recvBuf[recvBytes++] = chr;
//   }
// }

void snd() {
  Serial.print("Prm ");
  Serial.print(packet->position, 5);
  Serial.print(" ");
  Serial.print(packet->velocity, 5);
  Serial.print(" ");
  Serial.print(packet->acceleration, 5);
  Serial.print(" ");
  Serial.print(!packet->button);
  Serial.print("\n");
}

void snd_ack() {
  Serial.print("Ok");

  Serial.print("\n");
}

void parseMsg() {

  recvBytes = 0;
  string_complete = false;


  if (recvBuf[0] == 'P' && recvBuf[1] == '1')
  {

    char *header = strtok(recvBuf, " ");
    char *param1 = strtok(NULL, " ");

    packet->force = atof(param1);

    //snd_ack();
    Serial.print("Ok\n");
    //Serial.print(packet->force, 5);
    //Serial.print("\n");
  }
  else
  {
    //Serial.println("Unknown message");
  }
    //Serial.print("Ok\n");
}

int counter = 0;

uint8_t buttonPin = 13;
uint8_t button = 0;
uint8_t lastButton = 0;

uint8_t mode = 0;

float force = 0.0;

void hapticLoop()
{
  hapkit->getSensor()->readSensor();

  if (hapkit->isCalibrated())
  {
    hapkit->update();

    force = hapkit->calcEffectsForce();

    // Handle buttons events
    button = digitalRead(buttonPin);
    if (button - lastButton > 0) // rising edge
    {
      mode++;
    }
    lastButton = button;

    packet->position = hapkit->getPosition();
    packet->velocity = hapkit->getVelocity();
    packet->acceleration = hapkit->getAcceleration();
    packet->button = button;

    if ((counter++) % 30 == 0)
    {
      snd(); // Send data 10 times slower than the hapticLoop()
      // if (string_complete) {
      //   parseMsg();
      // }
    }
  }
}

void setup()
{
  Serial.begin(BAUD_RATE);

  fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uartout;

  // Set up button pin
  pinMode(buttonPin, OUTPUT);
  digitalWrite(buttonPin, LOW);
  pinMode(buttonPin, INPUT_PULLUP);

  hapkit = new Hapkit(HAPKIT_YELLOW, 1, A2);

  hapkit->setUpdateRate(1000.0); // 1kHz
  timer_tck.initialize(1000000 / hapkit->getUpdateRate());
  timer_tck.attachInterrupt(hapticLoop);

  hapkit->calibrate();

  hapkit->configureFilters(10.0, 5.0);

  mode = 0;
}

void loop()
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
