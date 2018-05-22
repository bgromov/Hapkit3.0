#include <TimerOne.h>
#include <hapkit3.h>

#define BAUD_RATE 921600 // The maximum supported by Windows side

#define RECVBUFSIZE 512

struct Packet
{
  float force;
  float position;
  float velocity;
  float acceleration;
};

char recvBuf[RECVBUFSIZE] = { '\0' };
int  recvBytes = 0;
bool string_complete = false;
TimerOne timer_tck;
TimerOne send_timer;
Hapkit* hapkit = NULL;
static FILE uartout = {0};
Packet *packet = new Packet();

void serialEvent() {
  while (Serial.available() > 0) {
    char chr = Serial.read();
    if (chr == '\n') {
      recvBuf[recvBytes++] = '\0';
      string_complete = true;
      continue;
    }
    recvBuf[recvBytes++] = chr;
  }
}

void snd() {
  Serial.print("Prm ");
  Serial.print(packet->position, 5);
  Serial.print(" ");
  Serial.print(packet->velocity, 5);
  Serial.print(" ");
  Serial.print(packet->acceleration, 5);
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

void hapticLoop()
{
  hapkit->getSensor()->readSensor();

  if (hapkit->isCalibrated())
  {
    hapkit->update();

    hapkit->setForce(-(hapkit->getPosition() - 0.045) * 50.0f); // Slingshot band
//    hapkit->setForce(-hapkit->getAcceleration() * 0.500); // 500g - Slingshot angle
//    hapkit->setForce(-hapkit->getAcceleration() * 1.500); // 1500g - Rocket dynamics (angle)

    packet->position = hapkit->getPosition();
    packet->velocity = hapkit->getVelocity();
    packet->acceleration = hapkit->getAcceleration();

    if ((counter++) % 10 == 0)
    {
      snd(); // Send data 10 times slower than the hapticLoop()
      if (string_complete) {
        parseMsg();
      }
    }
  }
}

void setup()
{
  Serial.begin(BAUD_RATE);

  hapkit = new Hapkit(HAPKIT_YELLOW, 2, A2);

  hapkit->setUpdateRate(1000.0); // 1kHz
  timer_tck.initialize(1000000 / hapkit->getUpdateRate());
  timer_tck.attachInterrupt(hapticLoop);

  hapkit->calibrate();
}

void loop()
{
  // Empty loop
}
