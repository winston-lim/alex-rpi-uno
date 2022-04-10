#include "serialize.h"
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <math.h>

typedef enum
{
  STOP = 0,
  FORWARD = 1,
  BACKWARD = 2,
  LEFT = 3,
  RIGHT = 4
} TDirection;

volatile TDirection dir = STOP;

/*
   Alex's configuration constants
*/

// Number of ticks per revolution from the
// wheel encoder.

#define COUNTS_PER_REV 160

// Wheel circumference in cm.
// We will use this to calculate forward/backward distance traveled
// by taking revs * WHEEL_CIRC

#define WHEEL_CIRC 20

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF 6  // Left forward pin
#define LR 5  // Left reverse pin
#define RF 10 // Right forward pin
#define RR 9  // Right reverse pin

// Alex's length and breadth in cm
#define ALEX_LENGTH 4
#define ALEX_BREADTH 6

// PI, for calculating turn in circumference
#define PI 3.141592654

// Alex's diagonal. We compute and store this once since it is
// expensive to compute and doesn't really change.
float AlexDiagonal = 0.0; // to change

// Alex's turning circumference, calculated once
float AlexCirc = 0.0; // to change

/*
      Alex's State Variables
*/

// Store the ticks from Alex's left and
// right encoders.
volatile unsigned long leftForwardTicks;
volatile unsigned long rightForwardTicks;
volatile unsigned long leftReverseTicks;
volatile unsigned long rightReverseTicks;

// Left and right encoder ticks for turning
volatile unsigned long leftForwardTicksTurns;
volatile unsigned long rightForwardTicksTurns;
volatile unsigned long leftReverseTicksTurns;
volatile unsigned long rightReverseTicksTurns;

// Store the revolutions on Alex's left
// and right wheels
volatile unsigned long leftRevs;
volatile unsigned long rightRevs;

// Forward and backward distance traveled
volatile unsigned long forwardDist;
volatile unsigned long reverseDist;

// Variable to keep track of whether we have moved a commanded distance
unsigned long deltaDist;
unsigned long newDist;

// Variables to keep track of our turning angle
unsigned long deltaTicks;
unsigned long targetTicks;

/*

   Alex Communication Routines.

*/

TResult readPacket(TPacket *packet)
{
  // Reads in data from the serial port and
  // deserializes it.Returns deserialized
  // data in "packet".

  char buffer[PACKET_SIZE];
  int len;

  len = readSerial(buffer);

  if (len == 0)
    return PACKET_INCOMPLETE;
  else
    return deserialize(buffer, len, packet);
}

void sendStatus()
{
  // Implement code to send back a packet containing key
  // information like leftTicks, rightTicks, leftRevs, rightRevs
  // forwardDist and reverseDist
  // Use the params array to store this information, and set the
  // packetType and command files accordingly, then use sendResponse
  // to send out the packet. See sendMessage on how to use sendResponse.
  TPacket statusPacket;
  statusPacket.packetType = PACKET_TYPE_RESPONSE;
  statusPacket.command = RESP_STATUS;
  statusPacket.params[0] = leftForwardTicks;
  statusPacket.params[1] = rightForwardTicks;
  statusPacket.params[2] = leftReverseTicks;
  statusPacket.params[3] = rightReverseTicks;
  statusPacket.params[4] = leftForwardTicksTurns;
  statusPacket.params[5] = rightForwardTicksTurns;
  statusPacket.params[6] = leftReverseTicksTurns;
  statusPacket.params[7] = rightReverseTicksTurns;
  statusPacket.params[8] = forwardDist;
  statusPacket.params[9] = reverseDist;
  sendResponse(&statusPacket);
}

void sendMessage(const char *message)
{
  // Sends text messages back to the Pi. Useful
  // for debugging.

  TPacket messagePacket;
  messagePacket.packetType = PACKET_TYPE_MESSAGE;
  strncpy(messagePacket.data, message, MAX_STR_LEN);
  sendResponse(&messagePacket);
}

void dbprint(char *format, ...)
{
  va_list args;
  char buffer[128];
  va_start(args, format);
  vsprintf(buffer, format, args);
  sendMessage(buffer);
}

void print_input(float param1, float param2)
{
  char _param1[11];
  char _param2[11];
  String(param1).toCharArray(_param1, 5);
  String(param2).toCharArray(_param2, 5);
  dbprint(_param1);
  dbprint(_param2);
}

void sendBadPacket()
{
  // Tell the Pi that it sent us a packet with a bad
  // magic number.
  TPacket badPacket;
  badPacket.packetType = PACKET_TYPE_ERROR;
  badPacket.command = RESP_BAD_PACKET;
  sendResponse(&badPacket);
}

void sendBadChecksum()
{
  // Tell the Pi that it sent us a packet with a bad
  // checksum.

  TPacket badChecksum;
  badChecksum.packetType = PACKET_TYPE_ERROR;
  badChecksum.command = RESP_BAD_CHECKSUM;
  sendResponse(&badChecksum);
}

void sendBadCommand()
{
  // Tell the Pi that we don't understand its
  // command sent to us.

  TPacket badCommand;
  badCommand.packetType = PACKET_TYPE_ERROR;
  badCommand.command = RESP_BAD_COMMAND;
  sendResponse(&badCommand);
}

void sendBadResponse()
{
  TPacket badResponse;
  badResponse.packetType = PACKET_TYPE_ERROR;
  badResponse.command = RESP_BAD_RESPONSE;
  sendResponse(&badResponse);
}

void sendOK()
{
  TPacket okPacket;
  okPacket.packetType = PACKET_TYPE_RESPONSE;
  okPacket.command = RESP_OK;
  sendResponse(&okPacket);
}

void sendResponse(TPacket *packet)
{
  // Takes a packet, serializes it then sends it out
  // over the serial port.
  char buffer[PACKET_SIZE];
  int len;

  len = serialize(buffer, packet, sizeof(TPacket));
  writeSerial(buffer, len);
}

/*
   Setup and start codes for external interrupts and
   pullup resistors.

*/
// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
  // Use bare-metal to enable the pull-up resistors on pins
  // 2 and 3. These are pins PD2 and PD3 respectively.
  // We set bits 2 and 3 in DDRD to 0 to make them inputs.
  DDRD &= 0b11110011;

  // setting pins 2 and 3 to HIGH
  PORTD |= 0b00001100;
}

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
  switch (dir)
  {
  case FORWARD:
    leftForwardTicks++;
    forwardDist = (unsigned long)((float)leftForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
    break;
  case BACKWARD:
    leftReverseTicks++;
    reverseDist = (unsigned long)((float)leftReverseTicks / COUNTS_PER_REV * WHEEL_CIRC);
    break;
  case LEFT:
    leftReverseTicksTurns++;
    break;
  case RIGHT:
    leftForwardTicksTurns++;
    break;
  }
}

void rightISR()
{
  switch (dir)
  {
  case FORWARD:
    rightForwardTicks++;
    break;
  case BACKWARD:
    rightReverseTicks++;
    break;
  case LEFT:
    rightForwardTicksTurns++;
    break;
  case RIGHT:
    rightReverseTicksTurns++;
    break;
  }
}

// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered. Use bare-metal.
void setupEINT()
{
  // Use bare-metal to configure pins 2 and 3 to be
  // falling edge triggered. Remember to enable
  // the INT0 and INT1 interrupts.
  EICRA |= 0b00001010;
  EIMSK |= 0b00000011;
}

// Implement the external interrupt ISRs below.
// INT0 ISR should call leftISR while INT1 ISR
// should call rightISR.
ISR(INT0_vect)
{
  leftISR();
}

ISR(INT1_vect)
{
  rightISR();
}

// Implement INT0 and INT1 ISRs above.

/*
   Setup and start codes for serial communications

*/
// Set up the serial connection. For now we are using
// Arduino Wiring, you will replace this later
// with bare-metal code.
void setupSerial()
{
  // To replace later with bare-metal.
  Serial.begin(9600);
}

// Start the serial connection. For now we are using
// Arduino wiring and this function is empty. We will
// replace this later with bare-metal code.

void startSerial()
{
  // Empty for now. To be replaced with bare-metal code
  // later on.
}

// Read the serial port. Returns the read character in
// ch if available. Also returns TRUE if ch is valid.
// This will be replaced later with bare-metal code.

int readSerial(char *buffer)
{

  int count = 0;

  while (Serial.available())
    buffer[count++] = Serial.read();

  return count;
}

// Write to the serial port. Replaced later with
// bare-metal code

void writeSerial(const char *buffer, int len)
{
  Serial.write(buffer, len);
}

/*
   Alex's motor drivers.

*/

// Set up Alex's motors. Right now this is empty, but
// later you will replace it with code to set up the PWMs
// to drive the motors.
void setupMotors()
{
  /* Our motor set up is:
        A1IN - Pin 5, PD5, OC0B
        A2IN - Pin 6, PD6, OC0A
        B1IN - Pin 10, PB2, OC1B
        B2In - pIN 11, PB3, OC2A
  */
}

// Start the PWM for Alex's motors.
// We will implement this later. For now it is
// blank.
void startMotors()
{
}

// Convert percentages to PWM values
int pwmVal(float speed)
{
  if (speed < 0.0)
    speed = 0;

  if (speed > 100.0)
    speed = 100.0;

  return (int)((speed / 100.0) * 255.0);
}

// Move Alex forward "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// move forward at half speed.
// Specifying a distance of 0 means Alex will
// continue moving forward indefinitely.
void forward(float dist, float speed)
{
  // Code to tell us how far to move
  if (dist > 0)
  {
    deltaDist = (int)(dist * 0.78);
  }
  else
  {
    deltaDist = 9999999;
  }
  newDist = forwardDist + deltaDist;
  dir = FORWARD;
  int val = pwmVal(speed);
  if (dist == 10 && speed == 70)
  {
    analogWrite(RF, (int)(val * 0.72));
    analogWrite(LF, (int)(val));
    analogWrite(LR, 0);
    analogWrite(RR, 0);
  }
  else if (dist == 10 && speed == 90)
  {
    analogWrite(RF, (int)(val));
    analogWrite(LF, (int)(val));
    analogWrite(LR, 0);
    analogWrite(RR, 0);
  }
  else if (dist == 5)
  {
    analogWrite(RF, (int)(val * 0.72));
    analogWrite(LF, (int)(val));
    analogWrite(LR, 0);
    analogWrite(RR, 0);
  }

  else if (dist == 40 && speed == 90)
  {
    analogWrite(RF, (int)(val));
    analogWrite(LF, (int)(val));
    analogWrite(LR, 0);
    analogWrite(RR, 0);
  }

  else
  {

    analogWrite(RF, (int)(val * 0.72));
    analogWrite(LF, (int)(val));
    analogWrite(LR, 0);
    analogWrite(RR, 0);
  }
}

// Reverse Alex "dist" cm at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// reverse at half speed.
// Specifying a distance of 0 means Alex will
// continue reversing indefinitely.
void reverse(float dist, float speed)
{
  // Code to rell us how far to move
  if (dist == 0)
  {
    deltaDist = 9999999;
  }
  else
  {
    deltaDist = dist;
  }
  newDist = reverseDist + deltaDist;
  dir = BACKWARD;
  int val = pwmVal(speed);
  if (dist == 10)
  {
    analogWrite(RR, (int)(val * 0.69));
    analogWrite(LR, (int)(val));
    analogWrite(LF, 0);
    analogWrite(RF, 0);
  }

  else if (dist == 5)
  {
    analogWrite(RR, (int)(val * 0.78));
    analogWrite(LR, (int)(val));
    analogWrite(LF, 0);
    analogWrite(RF, 0);
  }

  else
  {
    analogWrite(RR, (int)(val * 0.78));
    analogWrite(LR, (int)(val));
    analogWrite(LF, 0);
    analogWrite(RF, 0);
  }
}

// Function to estimate the number of wheel ticks needed to turn an angle
unsigned long computeDeltaTicks(float ang)
{
  // We will assume that angular distance moved = linerar distance moved in one wheel
  // revolution. This is (probably) incorrect but simplifies calculation.
  // # of wheel revs to make one full 360 turn is AlexCirc / WHEEL_CIRC
  // This is for 360 degrees. For ang degrees it will be (ang * AlexCirc) / (360.0 * WHEEL_CIRC)
  // To convert to ticks, we multiply by COUNTS_PER_REV

  unsigned long ticks = (unsigned long)((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));

  return ticks;
}

// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn left indefinitely.
void left(float ang, float speed)
{
  dir = LEFT;
  int val = pwmVal(speed);

  if (ang == 15)
  {
    ang *= 1.52;
  }

  if (ang == 0)
  {
    deltaTicks = 9999999;
  }
  else
  {
    if (ang == 90)
    {
      deltaTicks = 6 * computeDeltaTicks(15 * 1.52);
    }
    else if (ang == 360)
    {
      deltaTicks = 24 * computeDeltaTicks(15 * 1.52);
    }
    else
    {
      deltaTicks = computeDeltaTicks(ang);
    }
  }

  targetTicks = leftReverseTicksTurns + deltaTicks;

  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn left we reverse the left wheel and move
  // the right wheel forward.
  analogWrite(LR, val);
  analogWrite(RF, (int)val * 0.85);
  analogWrite(LF, 0);
  analogWrite(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
// "speed" is expressed as a percentage. E.g. 50 is
// turn left at half speed.
// Specifying an angle of 0 degrees will cause Alex to
// turn right indefinitely.
void right(float ang, float speed)
{
  dir = RIGHT;
  int val = pwmVal(speed);

  if (ang == 15)
  {
    ang *= 1.33;
  }

  if (ang == 0)
  {
    deltaTicks = 9999999;
  }
  else
  {
    if (ang == 90)
    {
      deltaTicks = 6 * computeDeltaTicks(15 * 1.33);
    }
    else if (ang == 360)
    {
      deltaTicks = 24 * computeDeltaTicks(15 * 1.33);
    }
    else
    {
      deltaTicks = computeDeltaTicks(ang);
    }
  }

  targetTicks = leftForwardTicksTurns + deltaTicks;
  // For now we will ignore ang. We will fix this in Week 9.
  // We will also replace this code with bare-metal later.
  // To turn right we reverse the right wheel and move
  // the left wheel forward.
  analogWrite(RR, (int)val * 0.78);
  analogWrite(LF, val);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
}

// Stop Alex. To replace with bare-metal code later.
void stop()
{
  dir = STOP;

  analogWrite(LF, 0);
  analogWrite(LR, 0);
  analogWrite(RF, 0);
  analogWrite(RR, 0);
}
// Stop Alex. To replace with bare-metal code later.
void stop_right_first()
{
  dir = STOP;
  analogWrite(RF, 0);
  delay(1);
  analogWrite(LF, 0);
  analogWrite(LR, 0);

  analogWrite(RR, 0);
}

/*
   Alex's setup and run codes

*/

// Clears all our counters
void clearCounters()
{
  leftForwardTicks = 0;
  rightForwardTicks = 0;
  leftReverseTicks = 0;
  rightReverseTicks = 0;
  leftForwardTicksTurns = 0;
  rightForwardTicksTurns = 0;
  leftReverseTicksTurns = 0;
  rightReverseTicksTurns = 0;
  leftRevs = 0;
  rightRevs = 0;
  forwardDist = 0;
  reverseDist = 0;
}

// Clears one particular counter
void clearOneCounter(int which)
{
  clearCounters();
}
// Intialize Vincet's internal states

void initializeState()
{
  clearCounters();
}

void handleCommand(TPacket *command)
{
  switch (command->command)
  {
  // For movement commands, param[0] = distance, param[1] = speed.
  case COMMAND_FORWARD:
    sendOK();
    forward((float)command->params[0], (float)command->params[1]);
    break;

  case COMMAND_REVERSE:
    sendOK();
    reverse((float)command->params[0], (float)command->params[1]);
    break;

  case COMMAND_TURN_LEFT:
    sendOK();
    left((float)command->params[0], (float)command->params[1]);
    break;

  case COMMAND_TURN_RIGHT:
    sendOK();
    right((float)command->params[0], (float)command->params[1]);
    break;

  case COMMAND_STOP:
    sendOK();
    stop();
    break;

  case COMMAND_GET_STATS:
    sendStatus();
    break;

  case COMMAND_CLEAR_STATS:
    sendOK();
    clearOneCounter(command->params[0]);
    break;

  default:
    sendBadCommand();
  }
}

void waitForHello()
{
  int exit = 0;

  while (!exit)
  {
    TPacket hello;
    TResult result;

    do
    {
      result = readPacket(&hello);
    } while (result == PACKET_INCOMPLETE);

    if (result == PACKET_OK)
    {
      if (hello.packetType == PACKET_TYPE_HELLO)
      {

        sendOK();
        exit = 1;
      }
      else
        sendBadResponse();
    }
    else if (result == PACKET_BAD)
    {
      sendBadPacket();
    }
    else if (result == PACKET_CHECKSUM_BAD)
      sendBadChecksum();
  } // !exit
}

void setup()
{
  // put your setup code here, to run once:

  // Compute the diagonal and circumference
  AlexDiagonal = sqrt((ALEX_LENGTH * ALEX_LENGTH) + (ALEX_BREADTH * ALEX_BREADTH));
  AlexCirc = PI * AlexDiagonal;

  cli();
  setupEINT();
  setupSerial();
  startSerial();
  setupMotors();
  startMotors();
  enablePullups();
  initializeState();
  sei();
}

void handlePacket(TPacket *packet)
{
  switch (packet->packetType)
  {
  case PACKET_TYPE_COMMAND:
    handleCommand(packet);
    break;

  case PACKET_TYPE_RESPONSE:
    break;

  case PACKET_TYPE_ERROR:
    break;

  case PACKET_TYPE_MESSAGE:
    break;

  case PACKET_TYPE_HELLO:
    break;
  }
}

void loop()
{

  // Uncomment the code below for Step 2 of Activity 3 in Week 8 Studio 2

  // forward(0, 50);
  // analogWrite(RF, pwmVal(80));

  // Uncomment the code below for Week 9 Studio 2

  // put your main code here, to run repeatedly:
  TPacket recvPacket; // This holds commands from the Pi
  TResult result = readPacket(&recvPacket);

  if (result == PACKET_OK)
    handlePacket(&recvPacket);
  else if (result == PACKET_BAD)
  {
    sendBadPacket();
  }
  else if (result == PACKET_CHECKSUM_BAD)
  {
    sendBadChecksum();
  }
  if (deltaDist > 0)
  {
    if (dir == FORWARD)
    {
      if (forwardDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop_right_first();
      }
    }
    else if (dir == BACKWARD)
    {
      if (reverseDist > newDist)
      {
        deltaDist = 0;
        newDist = 0;
        stop();
      }
    }
    else if (dir == STOP)
    {
      deltaDist = 0;
      newDist = 0;
      stop();
    }
  }

  if (deltaTicks > 0)
  {
    if (dir == LEFT)
    {
      if (leftReverseTicksTurns >= targetTicks)
      {
        deltaTicks = 0;
        targetTicks = 0;
        stop();
      }
    }
    else
    {
      if (dir == RIGHT)
      {
        if (leftForwardTicksTurns >= targetTicks)
        {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
      }
      else
      {
        if (dir == STOP)
        {
          deltaTicks = 0;
          targetTicks = 0;
          stop();
        }
      }
    }
  }
}
