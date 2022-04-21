#include "serialize.h"
#include <stdarg.h>
#include "packet.h"
#include "constants.h"
#include <math.h>
#include "buffer.h"

/**
 * Extra features implemented
 * 1. Full Baremetal(PWM, ultrasonic, , GPIO, interrupts, serial)
 * 2. Calibration for motor controls
 * 3. Sanitizing inputs(checking params of commands for invalid values that are not numbers)
 * 4. Emergency-stop Alex(uses ultrasonic sensors to automatically stop if obstacle is < THRESHOlD_DISTANCE away)
 * 5. TLS Setup on operator device/RPi
 */

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

   Config constants

*/

// Number of ticks per revolution from the
// wheel encoder.
#define COUNTS_PER_REV 160

// Wheel circumference in cm.
#define WHEEL_CIRC 20

// Motor control pins. You need to adjust these till
// Alex moves in the correct direction
#define LF 6  // Left forward pin
#define LR 5  // Left reverse pin
#define RF 10 // Right forward pin
#define RR 9  // Right reverse pin
#define PIN5 (1 << 5)
#define PIN6 (1 << 6)
#define PIN9 (1 << 1)
#define PIN10 (1 << 2)

// Alex's length and breadth in cm
#define ALEX_LENGTH 6
#define ALEX_BREADTH 10

#define PI 3.141592654

// Alex's diagonal(computed later)
float AlexDiagonal = 0.0;

// Alex's turning circumference(computed later)
float AlexCirc = 0.0;

// Ultrasonic variables
#define TRIGGER_PIN 12
#define ECHO_PIN 13
#define SPEED_OF_SOUND 0.0345
#define THRESHOLD_DISTANCE 7 // refers to distance from obstacle in front in centimeters

/*

    ##### Alex's State Variables

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

// Variable to keep track of whether ultrasonic has triggered a stop
volatile bool hasStopped = false;

// Variables for circular buffer for baremetal serial communications
volatile TBuffer txBuffer, rxBuffer;

/*

   ##### Alex utility functions.

*/

// Gets baudrate configuration
unsigned int get_baud_rate()
{
    // 16Mhz is the clockrate for Uno
    // 9600 is desired baurate rate
    return 16000000 / 16 / 9600 - 1;
}
// Gets distance of obstacle from the front of Alex
double getUltrasonicReading()
{
    PORTB |= 0b00001000;
    // digitalWrite(TRIGGER_PIN, HIGH);
    _delay_us(10);
    // delayMicroseconds(10);
    PORTB &= 0b11101111;
    // digitalWrite(TRIGGER_PIN, LOW);
    int sec = pulseIn(ECHO_PIN, HIGH);
    double cms = (sec * SPEED_OF_SOUND / 2);
    return cms;
}

// bare-metal version of analogWrite()
void baremetal_analog_write(int port, int pwmVal)
{
    // Our setup pins are reversed i.e. front is back, back is front
    /* Our motor set up is:
          A1IN - Pin 6, PD6, OC0A == LF
          A2IN - Pin 5, PD5, OC0B == LR
          B1IN - Pin 10, PB2, OC1B == RF
          B2In - Pin 9, PB1, OC1A == RR
    */
    if (port == LF)
    {
        OCR0A = pwmVal;
    }
    else if (port == LR)
    {
        OCR0B = pwmVal;
    }
    else if (port == RF)
    {
        OCR1B = pwmVal;
    }
    else if (port == RR)
    {
        OCR1A = pwmVal;
    }
}

// Gets pwmValue for a given percentage
int pwmVal(float speed)
{
    if (speed < 0.0)
        speed = 0;

    if (speed > 100.0)
        speed = 100.0;

    return (int)((speed / 100.0) * 255.0);
}

// Gets the number of wheel ticks needed to turn an angle
unsigned long computeDeltaTicks(float ang)
{
    unsigned long ticks = (unsigned long)((ang * AlexCirc * COUNTS_PER_REV) / (360.0 * WHEEL_CIRC));
    return ticks;
}

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

/*

   ##### Alex Communication Routines.

*/

// Read the serial port
int readSerial(char *buffer)
{

    // int count = 0;

    // while (Serial.available())
    //     buffer[count++] = Serial.read();

    // return count;
    int count = 0;

    for (; dataAvailable(&rxBuffer); ++count)
    {
        readBuffer(&rxBuffer, (unsigned char *)&buffer[count]);
    }
    return count;
}

// Write to the serial port
void writeSerial(const char *buffer, int len)
{
    // Serial.write(buffer, len);
    for (int i = 1; i < len; ++i)
    {
        writeBuffer(&txBuffer, buffer[i]);
    }

    // Enable and trigger USART_UDRE interrupt
    UDR0 = buffer[0];
    UCSR0B |= 0b100000;
}

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

void handleCommand(TPacket *command)
{
    int param1, param2;
    switch (command->command)
    {
    // For movement commands, param[0] = distance, param[1] = speed.
    case COMMAND_FORWARD:
        sendOK();
        param1 = command->params[0] + '0';
        param2 = command->params[1] + '0';
        if (param1 >= 48 && param2 >= 48 && param1 <= 148 && param2 <= 148)
        {
            hasStopped = false; // each forward motion needs to check obstacle distance
            forward((float)command->params[0], (float)command->params[1]);
        }
        else
        {
            dbprint("INVALID PARAMETERS");
        }
        break;

    case COMMAND_REVERSE:
        sendOK();
        param1 = command->params[0] + '0';
        param2 = command->params[1] + '0';
        if (param1 >= 48 && param2 >= 48 && param1 <= 148 && param2 <= 148)
        {
            reverse((float)command->params[0], (float)command->params[1]);
        }
        else
        {
            dbprint("INVALID PARAMETERS");
        }
        break;

    case COMMAND_TURN_LEFT:
        sendOK();
        param1 = command->params[0] + '0';
        param2 = command->params[1] + '0';
        if (param1 >= 48 && param2 >= 48 && param1 <= 148 && param2 <= 148)
        {
            left((float)command->params[0], (float)command->params[1]);
        }
        else
        {
            dbprint("INVALID PARAMETERS");
        }
        break;

    case COMMAND_TURN_RIGHT:
        sendOK();
        param1 = command->params[0] + '0';
        param2 = command->params[1] + '0';
        if (param1 >= 48 && param2 >= 48 && param1 <= 148 && param2 <= 148)
        {
            right((float)command->params[0], (float)command->params[1]);
        }
        else
        {
            dbprint("INVALID PARAMETERS");
        }
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

/*

   ##### Alex ISRs

*/

// Functions to be called by INT0 and INT1 ISRs.
void leftISR()
{
    double val = getUltrasonicReading();
    if (val < THRESHOLD_DISTANCE && !hasStopped)
    {
        stop();
        hasStopped = true; // to allow for subsequent commands to trigger movements(reverse) after autostop
        return;
    }
    // char x[8];
    // dtostrf(val, 5, 2, x);
    // dbprint(x);
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

ISR(INT0_vect)
{
    leftISR();
}

ISR(INT1_vect)
{
    rightISR();
}

// ISR for serial read
ISR(USART_RX_vect)
{
    unsigned char data = UDR0;
    writeBuffer(&rxBuffer, data);
}

// ISR for serial write
ISR(USART_UDRE_vect)
{
    unsigned char data;
    TBufferResult result = readBuffer(&txBuffer, &data);

    if (result == BUFFER_OK)
    {
        UDR0 = data;
    }
    else if (result == BUFFER_EMPTY)
    {
        UCSR0B &= 0b11011111;
    }
}

/*

   ##### Alex setup and start codes

*/

// Enable pull up resistors on pins 2 and 3
void enablePullups()
{
    // Enable the pull-up resistors on pins
    // 2 and 3. These are pins PD2 and PD3 respectively.
    // We set bits 2 and 3 in DDRD to 0 to make them inputs.
    DDRD &= 0b11110011;

    // setting pins 2 and 3 to HIGH
    PORTD |= 0b00001100;
}
// Set up the external interrupt pins INT0 and INT1
// for falling edge triggered
void setupEINT()
{
    // Configure pins 2 and 3 to be
    // falling edge triggered. Remember to enable
    // the INT0 and INT1 interrupts.
    EICRA |= 0b00001010;
    EIMSK |= 0b00000011;
}
// Set up the serial connection
void setupSerial()
{
    // Serial.begin(9600);

    // Initialise send and receive buffer to appropriate length(in BUF_LEN macro)
    initBuffer(&txBuffer, PACKET_SIZE); // PACKET_SIZE = 140
    initBuffer(&rxBuffer, PACKET_SIZE);

    // Set bits[2:1] for data size of 8 bits
    // Clear bit 3 for 1 stop bit
    // Clear bits[5:4] to disable parity
    // Clear bits [7:6] to select async mode
    UCSR0C = 0b00000110;
    // Clear OCSR0A for to disable double-speed and multiprocessor modes
    UCSR0A = 0;

    // Set baudrate
    unsigned int baudrate_config = get_baud_rate();
    UBRR0H = (unsigned char)(baudrate_config >> 8);
    UBRR0L = (unsigned char)baudrate_config;
}

// Start the serial connection
void startSerial()
{
    // Set RXCIE0 and UDRIE0 bits(5 and 7) to enable interrupts for Receive Complete and USART Data Register Empty conditions.
    // Set  RXEN0 and TXEN0 bits (3 and 4) to enable the receiver and transmitter
    UCSR0B = 0b10111000;
}

// Our pin set up is reversed as the we take the back of Alex as our front
void setupMotors()
{
    /* Our motor set up is:
          A1IN - Pin 6, PD6, OC0A == LF
          A2IN - Pin 5, PD5, OC0B == LR
          B1IN - Pin 10, PB2, OC1B == RF
          B2In - Pin 9, PB1, OC1A == RR
    */
    // Set PD5/6
    DDRD |= (PIN6 | PIN5);
    // Initialize timer0 and OCR0A/B to 0
    TCNT0 = 0;
    OCR0A = 0;
    OCR0B = 0;
    // Set prescalar value of 64
    TCCR0B = 0b00000011;

    // Set PB2/3
    DDRB |= (PIN9 | PIN10);
    // Initialize timer1 and OCR1A/B to 0
    TCNT1 = 0;
    OCR1A = 0;
    OCR1B = 0;
    // Set prescalar value of 64
    TCCR1B = 0b00000011;
}

// Start the PWM for Alex's motors.
void startMotors()
{
    // Select phase correct pwm, clear OC0A/B on compare match
    TCCR0A = 0b10100001;
    // Select phase correct pwm, clear OC1A/B on compare match
    TCCR1A = 0b10100001;
}

void initializeState()
{
    clearCounters();
}

void setupUltrasonic()
{
    DDRB |= 0b00010000; // PB4
    // pinMode(TRIGGER_PIN, OUTPUT);
    PORTB &= 0b11101111;
    // digitalWrite(TRIGGER_PIN, LOW);
    DDRB |= 0b00100000; // PB5
    // pinMode(ECHO_PIN, INPUT);
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
    setupUltrasonic();
    sei();
}

/*

   ##### Alex motor controls

*/

// Move Alex forward "dist" cm at speed "speed"
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
        baremetal_analog_write(RF, (int)(val * 0.72));
        baremetal_analog_write(LF, (int)(val));
        baremetal_analog_write(LR, 0);
        baremetal_analog_write(RR, 0);
    }
    else if (dist == 10 && speed == 90)
    {
        baremetal_analog_write(RF, (int)(val));
        baremetal_analog_write(LF, (int)(val));
        baremetal_analog_write(LR, 0);
        baremetal_analog_write(RR, 0);
    }
    else if (dist == 5)
    {
        baremetal_analog_write(RF, (int)(val * 0.72));
        baremetal_analog_write(LF, (int)(val));
        baremetal_analog_write(LR, 0);
        baremetal_analog_write(RR, 0);
    }

    else if (dist == 40 && speed == 90)
    {
        baremetal_analog_write(RF, (int)(val));
        baremetal_analog_write(LF, (int)(val));
        baremetal_analog_write(LR, 0);
        baremetal_analog_write(RR, 0);
    }

    else
    {

        baremetal_analog_write(RF, (int)(val * 0.72));
        baremetal_analog_write(LF, (int)(val));
        baremetal_analog_write(LR, 0);
        baremetal_analog_write(RR, 0);
    }
}

// Reverse Alex "dist" cm at speed "speed"
void reverse(float dist, float speed)
{
    // Code to rell us how far to move
    if (dist == 0)
    {
        deltaDist = 9999999;
    }
    else
    {
        deltaDist = (int)(dist * 0.78);
    }
    newDist = reverseDist + deltaDist;
    dir = BACKWARD;
    int val = pwmVal(speed);
    if (dist == 10)
    {
        baremetal_analog_write(RR, (int)(val * 0.70));
        baremetal_analog_write(LR, (int)(val));
        baremetal_analog_write(LF, 0);
        baremetal_analog_write(RF, 0);
    }

    else if (dist == 5)
    {
        baremetal_analog_write(RR, (int)(val * 0.78));
        baremetal_analog_write(LR, (int)(val));
        baremetal_analog_write(LF, 0);
        baremetal_analog_write(RF, 0);
    }

    else
    {
        baremetal_analog_write(RR, (int)(val * 0.78));
        baremetal_analog_write(LR, (int)(val));
        baremetal_analog_write(LF, 0);
        baremetal_analog_write(RF, 0);
    }
}

// Turn Alex left "ang" degrees at speed "speed"
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

    baremetal_analog_write(LR, val);
    baremetal_analog_write(RF, (int)val * 0.85);
    baremetal_analog_write(LF, 0);
    baremetal_analog_write(RR, 0);
}

// Turn Alex right "ang" degrees at speed "speed".
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
    baremetal_analog_write(RR, (int)val * 0.78);
    baremetal_analog_write(LF, val);
    baremetal_analog_write(LR, 0);
    baremetal_analog_write(RF, 0);
}

// stop alex
void stop()
{
    dir = STOP;

    baremetal_analog_write(LF, 0);
    baremetal_analog_write(LR, 0);
    baremetal_analog_write(RF, 0);
    baremetal_analog_write(RR, 0);
}

/*
   ##### Alex's main loop

*/

void loop()
{

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
                stop();
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
