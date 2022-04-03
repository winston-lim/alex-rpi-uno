# Arduino Uno

## Serial Communication with RPi

### Packet Structure(TPacket)

```
typedef struct
{

	char packetType;
	char command;
	char dummy[2]; // Padding to make up 4 bytes
	char data[MAX_STR_LEN]; // String data
	uint32_t params[16];
} TPacket;
```

### 1. Reading for packets in main loop()

```
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
```

### 2. Handling Packets

```
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
```

### 3. Handling commands(motor control)

```
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
```

## Distance and speed controls

Distance and speed are parameter values passed to the AVC through serial communications

When we receive a TPacket with `TPacket.packetType === PACKET_TYPE_COMMAND`, we first extract `TPacket.command` which tells us the motion e.g. `COMMAND_FORWARD`

If the command is for some motor control, we extract parameters from `TPacket.params`, which is an array of `uint32_t[]`

We then cast these values into `float` and pass it into its respective motor control handler function e.g. `forward()`

### Distance

- Hardware specifications
  - 2\* Wheel encoders(hall mark sensor) has 8 magnetic poles on the disk magnets
  - 2\* Motor gearbox has a 48:1 ratio
- Calibration/Setup

  - Manually obtain the number of ticks for a full revolution - this was found to be `160`
  - The wheel encoders are connected to `PD2` and `PD3` which corresponds to `INT0` and `INT1`
  - Set the respective ISRs to be triggered by a **falling edge** i.e. whenever a magnetic pole of the disk magnet crosses the wheel encoder
  - Each ISR execution increments a global state to maintain a count on the number of ticks in any motion
    - `leftForwardTicks` and `rightForwardTicks`
  - In order to keep track of the distance travelled by each motor, create a global state `forwardDist` and `reverseDist` that are similarly computed as such:

    ```
    forwardDist = (unsigned long)((float)rightForwardTicks / COUNTS_PER_REV * WHEEL_CIRC);
    ```

- Distance control
  - Our system needs to be able to move and stop continuously in its lifecycle, while correctly updating `forwardDist` and `reverseDist`
  - We define two global states, `deltaDist` and `newDist` to do this
    - Let `deltaDist` represent desired distance travelled in a motion
    - Let `newDist` represent the desired **total** distance
    - In our main `loop()` keep checking `if (forwardDist > newDist)`
    - If so, call `stop()` which will set the respective motor output pins to `LOW`

### Speed control

- In any particular motor control handler, e.g. `reverse()`, we receive two arguments `<someHandler>(float dist, float speed)`
- Speed is expressed as a percentage value i.e. must be less than 100.00
- We then obtain its respective PWM value i.e. out of 255 for an Uno where registers are `8bits`

  ```
  int pwmVal(float speed)
  {
    if (speed < 0.0)
      speed = 0;

    if (speed > 100.0)
      speed = 100.0;

    return (int)((speed / 100.0) * 255.0);
  }
  ```

- Lastly, using its associated pwmValue, we pass it into `analogWrite()`

  ```
  analogWrite(LF, pwmValue(speed));
  ```
