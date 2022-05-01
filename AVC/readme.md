# Firmware Design: Arduino Uno

## Serial Communications with RPi

We use U(S)ART communications with interrupts.

### USART with interrupts

#### Buffer library

In this project, we use a buffer library provided by the prof(`buffer.cpp`), which has an implementation for creating, reading and writing to a circular buffer.

The methods we use are mainly:

- `initBuffer(TBuffer *buffer, unsigned int size)`: To create a circular buffer; must be called before calling `readBuffer` or `writeBuffer`
- `writeBuffer(TBuffer *buffer, unsigned char data)`: Writes data to buffer and responds with `BUFFER_OK` if space is available. Discards data and returns `BUFFER_FULL` if space is not available. Discards data and returns `BUFFER_INVALID` if buffer was not initialized i.e. `initBuffer()` was not called.
- `readBuffer(TBuffer *buffer, unsigned char *data)`: Reads from buffer and stores in \*data as a queue(FIFO). If data is available for reading, responds with `BUFFER_OK`. If no data is available, returns `BUFFER_EMPTY`. If buffer was not initialized using `initBuffer()`, returns `BUFFER_INVALID`
- `dataAvailable(TBuffer *buffer)`: returns true if `buffer->count` is greater than 0 i.e. there is data to be read

#### Serial library

We also use a serialize library(`serialize.cpp`), which we use to serialize and deserialize data packet that we transfer with USART

The methods we use are mainly:

- `serialize()`: serialize a packet of type `TPacket` and return its length
- `deserialize()`: deserialize a packet and return a `TResult`

#### High level steps

1\. Setup serial communications(pin config)

2\. Define ISRs

3\. Handling reading of packet

4\. Handling writing of packet

#### Setup for serial communications

- Setup pin config

  ```
  void setupSerial()
  {
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
  ```

- Set up interrupt service routines(ISRs)

  ```
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
  ```

  - USART_RX triggers on receive complete i.e. whenenever Uno receives some data from RPi
    - This ensures rxBuffer is always up to date
  - USART_UDRE triggers when data register is empty i.e. Uno is able to transfer data to RPi
    - Even if Uno is able to transfer data, it may not want to i.e. we need to check that `txBuffer` is not empty

- Start serial(enabling ISRs)

  ```
  // Start the serial connection
  void startSerial()
  {
      // Set RXCIE0 and UDRIE0 bits(5 and 7) to enable interrupts for Receive Complete and USART Data Register Empty conditions.
      // Set  RXEN0 and TXEN0 bits (3 and 4) to enable the receiver and transmitter
      UCSR0B = 0b10111000;
  }
  ```

- Reading

  - The main loop continously calls `readPacket`

    ```
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
    ```

    - `readPacket` will first call `readSerial`, which returns a count

      ```
      int readSerial(char *buffer)
      {
          int count = 0;

          for (; dataAvailable(&rxBuffer); ++count)
          {
              readBuffer(&rxBuffer, (unsigned char *)&buffer[count]);
          }
          return count;
      }
      ```

    - So long as count is >0, some data was passed from serial port, then `readPacket` calls `deserialize()` which returns a `TResult`
    - If result<`TResult`> is `PACKET_OK`, then it will `handlePacket()`

- Writing

  - Whenever we want to send some data, we first create a TPacket and pass in some data

    ```
    void sendResponse(TPacket *packet)
    {
        // Takes a packet, serializes it then sends it out
        // over the serial port.
        char buffer[PACKET_SIZE];
        int len;

        len = serialize(buffer, packet, sizeof(TPacket));
        writeSerial(buffer, len);
    }
    ```

  - It will serialize the packet, then call `writeSerial()`
  - `writeSerial` will call `writeBuffer` from `buffer.cpp` which will update `txBuffer`

    ```
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
    ```

  - Then, we trigger the USART_UDRE interrupt to write to the `UDR0` register(see the ISR above)

### Handling Packets

- In the above, when we receive a packet, we mentioned that if `TResult == PACKET_OK`, we call `handlePacket()`

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

- Handle packet checks `packet->packetType`, then calls `handleCommand` where applicable

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

- `handleCommand` checks for the specific command which is in `command->command` and will extract the parameters associated with the packet in `command->params[]`
- Lastly, it will call the relevant command handler, such as `forward()` or `reverse()`

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

- Much later, we updated analogWrite() to use full baremetal programming

  - First, we update setupMotors to use timers()

    ```
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
    ```

  - Next, we start the timers
    ```
    // Start the PWM for Alex's motors.
    void startMotors()
    {
        // Select phase correct pwm, clear OC0A/B on compare match
        TCCR0A = 0b10100001;
        // Select phase correct pwm, clear OC1A/B on compare match
        TCCR1A = 0b10100001;
    }
    ```
  - This will set up PWM output at OC0A/B and OC1A/B based on the OCR0A/B and OCR1A/B values
  - Then in our baremetal version of analog write, we simply set the value of OCRnX
    ```
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
    ```
