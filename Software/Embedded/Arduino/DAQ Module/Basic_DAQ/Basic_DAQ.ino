/* 
* Basic_DAQ
*
* This sketch runs the Basic Data Acquisition module HW. It is currently a prototype project
* to explore data acquision priciples using arduino based HW, in this case an ATmega32u4
* Microcontroller.
*
* This sketch is designed to run on the Basic DAQ HW v0.1.
* Code development uses the Arduino IDE.
* Pinouts are referenced to the Adafruit Itsy Bitsy proto board.
*
* Created: 6/16/2025
* Author: Gerard L. Muir 
*
*/
#include <string.h>

const byte kNumChars = 32;                      // Size of serial read character buffer array.
const int kStreamBufferSize = 128;              // Size of streaming caracter buffer array.
const long kMinimumSampleInterval = 1000;       // Current minimum sample interval in microseconds is 1000 or 1k samples per second max rate.
const long kDefaultSampleInterval = 100000;     // 100000 microseconds = 10 samples per second (Default sample interval).
const unsigned long kSerialPortSpeed = 576000;  // The serial port speed in bits per second.

bool gotNewDataPacket = false;               // Indicates the begining of a new data packet.
bool commandIsAvailable = false;             // Do we have a command to process.
bool input_state[4];                         // The High/Low state of the digital input ports.
bool outputVoltageData = true;               // Stream formatted voltage datat or raw ADC data.
bool streamData = false;                     // Stream acquired measurement data to the serial port.
bool countStarted = false;                   // Collect edge counts.
bool publishTheCount = false;                // Broadcast the counter value as it changes.
volatile bool calculatePulsePeriod = false;  // Should the pulse period be caluculated.

char streamBuffer[kStreamBufferSize];  // Serial output stream buffer
char receivedChars[kNumChars];         // Character array of recieved Serial.read data.
char voltageCharacterArray[6][10];     // Voltage readings as characters.

int ADC_Value[6];                     // Integer values from ADC.
int txLedState = LOW;                 // USB Transmit and Recieve indicator.
volatile int sampleNumber = 0;        // Counter for thraking the samples taken.
volatile int edgeCountMode = CHANGE;  // The edge mode for counting pulses.

float aRefVoltage = 2.04;                  // Analog Reference Voltage.
float voltageFactor = aRefVoltage / 1024;  // 10 bit resolution is 2 to the 10th
float inputScaleFactor = .2;               // Vout = Vin*(r2/r1+r2). (5k/(5k+20k) 10V Max input voltage.  - v0.1 board
float voltage[6];                          // Calculated voltages

long txBlinkInterval = 5000;                   // 5 mili seconds = 5,000 micro seconds.
long sampleInterval = kDefaultSampleInterval;  // The current sample interval in microseconds.

unsigned long currentTime = micros();             // The current time.
unsigned long previousTxBlinkTime = currentTime;  // The previous TX/RX LED blink time. TX/RX timer.
unsigned long previousSampleTime = currentTime;   // The time in microseconds of the previous sample.

volatile unsigned long lastCountTime = currentTime;  // The last time a count was triggered.
volatile unsigned long sampleTime = currentTime;     // The time that the sample was taken in microseconds.
volatile unsigned long edgeCount = 0;                // The number of times an edge was detected.
volatile unsigned long intervalAccumulator = 0;      // The total of all interval recorded interval times.
volatile unsigned long countInterval = 0;            // The period in microseconds of the counter.
volatile unsigned long averageCountInterval = 0;     // The caluculated average interval between counts.

int64_t secondsSinceEpoc = 0;  // (i64) Seconds since the epoch 01/01/1904 00:00:00.00 UTC (using the Gregorian calendar and ignoring leap seconds)

// Port map.
int rx_tx = 16;              // TX/RX LED on ItsyBitsy MOSI pin 16
int pwr_led = 14;            // Power/Status LED on ItsyBitsy MISO pin 14.
int dio0 = 7;                // Digital Input/Output dio0 on ItsyBitsy INT6 pin 7
int dio1 = 11;               // Digital Input/Output dio1 on ItsyBitsy PCINT7 pin 11
int dio2 = 17;               // Digital Input/Output dio2 on ItsyBitsy SS (Defined as Pin 17, Not Available on board)
int dio3 = 30;               // Digital Input/Output dio3 on ItsyBitsy on PD5 (Defined as Pin 30, Not Available on board)
int ai0 = A3;                // Analog Input a0, on ItsyBitsy A3 pin 21
int ai1 = A2;                // Analog Input a1, on ItsyBitsy A2 pin 20
int ai2 = A1;                // Analog Input a2, on ItsyBitsy A1 pin 19
int ai3 = A0;                // Analog Input a3, on ItsyBitsy A0 pin 18
int ai4 = A6;                // Analog Input a4, on ItsyBitsy A6 pin 4
int ai5 = A11;               // Analog Input a5, ItsyBitsy pin A11 pin 12
int triggerPort = 7;         // dio0 can alternately be used as the trigger port. 
int counterPort = 11;        // dio1 can alternately be used as the counter port.
int countSamples = 20;       // Number of counter samples to average.

int analogPort[6] = { ai0, ai1, ai2, ai3, ai4, ai5 };              // List of analog port numbers.
int digital_IO_Port[4] = { dio0, dio1, dio2, dio3 };               // list of digital port numbers.
int digital_IO_PortDirection[4] = { INPUT, INPUT, INPUT, INPUT };  // Digital port dirction. Default to input.

void setup() {

  // Setup and initialize IO ports.
  pinMode(rx_tx, OUTPUT);                                    // Set the LED pin as output
  pinMode(pwr_led, OUTPUT);                                  // Set the LED pin as output
  pinMode(digital_IO_Port[0], digital_IO_PortDirection[0]);  // Set the digital read pin as input
  pinMode(digital_IO_Port[1], digital_IO_PortDirection[1]);  // Set the digital read pin as input
  pinMode(digital_IO_Port[2], digital_IO_PortDirection[2]);  // Set the digital read pin as input
  pinMode(digital_IO_Port[3], digital_IO_PortDirection[3]);  // Set the digital read pin as input
  pinMode(analogPort[0], INPUT);                             // Set as analog input.
  pinMode(analogPort[1], INPUT);                             // Set as analog input.
  pinMode(analogPort[2], INPUT);                             // Set as analog input.
  pinMode(analogPort[3], INPUT);                             // Set as analog input.
  pinMode(analogPort[4], INPUT);                             // Set as analog input.
  pinMode(analogPort[5], INPUT);                             // Set as analog input.

  digitalWrite(rx_tx, LOW);     // Initialize the LED to off.
  digitalWrite(pwr_led, HIGH);  // Initialize the LED to On.

  analogReference(EXTERNAL);       // Uses an external voltage source for the ADC reference voltage.
  Serial.begin(kSerialPortSpeed);  // Open the serial port.

  // Enable interrupts on the PB port to use PCINT7.
  PCICR |= (1 << PCIE0);  // (Same as PCICR = PCICR | (1<<PCIE0);)
}


void loop() {

  currentTime = micros();
  getIncommingSerialData();
  if (commandIsAvailable == true) processInComingSerialCommands();

  // Blink the RX/TX led off after a message was received or after a message was sent.
  if (currentTime - previousTxBlinkTime > txBlinkInterval) {
    previousTxBlinkTime = currentTime;
    digitalWrite(rx_tx, LOW);
  }

  if (streamData == true) {
    readInputData(currentTime);
  }

}  //loop

/**
*  @brief Turn on the Transmit/Receive indicator LED.
**/
void tx_rx_LED_on() {
  digitalWrite(rx_tx, HIGH);
}

/**
*  @brief Sets the streaming sample rate in micro seconds using the current
*  command string in receivedChars array.
**/
void setAnalogSampleRate() {

  double microSeconds;

  String commandString(receivedChars);
  int commandLength = commandString.length();
  String microSecondsString(commandString.substring(1, commandLength));
  microSeconds = microSecondsString.toInt();
  if (microSeconds >= kMinimumSampleInterval) sampleInterval = microSeconds;
}

/**
*  @brief Fills the receivedChars character array with one or more characters read from the serial port.
* A '<' is used to denote the begining of data and a ">' is used to denote the end of data."
**/
void getIncommingSerialData() {

  static boolean recvInProgress = false;
  static byte ndx = 0;
  boolean gotNewDataPacket = false;
  char startMarker = '<';
  char endMarker = '>';
  char receivedCharacter;

  while (Serial.available() > 0 && gotNewDataPacket == false) {

    tx_rx_LED_on();
    receivedCharacter = Serial.read();

    if (recvInProgress == true) {
      if (receivedCharacter != endMarker) {
        receivedChars[ndx] = receivedCharacter;
        ndx++;
        if (ndx >= kNumChars) {
          ndx = kNumChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        gotNewDataPacket = true;
        commandIsAvailable = true;
      }
    } else if (receivedCharacter == startMarker) {
      recvInProgress = true;
    }
  }
}

/**
*  @brief Parses the incomming serial command array and executes the resulting command.
**/
void processInComingSerialCommands() {

  char commandChar;
  commandIsAvailable = false;

  if (!isNullTerminator(receivedChars[0])) {
    commandChar = receivedChars[0];
    if (commandChar == 's') streamData = true;
    if (commandChar == 'h') streamData = false;
    if (commandChar == 'v') outputVoltageData = true;
    if (commandChar == 'b') outputVoltageData = false;
    if (commandChar == 'w') writeDigitalPin();
    if (commandChar == '*') processStarCommand();
    if (streamData == false) {  // These commands are allowed only if we are not streaming data.
      if (commandChar == 'f') setAnalogSampleRate();
      if (commandChar == 'o') setDigitalPinAsOutput();
      if (commandChar == 'i') setDigitalPinAsInput();
      if (commandChar == 'e') enablePullUpResistor();
      if (commandChar == 'd') disablePullUpResistor();
      if (commandChar == 'a') getAnalogPinData();
      if (commandChar == 'r') readDigitalPin();
      if (commandChar == 'e') readDigitalPins();
      if (commandChar == 'g') set_trigger();
      if (commandChar == 'j') clear_trigger();
      if (commandChar == 'k') set_counter();
      if (commandChar == 'l') clear_counter();
      if (commandChar == 'm') startCount();
      if (commandChar == 'n') countStarted = false; // Stop counter.
      if (commandChar == 'p') readCount();
      if (commandChar == 'q') publishCount();
      if (commandChar == 'u') edgeCount = 0;  // Reset count.
      if (commandChar == 'x') ReadPulsePeriod();
    }
  }
}

/**
*  @brief Read and stream the input measurement data.
*
*  @param currentTime - The current microseconds time count. 
**/
void readInputData(unsigned long currentTime) {

  if (streamData == true) {
    if (currentTime - previousSampleTime > sampleInterval) {

      tx_rx_LED_on();
      previousSampleTime = currentTime;
      readAnalogPins();
      readAllDigitalPins();

      if (outputVoltageData == true) {
        formatAnalogVoltageData();
        sprintf(streamBuffer, "A0:%sA1:%sA2:%sA3:%sA4:%sA5:%sD0:%1dD1:%1dD2:%1dD3:%1d",
                voltageCharacterArray[0],
                voltageCharacterArray[1],
                voltageCharacterArray[2],
                voltageCharacterArray[3],
                voltageCharacterArray[4],
                voltageCharacterArray[5], input_state[0], input_state[1], input_state[2], input_state[3]);
      } else {
        // Raw analog DAC values.
        sprintf(streamBuffer, "A0:%4dA1:%4dA2:%4dA3:%4dA4:%4dA5:%d4D0:%1dD1:%1dD2:%1dD3:%1d",
                ADC_Value[0],
                ADC_Value[1],
                ADC_Value[2],
                ADC_Value[3],
                ADC_Value[4],
                ADC_Value[5], input_state[0], input_state[1], input_state[2], input_state[3]);
      }
      Serial.println(streamBuffer);
    }
  }
}

/**
*  @brief Set specified digital IO pin mode to Input.
**/
void setDigitalPinAsInput() {

  int pin = getPinNumber(receivedChars[1]);
  if (isValidDigitalPin(pin)) {
    digital_IO_PortDirection[pin] = INPUT;
    pinMode(pin, INPUT);
  }
}

/**
*  @brief Set specified digital IO pin mode to Input.
**/
// Set specified digital IO pin mode to output
void setDigitalPinAsOutput() {

  int pin = getPinNumber(receivedChars[1]);
  if (isValidDigitalPin(pin)) {
    digital_IO_PortDirection[pin] = OUTPUT;
    pinMode(pin, OUTPUT);
  }
}

/**
*  @brief Enable the pullup resistor for the spcified digital input lines.
**/
void enablePullUpResistor() {

  int pin = getPinNumber(receivedChars[1]);
  if (isValidDigitalPin(pin)) {
    if (digital_IO_PortDirection[pin] == INPUT) {
      pinMode(digital_IO_Port[pin], INPUT_PULLUP);
    }
  }
}

/**
*  @brief Disable the pullup resistor for the specified digital input line.
**/
void disablePullUpResistor() {

  int pin = getPinNumber(receivedChars[1]);
  if (isValidDigitalPin(pin)) {
    if (digital_IO_PortDirection[pin] == INPUT) {
      pinMode(digital_IO_Port[pin], INPUT);
    }
  }
}

/**
*  @brief Extract a pin number from the command caracters.
*
*  @param pinChar - The pin number character.
*
*  @return int - The integer value of the pin number. -1 for invalid pin.
**/
int getPinNumber(char pinChar) {

  int pin = -1;

  if (!isNullTerminator(pinChar)) {
    if (isDigit(pinChar)) {
      String pinString(pinChar);
      pin = pinString.toInt();
      if (pin < 0 || pin > 3) pin = -1;
    }
  }
  return pin;
}

/**
*  @brief Read the current value of the specified digital input pin and
*  output it to the serial port.
**/
void readDigitalPin() {

  int pin = getPinNumber(receivedChars[1]);
  if (isValidDigitalPin(pin)) {
    if (digital_IO_PortDirection[pin] == INPUT) {
      int input_state = digitalRead(digital_IO_Port[pin]);
      sprintf(streamBuffer, "D%1d:%1d", pin, input_state);
      Serial.println(streamBuffer);
    }
  }
}

// Read the current value of all digital ports.
/**
*  @brief Read the current value of all digital ports.
**/
void readAllDigitalPins() {

  for (int portNum = 0; portNum <= 3; portNum++) {
    input_state[portNum] = digitalRead(digital_IO_Port[portNum]);
  }
}

/**
*  @brief Read the current value of all digital ports and write the results to the serial ouput.
**/
void readDigitalPins() {

  readAllDigitalPins();
  sprintf(streamBuffer, "D0:%1dD1:%1dD2:%1dD3:%1d", input_state[0], input_state[1], input_state[2], input_state[3]);
  Serial.println(streamBuffer);
}

/**
*  @brief Write the specified value to the given digital output pin if it is configured as an output.
**/
void writeDigitalPin() {

  if (!isNullTerminator(receivedChars[1])) {
    int pin = getPinNumber(receivedChars[1]);
    if (isValidDigitalPin(pin)) {
      if (digital_IO_PortDirection[pin] == OUTPUT) {
        if (receivedChars[2] != '\0') {
          String valString(receivedChars[2]);
          int val = valString.toInt();
          digitalWrite(digital_IO_Port[pin], val);
        }
      }
    }
  }
}


/**
*  @brief Process standardized VISA instrument commands.
**/
void processStarCommand() {

  String commandString(receivedChars);

  if (commandString.equals("*IDN?") & streamData == false) {  // Identify command.
    Serial.println("BDAQ-6104");
  }

  if (commandString.equals("*RST")) {  // Reset to default command.

    clear_trigger();
    clear_counter();
    setup();
    streamData = false;
    sampleInterval = kDefaultSampleInterval;
    outputVoltageData = true;
    streamData = false;
    countStarted = false;
    publishTheCount = false;
    calculatePulsePeriod = false;
    sampleNumber = 0;
    edgeCount = 0;
    intervalAccumulator = 0;
    countInterval = 0;

    Serial.end();
    Serial.begin(kSerialPortSpeed);
  }
}

/**
*  @brief Returns true if the pin number is not a -1.
*
*  @param pin - The pin number.
*
*  @return bool 
**/
bool isValidDigitalPin(int pin) {

  bool isValidPin = true;

  if (pin == -1) isValidPin = false;

  return isValidPin;
}

/**
*  @brief Returns true if the character is the null termination character of the string array.
*
*  @param char - The caracter to be evaluated.
*
*  @return bool
**/
bool isNullTerminator(char c) {

  bool isNullTerm = false;

  if (c == '\0') isNullTerm = true;

  return isNullTerm;
}

/**
*  @brief Read the value of all analog pins.
**/
void readAnalogPins() {

  for (int portNum = 0; portNum <= 5; portNum++) {
    ADC_Value[portNum] = analogRead(analogPort[portNum]);
  }
}

/**
*  @brief Reads all analog port values and writes them to the serial port.
**/
void getAnalogPinData() {

  readAnalogPins();
  if (outputVoltageData == true) {

    formatAnalogVoltageData();

    sprintf(streamBuffer, "A0:%sA1:%sA2:%sA3:%sA4:%sA5:%s",
            voltageCharacterArray[0],
            voltageCharacterArray[1],
            voltageCharacterArray[2],
            voltageCharacterArray[3],
            voltageCharacterArray[4],
            voltageCharacterArray[5]);
  } else {
    // Raw analog DAC values.
    sprintf(streamBuffer, "A0:%4dA1:%4dA2:%4dA3:%4dA4:%4dA5:%d",
            ADC_Value[0], ADC_Value[1], ADC_Value[2], ADC_Value[3], ADC_Value[4], ADC_Value[5]);
  }

  Serial.println(streamBuffer);
}

/**
*  @brief Process the ADC values to generate equivelent voltage values and places the resulting
*    string values in the floatString array.  
**/
void formatAnalogVoltageData() {

  for (int i = 0; i <= 5; i++) {
    voltage[i] = (ADC_Value[i] * voltageFactor) / inputScaleFactor;
    String floatString = String(voltage[i], 3);
    floatString.toCharArray(voltageCharacterArray[i], 5);
  }
}

/**
*  @brief Configures and activates the trigger  port.
**/
void set_trigger() {

  if (!isNullTerminator(receivedChars[1])) {
    if (isDigit(receivedChars[1])) {
      String isrString(receivedChars[1]);
      int isr_mode = isrString.toInt();
      if (isr_mode == 0) {
        pinMode(triggerPort, INPUT);
        attachInterrupt(digitalPinToInterrupt(triggerPort), triggerHandler, CHANGE);
      }
      if (isr_mode == 1) {
        pinMode(triggerPort, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(triggerPort), triggerHandler, FALLING);
      }
      if (isr_mode == 2) {
        pinMode(triggerPort, INPUT);
        attachInterrupt(digitalPinToInterrupt(triggerPort), triggerHandler, RISING);
      }
    }
  }
}

/**
*  @brief This Interrupt Service Routine writes the serial buffer with the trigger detected message
*  when a trigger event is fired. 
**/
void triggerHandler() {
  Serial.println("T0");
  tx_rx_LED_on();
}

/**
*  @brief Deactivates the trigger interrupt event detection.
**/
void clear_trigger() {
  detachInterrupt(digitalPinToInterrupt(triggerPort));
}

/**
*  @brief Configures the edge counter port. 
**/
void set_counter() {

  if (!isNullTerminator(receivedChars[1])) {
    if (isDigit(receivedChars[1])) {
      String isrString(receivedChars[1]);
      int isr_mode = isrString.toInt();
      if (isr_mode == 0) {
        edgeCountMode = CHANGE;
      }
      if (isr_mode == 1) {
        edgeCountMode = FALLING;
      }
      if (isr_mode == 2) {
        edgeCountMode = RISING;
      }
    }

    // Number of edges to count.
    if (!isNullTerminator(receivedChars[2])) {
      String commandString(receivedChars);
      int commandLength = commandString.length();
      String countSamplesString = String(commandString.substring(1, commandLength));
      countSamples = countSamplesString.toInt();
    }
  }
}

/**
*  @brief Deactivates the counter port interrupt. 
**/
void clear_counter() {
  PCMSK0 &= (0 << PCINT7);
}

/**
*  @brief Resets the edge count and starts the counter port edge detection function.
**/
void startCount() {

  edgeCount = 0;
  sampleNumber = 0;
  countStarted = true;
  PCIFR |= bit(PCIF0);      // clear any outstanding interrupts
  PCMSK0 |= (1 << PCINT7);  // Enable PCINT7 interrupt through the PC Mask register.
}

/**
*  @brief Writes the serical port with the current edge count data.
**/
void readCount() {

  sprintf(streamBuffer, "C: %20u", edgeCount);
  Serial.println(streamBuffer);
}

/**
*  @brief Activates or deactivates the streamming of the edge count to the serail ports.
**/
void publishCount() {

  if (!isNullTerminator(receivedChars[1])) {

    int publish_mode = getCharDigitValue(receivedChars[1]);
    if (publish_mode == 0) publishTheCount = false;
    if (publish_mode == 1) publishTheCount = true;
  }
}

// 
/**
*  @brief Returns the int value of a digit charater or -1 if inavalid.
*
*  @param char - The character string representing a digit.
*
*  @return int - The integer value of the character digit.
**/
int getCharDigitValue(char digitChar) {

  if (isDigit(digitChar)) {
    String digitString(digitChar);
    return digitString.toInt();
  } else {
    return -1;
  }
}

/**
*  @brief Activates the pulse period calculation routine.
**/
void ReadPulsePeriod() {

  calculatePulsePeriod = true;
}

// Arduino Builtin ISR handeler.
// Handels interrupts from the PCINT0 group( All 8 interrupts!).
/**
*  @brief Built in ISR handeler for the PCINT0 group( All 8 interrupts! PCINT0-7).
*  This ISR is run when a counter port event fires. It will start counting events
*  and calculate the pulse period if requested. The counter port is the only
*  port generating events in the PCINT0 group.
**/
ISR(PCINT0_vect) {

  if (countStarted == true) {
    // Filter the count by edge mode and return the period if requested.
    if (edgeCountMode == RISING) {
      if (digitalRead(counterPort) == 1) {
        sampleTime = micros();
        edgeCount++;
        calcPeriod();
        sendCountIfRequested();
      }
    } else if (edgeCountMode == FALLING) {
      if (digitalRead(counterPort) == 0) {
        sampleTime = micros();
        edgeCount++;
        calcPeriod();
        sendCountIfRequested();
      }
    } else {  // CHANGE
      sampleTime = micros();
      edgeCount++;
      calcPeriod();
      sendCountIfRequested();
    }
  }
}

/**
*  @brief  Strat writing the edge count on the counter port to the serial port.
**/
void sendCountIfRequested() {

  if (publishTheCount == true) {
    sprintf(streamBuffer, "C: %u", edgeCount);
    Serial.println(streamBuffer);
  }
}

/**
*  @brief This function will calculate the counter event period and write the resulting
*  counter value to the serial port when finished. The period is specified in micro seconds.
**/
void calcPeriod() {
  if (calculatePulsePeriod == true) {
    if (sampleNumber == 0) {
      lastCountTime = sampleTime;
      averageCountInterval = 0;
      sampleNumber++;
    } else {
      if (sampleNumber <= countSamples) {
        sampleNumber++;
        countInterval = sampleTime - lastCountTime;
        intervalAccumulator = intervalAccumulator + countInterval;
        lastCountTime = sampleTime;
      } else {
        // Return the period in micro seconds after the specified number of samples has been reached.
        // Frequency is 1/period when using RISING or FALLING edges.
        calculatePulsePeriod = false;
        averageCountInterval = intervalAccumulator / countSamples;
        intervalAccumulator = 0;
        sampleNumber = 0;
        sprintf(streamBuffer, "P: %u", averageCountInterval);
        Serial.println(streamBuffer);
        tx_rx_LED_on();
      }
    }
  }
}