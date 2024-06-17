   //Includes
// ******************************************************************************/
#include <OSCBoards.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <OSCMatch.h>
#include <OSCMessage.h>
#include <OSCTiming.h>
#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
#else
#include <SLIPEncodedSerial.h>
SLIPEncodedSerial SLIPSerial(Serial);
#endif
#include <LiquidCrystal.h>
#include <string.h>

/*******************************************************************************
   Macros and Constants
 ******************************************************************************/
#define LCD_CHARS           16
#define LCD_LINES           2   // Currently assume at least 2 lines

//#define SHIFT_BTN4             34
//#define SHIFT_BTN3            35
//#define SHIFT_BTN2            36
//#define SHIFT_BTN1            37
#define SHIFT_BTN           8

#define SUBSCRIBE           ((int32_t)1)
#define UNSUBSCRIBE         ((int32_t)0)

#define EDGE_DOWN           ((int32_t)1)
#define EDGE_UP             ((int32_t)0)

#define FORWARD             1
#define REVERSE             0

// Change these values to switch which direction increase/decrease pan/tilt
#define PAN_DIR             FORWARD
#define TILT_DIR            FORWARD
#define ZOOM_DIR            FORWARD
#define EDGE_DIR            FORWARD
#define WHITE_DIR            FORWARD


// Use these values to make the encoder more coarse or fine.
// This controls the number of wheel "ticks" the device sends to the console
// for each tick of the encoder. 1 is the default and the most fine setting.
// Must be an integer.
#define PAN_TILT_SCALE           1
#define ZOOM_EDGE_SCALE          1
#define RED_GREEN_SCALE          1
#define BLUE_AMBER_SCALE          1
#define WHITE_WHEEL_SCALE         1
#define sens                30
#define SIG_DIGITS          3   // Number of significant digits displayed

#define OSC_BUF_MAX_SIZE    512

const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";

//See displayScreen() below - limited to 10 chars (after 6 prefix chars)
#define VERSION_STRING      "2.0.0.1"

#define BOX_NAME_STRING     "box1"

// Change these values to alter how long we wait before sending an OSC ping
// to see if Eos is still there, and then finally how long before we
// disconnect and show the splash screen
// Values are in milliseconds
#define PING_AFTER_IDLE_INTERVAL    2500
#define TIMEOUT_AFTER_IDLE_INTERVAL 5000

/*******************************************************************************
   Local Types
 ******************************************************************************/
enum WHEEL_TYPE { EDGE_ZOOM, PAN_TILT, RED_GREEN, BLUE_AMBER, WHITE_WHEEL };
enum WHEEL_MODE { COARSE, FINE };

struct Encoder
{
  uint8_t pinA;
  uint8_t pinB;
  int pinAPrevious;
  int pinBPrevious;
  float pos;
  uint8_t direction;
};
struct Encoder panWheel;
struct Encoder tiltWheel;
struct Encoder zoomWheel;
struct Encoder edgeWheel;
struct Encoder whiteWheel;
struct Encoder intensityWheel;


enum ConsoleType
{
  ConsoleNone,
  ConsoleEos,
  ConsoleCobalt,
  ConsoleColorSource
};

/*******************************************************************************
   Global Variables
 ******************************************************************************/

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(120, 110, 125, 4, 13, 14);

bool updateDisplay = false;
ConsoleType connectedToConsole = ConsoleNone;
unsigned long lastMessageRxTime = 0;
bool timeoutPingSent = false;
int counter = 0;
bool push = false;
/*******************************************************************************
   Local Functions
 ******************************************************************************/

/*******************************************************************************
   Issues all our subscribes to Eos. When subscribed, Eos will keep us updated
   with the latest values for a given parameter.

   Parameters:  none

   Return Value: void

 ******************************************************************************/
/*void issueEosSubscribes()
{
  // Add a filter so we don't get spammed with unwanted OSC messages from Eos
  OSCMessage filter("/eos/filter/add");
  filter.add("/eos/out/param/*");
  filter.add("/eos/out/ping");
  SLIPSerial.beginPacket();
  filter.send(SLIPSerial);
  SLIPSerial.endPacket();

  // subscribe to Eos pan & tilt updates
  OSCMessage subPan("/eos/subscribe/param/pan");
  subPan.add(SUBSCRIBE);
  SLIPSerial.beginPacket();
  subPan.send(SLIPSerial);
  SLIPSerial.endPacket();

  OSCMessage subTilt("/eos/subscribe/param/tilt");
  subTilt.add(SUBSCRIBE);
  SLIPSerial.beginPacket();
  subTilt.send(SLIPSerial);
  SLIPSerial.endPacket();

    OSCMessage subZoom("/eos/subscribe/param/zoom");
  subZoom.add(SUBSCRIBE);
  SLIPSerial.beginPacket();
  subZoom.send(SLIPSerial);
  SLIPSerial.endPacket();

      OSCMessage subEdge("/eos/subscribe/param/edge");
  subEdge.add(SUBSCRIBE);
  SLIPSerial.beginPacket();
  subEdge.send(SLIPSerial);
  SLIPSerial.endPacket();
}/*/

/*******************************************************************************
   Given a valid OSCMessage (relevant to Pan/Tilt), we update our Encoder struct
   with the new position information.

   Parameters:
    msg - The OSC message we will use to update our internal data
    addressOffset - Unused (allows for multiple nested roots)

   Return Value: void

 ******************************************************************************/
void parseFloatPanUpdate(OSCMessage& msg, int addressOffset)
{
  panWheel.pos = msg.getOSCData(0)->getFloat();
  updateDisplay = true;
}

void parseFloatTiltUpdate(OSCMessage& msg, int addressOffset)
{
  tiltWheel.pos = msg.getOSCData(0)->getFloat();
  updateDisplay = true;
}

void parseFloatZoomUpdate(OSCMessage& msg, int addressOffset)
{
  zoomWheel.pos = msg.getOSCData(0)->getFloat();
  updateDisplay = true;
}

void parseFloatEdgeUpdate(OSCMessage& msg, int addressOffset)
{
  edgeWheel.pos = msg.getOSCData(0)->getFloat();
  updateDisplay = true;
}

void parseEos(OSCMessage& msg, int addressOffset)
{
  // If we don't think we're connected, reconnect and subscribe
  if (connectedToConsole != ConsoleEos)
  {
    //issueEosSubscribes();
    connectedToConsole = ConsoleEos;
    updateDisplay = true;
  }

  if (!msg.route("/out/param/pan", parseFloatPanUpdate, addressOffset))
    msg.route("/out/param/tilt", parseFloatTiltUpdate, addressOffset);
    msg.route("/out/param/zoom", parseFloatZoomUpdate, addressOffset);
    msg.route("/out/param/edge", parseFloatEdgeUpdate, addressOffset);
}

/******************************************************************************/

void parseCobalt(OSCMessage& msg, int addressOffset)
{
  // Cobalt doesn't currently send anything other than ping
  connectedToConsole = ConsoleCobalt;
  updateDisplay = true;
}

void parseColorSource(OSCMessage& msg, int addressOffset)
{
  // ColorSource doesn't currently send anything other than ping
  connectedToConsole = ConsoleColorSource;
  updateDisplay = true;
}

/*******************************************************************************
   Given an unknown OSC message we check to see if it's a handshake message.
   If it's a handshake we issue a subscribe, otherwise we begin route the OSC
   message to the appropriate function.

   Parameters:
    msg - The OSC message of unknown importance

   Return Value: void

 ******************************************************************************/
void parseOSCMessage(String& msg)
{
  // check to see if this is the handshake string
  if (msg.indexOf(HANDSHAKE_QUERY) != -1)
  {
    // handshake string found!
    SLIPSerial.beginPacket();
    SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
    SLIPSerial.endPacket();

    // An Eos would do nothing until subscribed
    // Let Eos know we want updates on some things
   // issueEosSubscribes();

    updateDisplay = true;
  }
  else
  {
    // prepare the message for routing by filling an OSCMessage object with our message string
    OSCMessage oscmsg;
    oscmsg.fill((uint8_t*)msg.c_str(), (int)msg.length());
    // route pan/tilt messages to the relevant update function

    // Try the various OSC routes
    if (oscmsg.route("/eos", parseEos))
      return;
    if (oscmsg.route("/cobalt", parseCobalt))
      return;
    if (oscmsg.route("/cs", parseColorSource))
      return;
  }
}

/*******************************************************************************
   Updates the display with the latest pan and tilt positions.

   Parameters:  none

   Return Value: void

 ******************************************************************************/
/*void displayStatus()
{
  lcd.clear();

  switch (connectedToConsole)
  {
    case ConsoleNone:
      {
        // display a splash message before the Eos connection is open
        lcd.setCursor(0, 0);
        lcd.print(BOX_NAME_STRING " v" VERSION_STRING);
        lcd.setCursor(0, 1);
        lcd.print("Waiting...");
      } break;

    case ConsoleEos:
      {
        // put the cursor at the begining of the first line
        lcd.setCursor(0, 0);
        lcd.print("Pan:  ");
        lcd.print(panWheel.pos, SIG_DIGITS);

        // put the cursor at the begining of the second line
        lcd.setCursor(0, 1);
        lcd.print("Tilt: ");
        lcd.print(tiltWheel.pos, SIG_DIGITS);
      } break;

    case ConsoleCobalt:
      {
        lcd.setCursor(7, 0);
        lcd.print("Cobalt");
        lcd.setCursor(0, 1);
        lcd.print("Pan");
        lcd.setCursor(12, 1);
        lcd.print("Tilt");
      } break;

    case ConsoleColorSource:
      {
        lcd.setCursor(2, 0);
        lcd.print("ColorSource");
        lcd.setCursor(0, 1);
        lcd.print("Pan");
        lcd.setCursor(12, 1);
        lcd.print("Tilt");
      } break;

  }

  updateDisplay = false;
}
*/
/*******************************************************************************
   Initializes a given encoder struct to the requested parameters.

   Parameters:
    encoder - Pointer to the encoder we will be initializing
    pinA - Where the A pin is connected to the Arduino
    pinB - Where the B pin is connected to the Arduino
    direction - Determines if clockwise or counterclockwise is "forward"

   Return Value: void

 ******************************************************************************/
void initEncoder(struct Encoder* encoder, uint8_t pinA, uint8_t pinB, uint8_t direction)
{
  encoder->pinA = pinA;
  encoder->pinB = pinB;
  encoder->pos = 0;
  encoder->direction = direction;

  pinMode(pinA, INPUT_PULLUP);
  pinMode(pinB, INPUT_PULLUP);

  encoder->pinAPrevious = digitalRead(pinA);
  encoder->pinBPrevious = digitalRead(pinB);
}

/*******************************************************************************
   Checks if the encoder has moved by comparing the previous state of the pins
   with the current state. If they are different, we know there is movement.
   In the event of movement we update the current state of our pins.

   Parameters:
    encoder - Pointer to the encoder we will be checking for motion

   Return Value:
    encoderMotion - Returns the 0 if the encoder has not moved
                                1 for forward motion
                               -1 for reverse motion

 ******************************************************************************/
int8_t updateEncoder(struct Encoder* encoder)
{
  int8_t encoderMotion = 0;
  int pinACurrent = digitalRead(encoder->pinA);
  int pinBCurrent = digitalRead(encoder->pinB);

  // has the encoder moved at all?
  if (encoder->pinAPrevious != pinACurrent)
  {
    // Since it has moved, we must determine if the encoder has moved forwards or backwards
    encoderMotion = (encoder->pinAPrevious == encoder->pinBPrevious) ? -1 : 1;

    // If we are in reverse mode, flip the direction of the encoder motion
    if (encoder->direction == REVERSE)
      encoderMotion = -encoderMotion;

    // Update the previous state of the pins
    encoder->pinAPrevious = pinACurrent;
    encoder->pinBPrevious = pinBCurrent;
  }

  return encoderMotion;
}

/*******************************************************************************
   Sends a message to Eos informing them of a wheel movement.

   Parameters:
    type - the type of wheel that's moving (i.e. pan or tilt)
    ticks - the direction and intensity of the movement

   Return Value: void

 ******************************************************************************/

void sendOscMessage(const String &address, float value)
{
  OSCMessage msg(address.c_str());
  msg.add(value);
  SLIPSerial.beginPacket();
  msg.send(SLIPSerial);
  SLIPSerial.endPacket();
}

void sendEosWheelMove(WHEEL_TYPE type, float ticks)
{
  String wheelMsg("/eos/wheel");

  // Determine if SHIFT_BTN is pressed
  bool push = (digitalRead(SHIFT_BTN) == LOW);


  // Determine which OSC message to send based on type and shift button state
  if (type == PAN_TILT)
    wheelMsg.concat(push ? "/TILT" : "/PAN");
  else if (type == EDGE_ZOOM)
    wheelMsg.concat(push ? "/edge" : "/zoom");
      else if (type == RED_GREEN)
    wheelMsg.concat(push ? "/GREEN" : "/RED");
          else if (type == BLUE_AMBER)
    wheelMsg.concat(push ? "/AMBER" : "/BLUE");
              else if (type == WHITE_WHEEL)
    wheelMsg.concat(push ? "/white" : "/WHITE");
  else
    return; // Handle other types of wheels as needed

  // Send the OSC message
  sendOscMessage(wheelMsg, ticks);
}

void sendCobaltWheelMove(WHEEL_TYPE type, float ticks)
{
  String wheelMsg("/cobalt/param");

  if (type == PAN_TILT)
    wheelMsg.concat("/pan/wheel");
  else if (type == EDGE_ZOOM)
    wheelMsg.concat("/tilt/wheel");
  else
    // something has gone very wrong
    return;

  if (digitalRead(SHIFT_BTN) != LOW)
    ticks = ticks * 16;

  sendOscMessage(wheelMsg, ticks);
}

void sendColorSourceWheelMove(WHEEL_TYPE type, float ticks)
{
  String wheelMsg("/cs/param");

  if (type == PAN_TILT)
    wheelMsg.concat("/pan/wheel");
  else if (type == EDGE_ZOOM)
    wheelMsg.concat("/tilt/wheel");
  else
    // something has gone very wrong
    return;

  if (digitalRead(SHIFT_BTN) != LOW)
    ticks = ticks * 2;

  sendOscMessage(wheelMsg, ticks);
}

/******************************************************************************/

void handleIntensityWheel(float ticks) {
  String wheelMsg("/eos");

  if (ticks > 0) {
    if (counter < sens) {
      counter++;
      return; // Skip sending the message until the counter is at the threshold
    } else {
      wheelMsg.concat("/key/+%");
      counter = 0; // Reset counter after reaching threshold
    }
  } else if (ticks < 0) {
    if (counter > -sens) {
      counter--;
      return; // Skip sending the message until the counter is at the threshold
    } else {
      wheelMsg.concat("/key/-%");
      counter = 0; // Reset counter after reaching threshold
    }
  } else {
    // No movement detected
    return;
  }

  // Send the OSC message
  OSCMessage msg(wheelMsg.c_str());
  SLIPSerial.beginPacket();
  msg.send(SLIPSerial);
  SLIPSerial.endPacket();
}
void sendWheelMove(WHEEL_TYPE type, float ticks)
{
  switch (connectedToConsole)
  {
    default:
    case ConsoleEos:
      sendEosWheelMove(type, ticks);
      break;
    case ConsoleCobalt:
      sendCobaltWheelMove(type, ticks);
      break;
    case ConsoleColorSource:
      sendColorSourceWheelMove(type, ticks);
      break;
  }
}

/*******************************************************************************
   Sends a message to the console informing them of a key press.

   Parameters:
    down - whether a key has been pushed down (true) or released (false)
    key - the OSC key name that has moved

   Return Value: void

 ******************************************************************************/
/*void sendKeyPress(bool down, const String &key)
{
  String keyAddress;
  switch (connectedToConsole)
  {
    default:
    case ConsoleEos:
      keyAddress = "/eos/key/" + key;
      break;
    case ConsoleCobalt:
      keyAddress = "/cobalt/key/" + key;
      break;
    case ConsoleColorSource:
      keyAddress = "/cs/key/" + key;
      break;
  }
  OSCMessage keyMsg(keyAddress.c_str());

  if (down)
    keyMsg.add(EDGE_DOWN);
  else
    keyMsg.add(EDGE_UP);

  SLIPSerial.beginPacket();
  keyMsg.send(SLIPSerial);
  SLIPSerial.endPacket();
}
*/
/*******************************************************************************
   Checks the status of all the relevant buttons (i.e. Next & Last)

   NOTE: This does not check the shift key. The shift key is used in tandem with
   the encoder to determine coarse/fine mode and thus does not report directly.

   Parameters: none

   Return Value: void

 ******************************************************************************/

/*void checkButtons()
{
  // OSC configuration
  const int keyCount = 2;
  const int keyPins[2] = {NEXT_BTN, LAST_BTN};
  const String keyNames[4] = {
    "NEXT", "LAST",
    "soft6", "soft4"
  };

  static int keyStates[2] = {HIGH, HIGH};

  // Eos and Cobalt buttons are the same
  // ColorSource is different
  int firstKey = (connectedToConsole == ConsoleColorSource) ? 2 : 0;

  // Loop over the buttons
  for (int keyNum = 0; keyNum < keyCount; ++keyNum)
  {
    // Has the button state changed
    if (digitalRead(keyPins[keyNum]) != keyStates[keyNum])
    {
      // Notify console of this key press
      if (keyStates[keyNum] == LOW)
      {
        sendKeyPress(false, keyNames[firstKey + keyNum]);
        keyStates[keyNum] = HIGH;
      }
      else
      {
        sendKeyPress(true, keyNames[firstKey + keyNum]);
        keyStates[keyNum] = LOW;
      }
    }
  }
}
/*
/*******************************************************************************
   Here we setup our encoder, lcd, and various input devices. We also prepare
   to communicate OSC with Eos by setting up SLIPSerial. Once we are done with
   setup() we pass control over to loop() and never call setup() again.

   NOTE: This function is the entry function. This is where control over the
   Arduino is passed to us (the end user).

   Parameters: none

   Return Value: void

 ******************************************************************************/
void setup()
{
  SLIPSerial.begin(115200);
  // This is a hack around an Arduino bug. It was taken from the OSC library
  //examples
#ifdef BOARD_HAS_USB_SERIAL
  while (!SerialUSB);
#else
  while (!Serial);
#endif

  // This is necessary for reconnecting a device because it needs some time
  // for the serial port to open. The handshake message may have been sent
  // from the console before #lighthack was ready

  SLIPSerial.beginPacket();
  SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
  SLIPSerial.endPacket();

  // If it's an Eos, request updates on some things
  //issueEosSubscribes();

  initEncoder(&panWheel, A1, A2, PAN_DIR);
  initEncoder(&tiltWheel, A3, A4, TILT_DIR);
  initEncoder(&zoomWheel, 9, 10, ZOOM_DIR);
  initEncoder(&edgeWheel, 6, 5, EDGE_DIR);
   initEncoder(&whiteWheel, A5, 7, WHITE_DIR);
  initEncoder(&intensityWheel, 11, 12, FORWARD); // Pins 8 and 9, direction FORWARD


  lcd.begin(LCD_CHARS, LCD_LINES);
  lcd.clear();
  //pinMode(SHIFT_BTN4, INPUT_PULLUP);
 // pinMode(SHIFT_BTN3, INPUT_PULLUP);
 // pinMode(SHIFT_BTN2, INPUT_PULLUP);
 // pinMode(SHIFT_BTN1, INPUT_PULLUP);
  pinMode(SHIFT_BTN, INPUT_PULLUP);

  //displayStatus();
}

/*******************************************************************************
   Here we service, monitor, and otherwise control all our peripheral devices.
   First, we retrieve the status of our encoders and buttons and update Eos.
   Next, we check if there are any OSC messages for us.
   Finally, we update our display (if an update is necessary)

   NOTE: This function is our main loop and thus this function will be called
   repeatedly forever

   Parameters: none

   Return Value: void

 ******************************************************************************/
void loop()
{
  static String curMsg;
  int size;

  // Get the updated state of each encoder
  int32_t panMotion = updateEncoder(&panWheel);
  int32_t tiltMotion = updateEncoder(&tiltWheel);
  int32_t zoomMotion = updateEncoder(&zoomWheel);
  int32_t edgeMotion = updateEncoder(&edgeWheel);
    int32_t whiteMotion = updateEncoder(&whiteWheel);
  int32_t intensityMotion = updateEncoder(&intensityWheel);

  // Scale the result by a scaling factor
  panMotion *= PAN_TILT_SCALE;
  tiltMotion *= ZOOM_EDGE_SCALE;
  zoomMotion *= RED_GREEN_SCALE;
  edgeMotion *= BLUE_AMBER_SCALE;
  whiteMotion *= WHITE_WHEEL_SCALE;
  // Check for next/last updates
  //checkButtons();

  // Update the wheels
  if (tiltMotion != 0)
    sendWheelMove(EDGE_ZOOM, tiltMotion);

  if (zoomMotion != 0)
    sendWheelMove(RED_GREEN, zoomMotion);

  if (edgeMotion != 0)
    sendWheelMove(BLUE_AMBER, edgeMotion);

  if (whiteMotion != 0)
    sendWheelMove(WHITE_WHEEL, whiteMotion);

  if (panMotion != 0)
    sendWheelMove(PAN_TILT, panMotion);

      if (intensityMotion != 0)
    handleIntensityWheel(intensityMotion);


  // Handle OSC messages from Eos
  size = SLIPSerial.available();
  if (size > 0)
  {
    // Fill the msg with all of the available bytes
    while (size--)
      curMsg += (char)(SLIPSerial.read());
  }
  if (SLIPSerial.endofPacket())
  {
    parseOSCMessage(curMsg);
    lastMessageRxTime = millis();
    // We only care about the ping if we haven't heard recently
    // Clear flag when we get any traffic
    timeoutPingSent = false;
    curMsg = String();
  }

  // Check for timeout and send ping if necessary
  if (lastMessageRxTime > 0)
  {
    unsigned long diff = millis() - lastMessageRxTime;
    // Check if it's been too long and we need to timeout
    if (diff > TIMEOUT_AFTER_IDLE_INTERVAL)
    {
      connectedToConsole = ConsoleNone;
      lastMessageRxTime = 0;
      updateDisplay = true;
      timeoutPingSent = false;
    }

    // It could be the console is sitting idle. Send a ping once to
    // double-check that it's still there, but only once after 2.5s have passed
    if (!timeoutPingSent && diff > PING_AFTER_IDLE_INTERVAL)
    {
      OSCMessage ping("/eos/ping");
      ping.add(BOX_NAME_STRING "_hello"); // This way we know who is sending the ping
      SLIPSerial.beginPacket();
      ping.send(SLIPSerial);
      SLIPSerial.endPacket();
      timeoutPingSent = true;
    }
  }

  // Update the display if necessary
  //if (updateDisplay)
   // displayStatus();
}