/*
  Original source is here: 
  https://projecthub.arduino.cc/utilstudio/mearm-161-robot-joystick-board-recording-movements-ir-235439
  meArm analog joysticks version 1.6.1.4 - UtilStudio.com Jan 2019
  
  Extended by AMA to support 6 servos (added WristRoll and WristPitch)
  Uses two analogue joysticks and six servos.
  Hold the left joystick button to control WristRoll and WristPitch with the right joystick

  Changes in this version:
  - infraredSensor enabled
  - Short press on left joystick button (pinRecord) toggles recording.
  - Long press on left joystick button does NOT toggle recording (but still acts as "hold" to control wrists).
  - Debounce implemented for both record and play buttons (millis-based).
  - Play long-press (2s) starts autoplay (repeatePlaying). Short press toggles play.
  - CheckIrController extended with wrist codes (placeholders).
  - Fix: only print IR code when a new code is decoded (no repeated "0" output).
  - updated IRremote library
*/

#include <Servo.h>

#define infraredSensor       // enable infrared controller support

#if defined infraredSensor
#define DECODE_NEC //very small NEC only decoder. Reduces memory footprint and decreases decoding time.
#include <IRremote.hpp>
#endif

//bool debug = false;
bool debug = true;

bool repeatePlaying  = false;   /* Repeatedly is running recorded cycle */
int delayBetweenCycles  = 2000; /* Delay between cycles */

const int basePin = 11;       /* Base  servo */
const int shoulderPin = 10;   /* Shoulder servo */
const int elbowPin  = 9;       /* Elbow servo */
const int gripperPin = 5;     /* Gripper servo -  changed */
const int wristRollPin = 7;   /* Wrist roll servo - added */
const int wristPitchPin = 6;  /* Wrist pitch servo - added */

int xdirPin = A0;        /* Base - joystick1*/
int ydirPin =  A1;       /* Shoulder - joystick1 */
int zdirPin = A2;        /* Elbow or WristRoll - joystick2  */
int gdirPin = A3;        /* Gripper or WristPitch - joystick2 */

int pinRecord = 2;     /* Button record (left joystick button) */
int pinPlay = 4;       /* Button play - changed */
int  pinLedRecord = 3;  /* LED - indicates recording (light) or auto play mode (blink  ones) - changed */

int IR_RECEIVE_PIN = 12;      /* Infrared sensor */

const  int buffSize = 412; /* Size of recording buffer */

int startBase = 90;
int startShoulder = 90; //42;
int startElbow = 90; //50;
int startGripper = 90; //30;
int startWristRoll = 90;
int startWristPitch = 90;

int posBase = 90;
int posShoulder = 90;
int posElbow = 90;
int posGripper =  90;
int posWristRoll = 90;
int posWristPitch = 90;

int lastBase = 90;
int lastShoulder = 90;
int lastElbow = 90;
int lastGripper = 90;
int lastWristRoll = 90;
int lastWristPitch = 90;

int minBase = 0;
int maxBase = 180; //150;
int minShoulder  = 0; //25;
int maxShoulder = 180; //135;
int minElbow = 0; //10;
int maxElbow = 180; //118;
int minGripper = 0; //10;
int maxGripper = 180; //61;
int minWristRoll = 0;
int maxWristRoll = 180;
int minWristPitch = 0; //10;
int maxWristPitch = 180; //170;

const int countServo = 6;
byte  currentPos[countServo] = {
  0, 0, 0, 0, 0, 0};
byte lastPos[countServo] = {
  255, 255, 255, 255, 255, 255};
byte minPos[countServo] = {
  minBase, minShoulder,  minElbow, minGripper, minWristRoll, minWristPitch};
byte maxPos[countServo] = {
  maxBase, maxShoulder,  maxElbow, maxGripper, maxWristRoll, maxWristPitch};
int servoPin[countServo] = {
  basePin, shoulderPin,  elbowPin, gripperPin, wristRollPin, wristPitchPin};


uint16_t buff[buffSize];
uint16_t buffAdd[countServo];
int  recPos = 0;
int playPos = 0;
int blockPlayPos = 0;

int buffPos = 0;

struct  tPlayBuff
{
  byte ServoNumber;
  byte Angle;
};

tPlayBuff playBuff[countServo];
int  playBuffCount = 0;

byte cyclesRecord = 0;
byte cyclesPlay = 0;

/* Raw reads and stable states for debounced buttons */
int rawRecordReading = HIGH;
int rawPlayReading = HIGH;
int recordStableState = HIGH;
int playStableState = HIGH;

unsigned long lastRecordDebounceTime = 0;
unsigned long lastPlayDebounceTime = 0;

/* Press timing */
unsigned long recordPressMillis = 0;
unsigned long playPressMillis = 0;

/* Flags */
bool record = false;
bool play = false;
bool playAutoActivated = false; // set when long-press autoplay activated

String  command = "Manual";
int printPos = 0;

/* Debounce and press thresholds (ms) */
const unsigned long DEBOUNCE_MS = 50;
const unsigned long SHORT_PRESS_MS = 500; // short press threshold (ms) for record/play
const unsigned long PLAY_LONG_PRESS_MS = 2000; // long-press threshold for autoplay

int buttonPlayDelay = 20; // not used now but kept for compatibility

bool ledLight = false;

int servoTime = 0;

/* joystick / IR deltas */
float  dx = 0;
float dy = 0;
float dz = 0;
float dg = 0;
/* additional deltas from IR specifically for wrists */
float dwr = 0;
float dwp = 0;

Servo servoBase;
Servo  servoShoulder;
Servo servoElbow;
Servo servoGripper;
Servo servoWristRoll;
Servo servoWristPitch;

#if defined infraredSensor
//IRrecv  irrecv(RECV_PIN);
//decode_results results;
IRRawDataType lastRawDataToRepeat = 0; //keep the last ir command to repeat it when the ir key is being hold.

// Please, replace codes  of infrared controller with your own codes
// in function CheckIrController. Four new codes below are placeholders
// for WristRoll +/- and WristPitch +/- — replace with your remote's codes.
void  CheckIrController()
{
  if (IrReceiver.decode())
  {
    if (IrReceiver.decodedIRData.decodedRawData == 0) //last ir key is being hold
      {} // repeat the last command, do not change lastNonzeroRawData
    else
      {lastRawDataToRepeat = IrReceiver.decodedIRData.decodedRawData;}

    switch (lastRawDataToRepeat) //(IrReceiver.decodedIRData.decodedRawData)
    {
    case 0xE916FF00: //car_mp3 0
      dx  = 4;
      break;
    case 0xF30CFF00: //car_mp3 1
      dx =  -4;
      break;
    case 0xE619FF00: //car_mp3 100+
      dy  = 4;
      break;
    case 0xE718FF00: //car_mp3 2
      dy  = -4;
      break;
    case 0xF20DFF00: //car_mp3 200+
      dz = -4;
      break;
    case 0xA15EFF00: //car_mp3 3
      dz = 4;
      break;
    case 0xF708FF00: //car_mp3 4
      dg = -4;
      break;
    case  0xBD42FF00: //car_mp3 7
      dg = 4;
      break;

    // Legacy IR "button" toggles — map these to toggle recording/play states directly
    case 0xBB44FF00: //car_mp3 PREV
      // Toggle recording state via IR (short-press behavior)
      if (record) {
        record = false;
        if (debug) PrintBuffer();
      } else {
        record = true;
        play = false;
        repeatePlaying = false;
        recPos = 0;
        cyclesRecord = 0;
      }
      lastRawDataToRepeat = 0; //do not repeat this command
      break;

    case 0xBC43FF00: //car_mp3 PLAY
      // Toggle play state via IR
      if (play) {
        play = false;
        repeatePlaying = false;
      } else {
        if (record) record = false;
        play = true;
        repeatePlaying = false;
      }
      lastRawDataToRepeat = 0; //do not repeat this command
      break;

    case 0xBF40FF00: //car_mp3 NEXT
      repeatePlaying  = !repeatePlaying;

      if (repeatePlaying)
      {
        record  = false;
        play = true;
      }
      else
      {
        play  = false;
      }
      lastRawDataToRepeat = 0; //do not repeat this command
      break;

    // New wrist control IR codes ----------------------------
    // WristRoll increase
    case 0xE31CFF00: //car_mp3 5
      dwr = 4;   // roll increase
      break;
    // WristRoll decrease
    case 0xAD52FF00: //car_mp3 8
      dwr = -4;  // roll decrease
      break;
    // WristPitch increase
    case 0xA55AFF00: //car_mp3 6
      dwp = 4;   // pitch increase
      break;
    // WristPitch decrease
    case 0xB54AFF00: //car_mp3 9
      dwp = -4;  // pitch decrease
      break;
    // --------------------------------------------------------------------
    }

    // Print only when a new IR code is decoded
    if (debug) {
      //Serial.println(results.value, HEX);
      Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX); // Print "old" raw data
      //IrReceiver.printIRResultShort(&Serial); // Print complete received data in one line
      //IrReceiver.printIRSendUsage(&Serial);   // Print the statement required to send this data
    }

    // Resume receiver to accept next code
    //irrecv.resume();
    IrReceiver.resume(); // Enable receiving of the next value
  }
}
#endif

void setup()  {
  Serial.begin(9600);

  pinMode(xdirPin, INPUT);
  pinMode(ydirPin,  INPUT);
  pinMode(zdirPin, INPUT);
  pinMode(gdirPin, INPUT);

  pinMode(pinRecord,  INPUT_PULLUP);
  pinMode(pinPlay, INPUT_PULLUP);

  pinMode(pinLedRecord,  OUTPUT);

  servoBase.attach(basePin);
  servoShoulder.attach(shoulderPin);
  servoElbow.attach(elbowPin);
  servoGripper.attach(gripperPin);
  servoWristRoll.attach(wristRollPin);
  servoWristPitch.attach(wristPitchPin);

  // initialize raw and stable button states from hardware to avoid spurious change detection
  rawRecordReading = digitalRead(pinRecord);
  rawPlayReading = digitalRead(pinPlay);
  recordStableState = rawRecordReading;
  playStableState = rawPlayReading;
  lastRecordDebounceTime = millis();
  lastPlayDebounceTime = millis();

  StartPosition();

#if  defined infraredSensor
  //irrecv.enableIRIn(); // Start the receiver
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK); // Start the receiver //DISABLE_LED_FEEDBACK with onboard LED pin 13
#endif

  digitalWrite(pinLedRecord, HIGH);
  delay(1000);
  digitalWrite(pinLedRecord,  LOW);
}

void loop() {
  unsigned long now = millis();

  // --- Read buttons raw ---
  int readingRecord = digitalRead(pinRecord);
  int readingPlay = digitalRead(pinPlay);

  // --- Debounce record button ---
  if (readingRecord != rawRecordReading) {
    lastRecordDebounceTime = now;
    rawRecordReading = readingRecord;
  }
  if ((now - lastRecordDebounceTime) > DEBOUNCE_MS) {
    if (recordStableState != rawRecordReading) {
      // stable state changed
      recordStableState = rawRecordReading;

      if (recordStableState == LOW) {
        // pressed
        recordPressMillis = now;
      } else {
        // released: evaluate duration
        unsigned long duration = now - recordPressMillis;
        if (duration < SHORT_PRESS_MS) {
          // short press -> toggle recording
          record = !record;

          if (record)
          {
            play = false;
            repeatePlaying  = false;
            recPos = 0;
            cyclesRecord = 0;
          }
          else
          {
            if (debug) PrintBuffer();
          }
        } else {
          // long press: do not toggle recording (per request)
        }
      }
    }
  }

  // --- Debounce play button ---
  if (readingPlay != rawPlayReading) {
    lastPlayDebounceTime = now;
    rawPlayReading = readingPlay;
  }
  if ((now - lastPlayDebounceTime) > DEBOUNCE_MS) {
    if (playStableState != rawPlayReading) {
      // stable state changed
      playStableState = rawPlayReading;

      if (playStableState == LOW) {
        // pressed: start timing
        playPressMillis = now;
        playAutoActivated = false;
      } else {
        // released: decide
        unsigned long duration = now - playPressMillis;
        if (playAutoActivated) {
          // long-press already activated autoplay while held; on release just clear flag
          playAutoActivated = false;
        } else if (duration < SHORT_PRESS_MS) {
          // short press -> toggle play
          if (record) {
            record = false;
          }
          play = !play;
          repeatePlaying = false;
        } else if (duration >= PLAY_LONG_PRESS_MS) {
          // on release after long press, start autoplay if not already started
          repeatePlaying = true;
          record = false;
          play = true;
          playAutoActivated = false;
        } else {
          // between short and long press: treat as short press (toggle)
          if (duration < PLAY_LONG_PRESS_MS) {
            if (record) {
              record = false;
            }
            play = !play;
            repeatePlaying = false;
          }
        }
      }
    }
  }

  // If play button is being held and exceeds long-press threshold, activate autoplay immediately
  if (playStableState == LOW && !playAutoActivated && playPressMillis != 0) {
    if (now - playPressMillis >= PLAY_LONG_PRESS_MS) {
      // activate autoplay
      repeatePlaying = true;
      record = false;
      play = true;
      playAutoActivated = true;
    }
  }

  // --- IR and joystick processing ---

  dx = 0;
  dy = 0;
  dz = 0;
  dg = 0;
  dwr = 0;
  dwp = 0;

#if defined infraredSensor
  // Handle IR; CheckIrController prints only when a new code is decoded.
  CheckIrController();
#endif

  if (repeatePlaying)
  {
    play = true;
  }

  // Read joystick analog values if no IR delta present for that axis
  if (dx == 0) dx = map(analogRead(xdirPin), 0, 1023, 4.0, -4.0);
  if (dy == 0) dy = map(analogRead(ydirPin), 0, 1023, 4.0, -4.0);
  if (dz ==  0) dz = map(analogRead(zdirPin), 0, 1023, 4.0, -4.0);
  if (dg == 0) dg = map(analogRead(gdirPin),  0, 1023, -4.0, 4.0);

  if (abs(dx) < 1.5) dx = 0;
  if (abs(dy) < 1.5)  dy = 0;
  if (abs(dz) < 1.5) dz = 0;
  if (abs(dg) < 1.5) dg = 0;

  posBase += dx;
  posShoulder += dy;

  // If left joystick button is kept pressed, use right joystick to control wrists
  if (recordStableState == LOW)
  {
    // right joystick controls wrist servos
    posWristRoll += dz;
    posWristPitch += dg;
  }
  else
  {
    // normal behaviour: right joystick controls elbow and gripper
    posElbow += dz;
    posGripper +=  dg;
  }

  // Apply IR wrist deltas as well (they act independently of button state)
  if (dwr != 0) posWristRoll += dwr;
  if (dwp != 0) posWristPitch += dwp;

  if ((play) | (cyclesPlay > 0))
  {
    if (cyclesPlay > 0)
    {
      cyclesPlay--;

      if (cyclesPlay > 0)
      {
        if  (play)
        {
          playPos = blockPlayPos;
        }
        else
        {
          play = false;
        }
      }
    }

    if  (play)
    {
      if (playPos >= recPos)
      {
        playPos =  0;

        if (repeatePlaying)
        {
          if (debug)
          {
            Serial.println("Auto start ");
          }

          delay(delayBetweenCycles);
          // StartPosition();
        }
        else
        {
          play  = false;
          repeatePlaying = false;
        }
      }

      blockPlayPos  = playPos;
      playBuffCount = 0;

      bool endOfData = false;

      while (!endOfData)
      {
        if (playPos >= buffSize - 1) break;
        if (playPos >= recPos) break;

        uint16_t data = buff[playPos];
        byte angle = data & 0xFF;
        uint16_t servoNumber = data & 0xf00;
        endOfData = data & 0x8000;
        uint16_t repeatCycles = data & 0x7000;
        repeatCycles = repeatCycles >> 12;

        if (play & cyclesPlay  <= 0)
        {
          cyclesPlay = repeatCycles;
        }

        servoNumber = servoNumber >> 8;

        playBuff[playBuffCount].ServoNumber  = servoNumber;
        playBuff[playBuffCount].Angle = angle;

        playBuffCount++;

        switch (servoNumber)
        {
        case 0:
          posBase  = angle;
          break;

        case 1:
          posShoulder = angle;
          break;

        case 2:
          posElbow = angle;
          break;

        case 3:
          posGripper = angle;
          break;

        case 4:
          posWristRoll = angle;
          break;

        case 5:
          posWristPitch = angle;
          break;
        }

        playPos++;

        if (playPos >= recPos)
        {
          play = false;

          break;
        }
      }
    }
  }
  else
  {
    cyclesPlay = 0;
  }

  if (posBase > maxBase)  posBase = maxBase;
  if (posShoulder > maxShoulder) posShoulder = maxShoulder;
  if (posElbow > maxElbow) posElbow = maxElbow;
  if (posGripper > maxGripper)  posGripper = maxGripper;
  if (posWristRoll > maxWristRoll) posWristRoll = maxWristRoll;
  if (posWristPitch > maxWristPitch) posWristPitch = maxWristPitch;

  if (posBase < minBase) posBase = minBase;
  if (posShoulder < minShoulder) posShoulder = minShoulder;
  if (posElbow <  minElbow) posElbow = minElbow;
  if (posGripper < minGripper) posGripper = minGripper;
  if (posWristRoll < minWristRoll) posWristRoll = minWristRoll;
  if (posWristPitch < minWristPitch) posWristPitch = minWristPitch;

  currentPos[0] = posBase;
  currentPos[1] = posShoulder;
  currentPos[2]  = posElbow;
  currentPos[3] = posGripper;
  currentPos[4] = posWristRoll;
  currentPos[5] = posWristPitch;

  bool positionChanged = false;

  if ((play) | (cyclesPlay > 0))
  {
    for (int i = 0; i < playBuffCount;  i++)
    {
      MoveServo(servoPin[playBuff[i].ServoNumber], playBuff[i].Angle,  servoTime);
    }
  }
  else {
    for (int i = 0; i < countServo; i++)
    {
      if (currentPos[i] > maxPos[i]) currentPos[i] = maxPos[i];
      if  (currentPos[i] < minPos[i]) currentPos[i] = minPos[i];

      if (currentPos[i]  != lastPos[i])
      {
        positionChanged = true;
        MoveServo(servoPin[i],  currentPos[i], servoTime);
      }
    }
  }

  if (positionChanged)
  {
    if (record)
    {
      buffPos = 0;
      cyclesRecord = 0;

      for (int i = 0; i < countServo; i++)
      {
        if (lastPos[i]  != currentPos[i])
        {
          uint16_t value = (currentPos[i] | (0x100  * i)) & 0x0FFF;
          buffAdd[buffPos] = value;
          buffPos++;
        }
      }
      buffAdd[buffPos - 1] = buffAdd[buffPos - 1] | 0x8000;

      AddToBuff();
    }
  }
  else
  {
    if (record)
    {
      if (recPos > 0)
      {
        cyclesRecord ++;

        bool  added = false;
        bool first = true;

        for (int i = recPos  - 1; i >= 0; i--)
        {
          bool endOfData = buff[i] & 0x8000;

          if (!first && endOfData) break;

          if (cyclesRecord > 7)
          {
            cyclesRecord = 0;

            AddToBuff();
            added = true;
          }

          if (!added) {
            uint16_t  val = cyclesRecord & 7;
            val = val << 12;
            val = val  & 0x7000;

            buff[i] = buff[i] & 0x8FFF;
            buff[i]  = buff[i] | val;

            first  = false;
          }
        }
      }
    }
  }

  lastBase  = posBase;
  lastShoulder = posShoulder;
  lastElbow = posElbow;
  lastGripper  = posGripper;
  lastWristRoll = posWristRoll;
  lastWristPitch = posWristPitch;

  for (int i = 0; i < countServo; i++)
  {
    lastPos[i]  = currentPos[i];
  }

  if ( repeatePlaying)
  {
    ledLight = !ledLight;
  }
  else
  {
    if (ledLight)
    {
      ledLight = false;
    }

    if (record)
    {
      ledLight = true;
    }
  };

  digitalWrite(pinLedRecord, ledLight);
  delay(50);
}

void AddToBuff()
{
  for (int i = 0; i < buffPos; i++)
  {
    // Safety: guard against overflow
    if ((recPos + i) < buffSize) buff[recPos + i] = buffAdd[i];
  }

  recPos += buffPos;

  if (recPos >= buffSize - countServo)
  {
    record = false;
  }
}

void MoveServo(uint8_t idServo, int  angle, int timeMs)
{
  switch (idServo)
  {
  case basePin:
    servoBase.write(angle);
    break;
  case shoulderPin:
    servoShoulder.write(angle);
    break;
  case elbowPin:
    servoElbow.write(angle);
    break;
  case gripperPin:
    servoGripper.write(angle);
    break;
  case wristRollPin:
    servoWristRoll.write(angle);
    break;
  case wristPitchPin:
    servoWristPitch.write(angle);
    break;
  }

  if (debug)
  {
    if (record) Serial.print(" Record ");
    if (play) Serial.print(" Play  ");
    if (repeatePlaying) Serial.print(" Auto ");

    Serial.print("  Servo ");
    Serial.print(idServo);
    Serial.print(" Angle ");
    Serial.println(angle);
  }
}

void PrintBuffer()
{
  for (int i = 0; i < recPos; i++)
  {
    uint16_t data = buff[i];
    byte angle = data & 0xFF;
    uint16_t  servoNumber = data & 0xF00;
    servoNumber = servoNumber >> 8;
    bool endOfData  = data & 0x8000;

    Serial.print("Servo=");
    Serial.print(servoNumber,  HEX);
    Serial.print("\tAngle=");
    Serial.print(angle);
    Serial.print("\tEnd=");
    Serial.print(endOfData);
    Serial.print("\tData=");
    Serial.print(data,  BIN);
    Serial.println();
  }
}

void StartPosition()
{
  int angleBase = servoBase.read();
  int angleShoulder = servoShoulder.read();
  int angleElbow = servoElbow.read();
  int angleGripper = servoGripper.read();
  int angleWristRoll = servoWristRoll.read();
  int angleWristPitch = servoWristPitch.read();

  Serial.print(angleBase);
  Serial.print("\t");
  Serial.print(angleShoulder);
  Serial.print("\t");
  Serial.print(angleElbow);
  Serial.print("\t");
  Serial.print(angleGripper);
  Serial.print("\t");
  Serial.print(angleWristRoll);
  Serial.print("\t");
  Serial.print(angleWristPitch);
  Serial.println("\t");

  posBase = startBase;
  posShoulder = startShoulder;
  posElbow = startElbow;
  posGripper = startGripper;
  posWristRoll = startWristRoll;
  posWristPitch = startWristPitch;

  servoBase.write(posBase);
  servoShoulder.write(posShoulder);
  servoElbow.write(posElbow);
  servoGripper.write(posGripper);
  servoWristRoll.write(posWristRoll);
  servoWristPitch.write(posWristPitch);
}