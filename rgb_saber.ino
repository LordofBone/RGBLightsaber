// import stuffs
#include "SoftwareSerial.h"
#include "Adafruit_Pixie.h"
#include "Arduino.h"
#include "DFRobotDFPlayerMini.h"

// pixie setup
#define NUMPIXELS 1 // Number of Pixies in the strip
#define PIXIEPIN  8 // Pin number for SoftwareSerial output

// audio + rgb pins
#define audioPIN 9
#define redPIN A3
#define greenPIN A0
#define bluePIN A4

// define outputs
#define outputA 6
#define outputB 4
#define outputC 5

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 12;     // the number of the pushbutton pin
const int buttonPin1 = 2;     // the number of the pushbutton pin

// decent brightness but not too much to overheat the pixie or drain power too much
#define pixieBrightness 120

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status
int buttonState1 = 0;

// set states for use with encoder
int aState;
int aLastState;

// set blade to OFF
bool bladeOn = false;

// int for mode
int mode = 0;

// colour values
int rColour = 0;
int gColour = 0;
int bColour = 0;

// timer setup for buzzing sound
unsigned long interval = 3000;   // the time we need to wait
unsigned long previousMillis = 0; // millis() returns an unsigned long.

// serial setup for pixie
SoftwareSerial pixieSerial(-1, PIXIEPIN);

Adafruit_Pixie strip = Adafruit_Pixie(NUMPIXELS, &pixieSerial);
// Alternately, can use a hardware serial port for output, e.g.:
// Adafruit_Pixie strip = Adafruit_Pixie(NUMPIXELS, &Serial1);

// serial setup for mp3 player
SoftwareSerial mySoftwareSerial(10, 11); // RX, TX
DFRobotDFPlayerMini myDFPlayer;
void printDetail(uint8_t type, int value);

void setup() {

  // begin serials
  Serial.begin(19200);
  mySoftwareSerial.begin(9600);
  pixieSerial.begin(115200); // Pixie REQUIRES this baud rate

  // start dbrobot player
  Serial.println();
  Serial.println(F("DFRobot DFPlayer Mini Demo"));
  Serial.println(F("Initializing DFPlayer ... (May take 3~5 seconds)"));

  if (!myDFPlayer.begin(mySoftwareSerial)) {  //Use softwareSerial to communicate with mp3.
    Serial.println(F("Unable to begin:"));
    Serial.println(F("1.Please recheck the connection!"));
    Serial.println(F("2.Please insert the SD card!"));
    while (true);
  }
  Serial.println(F("DFPlayer Mini online."));

  // volume set to max
  myDFPlayer.volume(30);  //Set volume value. From 0 to 30

  // setup pins for rotary encoder
  pinMode (outputA, INPUT_PULLUP);
  pinMode (outputB, INPUT_PULLUP);

  pinMode(outputC, OUTPUT);
  digitalWrite(outputC, LOW);

  // setup pins for buttons
  pinMode(buttonPin, INPUT);
  pinMode(buttonPin1, INPUT);

  // set pixie brightness from previous variable
  strip.setBrightness(pixieBrightness);  // Adjust as necessary to avoid blinding

  // digital read for rotary state
  aLastState = digitalRead(outputA);

}


void loop() {

  // dfrobot player debug stuff
  //  if (myDFPlayer.available()) {
  //    printDetail(myDFPlayer.readType(), myDFPlayer.read()); //Print the detail message from DFPlayer to handle different errors and states.
  //  }

  // read button states
  buttonState = digitalRead(buttonPin);
  buttonState1 = digitalRead(buttonPin1);

  //setup leds to show what mode is being adjusted
  if (mode == 0) {
    analogWrite(redPIN, 255);
    analogWrite(greenPIN, 255);
    analogWrite(bluePIN, 255);
  }
  if (mode == 1) {
    analogWrite(redPIN, 100);
    analogWrite(greenPIN, 255);
    analogWrite(bluePIN, 255);

  }
  if (mode == 2) {
    analogWrite(redPIN, 255);
    analogWrite(greenPIN, 100);
    analogWrite(bluePIN, 255);
  }
  if (mode == 3) {
    analogWrite(redPIN, 255);
    analogWrite(greenPIN, 255);
    analogWrite(bluePIN, 100);
  }

  //when button is pushed change mode
  if (isButtonPushDown()) {
    if (mode < 3) {
      mode++;
    }
    else {
      mode = 0;
    }
    delay(500);
  }

  //when activate button is pressed it will activate the leds and play the saber on sound
  //if the blade is already active it will shut off the leds and play the saber off sound
  if (isButtonPushDown2()) {
    Serial.println("button0");
    //if blade currently off set it to on and play sound and fire up the neopixels with current colours
    if (bladeOn == false) {
      Serial.println(bladeOn);
      myDFPlayer.playFolder(1, 1);
      bladeOn = !bladeOn;
      delay(500);
    }
    //if blade currently on set it to off, play sound and clear neopixels
    else if (bladeOn == true) {
      Serial.println(bladeOn);
      myDFPlayer.playFolder(1, 2);
      bladeOn = !bladeOn;
      delay(500);
    }
  }

  aState = digitalRead(outputA); // Reads the "current" state of the outputA

  //      Serial.print("Rotary A: ");
  //    Serial.println(digitalRead(outputA));
  //    Serial.print("Rotary B: ");
  //    Serial.println(digitalRead(outputB));

  // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState) {
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(outputB) != aState) {

      //depending on mode change r/g/b colour - if the number goes over 255 reset to 0 and vice-versa
      if (mode == 1) {
        if (rColour < 255) {
          ++rColour;
        }
        else {
          rColour = 0;
        }
      }
      if (mode == 2) {
        if (gColour < 255) {
          ++gColour;
        }
        else {
          gColour = 0;
        }
      }
      if (mode == 3) {
        if (bColour < 255) {
          ++bColour;
        }
        else {
          bColour = 0;
        }
      }

    } else {

      if (mode == 1) {
        if (rColour > 0) {
          --rColour;
        }
        else {
          rColour = 255;
        }
      }
      if (mode == 2) {
        if (gColour > 0) {
          --gColour;
        }
        else {
          gColour = 255;
        }
      }
      if (mode == 3) {
        if (bColour > 0) {
          --bColour;
        }
        else {
          bColour = 255;
        }
      }

    }

    //print out led values
    Serial.print("Red: ");
    Serial.println(rColour);
    Serial.print("Green: ");
    Serial.println(gColour);
    Serial.print("Blue: ");
    Serial.println(bColour);

    delay(10);

  }
  aLastState = aState; // Updates the previous state of the outputA with the current state

  digitalWrite(outputA, HIGH);
  digitalWrite(outputB, HIGH);

  unsigned long currentMillis = millis(); // grab current time

  //display the neopixels + play blade sound every 3 seconds
  if (bladeOn == true) {
    strip.setPixelColor(0, rColour, gColour, bColour);
    if ((unsigned long)(currentMillis - previousMillis) >= interval) {
      myDFPlayer.playFolder(1, 5);
      previousMillis = millis();
    }
  }
  else {
    //colour values to zero to reset the blade
    strip.setPixelColor(0, 0, 0, 0);
  }

  strip.show();

}

//function for button push detection
boolean isButtonPushDown(void) {
  if (digitalRead(buttonPin1)) {
    delay(5);
    if (digitalRead(buttonPin1))
      return true;
  }
  return false;
}


//function for button push detection
boolean isButtonPushDown2(void) {
  if (digitalRead(buttonPin)) {
    delay(5);
    if (digitalRead(buttonPin))
      return true;
  }
  return false;
}

// dfplayer info stuffs
void printDetail(uint8_t type, int value) {
  switch (type) {
    case TimeOut:
      Serial.println(F("Time Out!"));
      break;
    case WrongStack:
      Serial.println(F("Stack Wrong!"));
      break;
    case DFPlayerCardInserted:
      Serial.println(F("Card Inserted!"));
      break;
    case DFPlayerCardRemoved:
      Serial.println(F("Card Removed!"));
      break;
    case DFPlayerCardOnline:
      Serial.println(F("Card Online!"));
      break;
    case DFPlayerPlayFinished:
      Serial.print(F("Number:"));
      Serial.print(value);
      Serial.println(F(" Play Finished!"));
      break;
    case DFPlayerError:
      Serial.print(F("DFPlayerError:"));
      switch (value) {
        case Busy:
          Serial.println(F("Card not found"));
          break;
        case Sleeping:
          Serial.println(F("Sleeping"));
          break;
        case SerialWrongStack:
          Serial.println(F("Get Wrong Stack"));
          break;
        case CheckSumNotMatch:
          Serial.println(F("Check Sum Not Match"));
          break;
        case FileIndexOut:
          Serial.println(F("File Index Out of Bound"));
          break;
        case FileMismatch:
          Serial.println(F("Cannot Find File"));
          break;
        case Advertise:
          Serial.println(F("In Advertise"));
          break;
        default:
          break;
      }
      break;
    default:
      break;
  }
}
