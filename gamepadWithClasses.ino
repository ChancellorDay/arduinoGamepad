#include <Keypad.h>
#include <Joystick.h>
#include <Keyboard.h>
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_GAMEPAD,
  32, //number of buttons
  0, //number of hat switches
  //Set as many axis to "true" as you have potentiometers for
  true, // x axis
  false, // y axis
  true, // z axis
  false, // rx axis
  false, // ry axis
  false, // rz axis
  false, // rudder
  false, // throttle
  false, // accelerator
  false, // brake
  false); // steering wheel

int xAxis_ = 0;
int yAxis_ = 0;
int zAxis_ = 0;
int rxAxis_ = 0;
int ryAxis_ = 0;
int rzAxis_ = 0;
const bool initAutoSendState = true;
/* 
IMPORTANT! - HOW TO USE

DEFINE YOUR HARDWARE ABOVE THE LOOP AT THE BOTTOM,
USE THE PROVIDED CONSTRUCTORS AND THEIR DOCUMENTATION PROVIDED

IF YOU HAVE QUESTIONS, CONTACT ME AT -- DCHANCE@UMICH.EDU
*/
//These do not need to be defined, do not touch
int NUMENCODERS;
int NUMPOTENTIOMETERS;
int gamePadButtonNum;

#define DIR_CCW 0x10
#define DIR_CW 0x20
#define R_START 0x0

#ifdef HALF_STEP
#define R_CCW_BEGIN 0x1
#define R_CW_BEGIN 0x2
#define R_START_M 0x3
#define R_CW_BEGIN_M 0x4
#define R_CCW_BEGIN_M 0x5
const unsigned char ttable[6][4] = {
  // R_START (00)
  {R_START_M,            R_CW_BEGIN,     R_CCW_BEGIN,  R_START},
  // R_CCW_BEGIN
  {R_START_M | DIR_CCW, R_START,        R_CCW_BEGIN,  R_START},
  // R_CW_BEGIN
  {R_START_M | DIR_CW,  R_CW_BEGIN,     R_START,      R_START},
  // R_START_M (11)
  {R_START_M,            R_CCW_BEGIN_M,  R_CW_BEGIN_M, R_START},
  // R_CW_BEGIN_M
  {R_START_M,            R_START_M,      R_CW_BEGIN_M, R_START | DIR_CW},
  // R_CCW_BEGIN_M
  {R_START_M,            R_CCW_BEGIN_M,  R_START_M,    R_START | DIR_CCW},
};
#else
#define R_CW_FINAL 0x1
#define R_CW_BEGIN 0x2
#define R_CW_NEXT 0x3
#define R_CCW_BEGIN 0x4
#define R_CCW_FINAL 0x5
#define R_CCW_NEXT 0x6

const unsigned char ttable[7][4] = {
  // R_START
  {R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START},
  // R_CW_FINAL
  {R_CW_NEXT,  R_START,     R_CW_FINAL,  R_START | DIR_CW},
  // R_CW_BEGIN
  {R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START},
  // R_CW_NEXT
  {R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START},
  // R_CCW_BEGIN
  {R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START},
  // R_CCW_FINAL
  {R_CCW_NEXT, R_CCW_FINAL, R_START,     R_START | DIR_CCW},
  // R_CCW_NEXT
  {R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START},
};
#endif


//Hardware is the base class which has multiple derived classes (RotaryEncoder, Button, Potentiometer)
class Hardware {
  public:
    //HardwareCode is a variable that contains the 2 character string code which represents the
    //type of hardware it is
    String hardwareCode;
    //RotaryEncoders: RE
    //Rotary Buttons: RB
    //Rotational Potentiomoters: RP
    //Sliding Potentiomoters: SP
    //Button: BT

    //num is value that represnt what number of each hardware there is, for example 
    //if you have 2 encoders and 3 buttons, you would have RE0, RE1, BT0, BT1, BT2
    int num;


    //serialSignal calls Serial.println() with a coded message to be recieved by our browser application
    //The browser application can be found here: https://github.com/twhelgeson/robot-gui
    //you do not need the browser application to take advantage of the signals sent
    //the cypher is located here: https://docs.google.com/spreadsheets/d/1s-_qcMhMEQdyH7Sp79ui6K6VUlbxQhxz6EQCZoBVXGc/edit?usp=sharing
    //call serialSignal with override code when using multifunction hardware that has different coding scheme
    //example: serialSignal("D", "RB") would print RB2D meaning rotary button 2 (number of encoder) was double clicked
    void serialSignal(int state) {
      String stateString = String(state);
      String numString = String(num);
      Serial.println(hardwareCode + numString + stateString);
    }
    void serialSignal(double state) {
      String stateString = String(state);
      String numString = String(num);
      Serial.println(hardwareCode + numString + "#" + stateString);
    }
    void serialSignal(String state) {
      String numString = String(num);
      Serial.println(hardwareCode + numString + state);
    }
    //overrideCode option below, used for secondary use of multifunction hardware (ex, button press on RE)
    void serialSignal(int state, String overrideCode) {
      String stateString = String(state);
      String numString = String(num);
      Serial.println(overrideCode + numString + stateString);
    }
    void serialSignal(double state, String overrideCode) {
      String stateString = String(state);
      String numString = String(num);
      Serial.println(overrideCode + numString + "#" + stateString);
    }
    void serialSignal(String state, String overrideCode) {
      String numString = String(num);
      Serial.println(overrideCode + numString + state);
    }
};

//Derived from hardware class
class RotaryEncoder: public Hardware {
  public:
    int clkPin;
    int dtPin;
    bool sw;
    int swPin;
    String currentDir;

    int ccwButton;
    int cwButton;
    
    volatile unsigned char state;

    int currentStateCLK;
    int lastStateCLK;
    unsigned long lastButtonPress;


    //Constructrs below, use top one if you dont use rotary button, use bottom if you include rotary button
    RotaryEncoder(int clkPinInput, int dtPinInput) {
      hardwareCode = "RE";
      num = NUMENCODERS;
      clkPin = clkPinInput;
      dtPin = dtPinInput;
      sw = false;
      NUMENCODERS++;
      lastButtonPress = 0;
      lastStateCLK = digitalRead(clkPinInput);

      ccwButton = gamePadButtonNum;
      cwButton = ccwButton + 1;
      gamePadButtonNum = gamePadButtonNum + 2;



      // Set encoder pins as inputs
      pinMode(clkPin,INPUT);
      pinMode(dtPin,INPUT);
    }
    RotaryEncoder(int clkPinInput, int dtPinInput, int swPinInput) {
      hardwareCode = "RE";
      num = NUMENCODERS;
      clkPin = clkPinInput;
      dtPin = dtPinInput;
      sw = true;
      swPin = swPinInput;
      NUMENCODERS++;
      lastButtonPress = 0;
      lastStateCLK = digitalRead(clkPinInput);

      // Set encoder pins as inputs
      pinMode(clkPin,INPUT);
      pinMode(dtPin,INPUT);
      pinMode(swPin, INPUT_PULLUP);

      ccwButton = gamePadButtonNum;
      cwButton = ccwButton + 1;
      gamePadButtonNum = gamePadButtonNum + 2;




    }



    //Rotary Encoder Functions Below

    //readEncoder will print a code to serial to be read in by the simulator
    //Again, the code cypher can be found below V V V
    //https://docs.google.com/spreadsheets/d/1s-_qcMhMEQdyH7Sp79ui6K6VUlbxQhxz6EQCZoBVXGc/edit?usp=sharing
    /*void readEncoder() {
      
      //I found this code, and modified slightly
      //found here: https://lastminuteengineers.com/rotary-encoder-arduino-tutorial/

      // Read the current state of CLK
      currentStateCLK = digitalRead(clkPin);
      // If last and current state of CLK are different, then pulse occurred
      // React to only 1 state change to avoid double count
      if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
        // If the DT state is different than the CLK state then
        // the encoder is rotating CW
        if (digitalRead(dtPin) != currentStateCLK) {
          currentDir ="CW";
          
        } else {
          // Encoder is rotating CCW
          currentDir ="CC";
        }
        serialSignal(currentDir);
      }
      // Remember last CLK state
      lastStateCLK = currentStateCLK;
      // Read the button state

      //only look at switch pin if switch pin exists
      if(sw) {
        int btnState = digitalRead(swPin);
        //If we detect LOW signal, button is pressed
        if (btnState == LOW) {
          //if 50ms have passed since last LOW pulse, it means that the
          //button has been pressed, released and pressed again
          if (millis() - lastButtonPress > 50) {
            serialSignal("C", "RB");
        }
        // Remember last button press event
        lastButtonPress = millis();
        }
      }
      // Put in a slight delay to help debounce the reading
      //delay(1);
    } */

    unsigned char readEncoder() {
      
      if(sw) {
        int btnState = digitalRead(swPin);
        //If we detect LOW signal, button is pressed
        if (btnState == LOW) {
          //if 50ms have passed since last LOW pulse, it means that the
          //button has been pressed, released and pressed again
          if (millis() - lastButtonPress > 50) {
            serialSignal("C", "RB");
        }
        // Remember last button press event
        lastButtonPress = millis();
        }
      }

  
      
      unsigned char pinstate = (digitalRead(clkPin) << 1) | digitalRead(dtPin);
      state = ttable[state & 0xf][pinstate];
      return (state & 0x30);
    }
    	
};

class Potentiometer: public Hardware {
  public:
    int pin;
    double lastSentOutputNum;
    String axis;


    Potentiometer(int pinInput, String axisInput) {
      hardwareCode = "RP";
      num = NUMPOTENTIOMETERS;
      pin = pinInput;
      lastSentOutputNum = getAverageOutputOutOfOne();
      serialSignal(lastSentOutputNum);
      axis = axisInput;

      NUMPOTENTIOMETERS++;

      updateJoystick();
      
    }

    //gets the average output from potentiometer and converts it to a double between 0.00 and 1.00
    double getAverageOutputOutOfOne(){
      double total = 0.0;
      
      //20 is just a magic number for the number of readings to take, dont think about it :)
      for (int i = 0; i < 20; i++) {
        total = total + analogRead(pin);
      }
      return total / (20 * 1023.0);
    }


    double getAverageOutput(){
      double total = 0.0;
      
      //20 is just a magic number for the number of readings to take, dont think about it :)
      for (int i = 0; i < 20; i++) {
        total = total + analogRead(pin);
      }
      return total / (20);
    }

    
    void updateJoystick() {
      int currentOutputLevel = getAverageOutput();
        if(axis == "x") {
          xAxis_ = map(currentOutputLevel,0,1023,0,255);
          Joystick.setXAxis(xAxis_); 
        } else if (axis == "y") {
          yAxis_ = map(currentOutputLevel,0,1023,0,255);
          Joystick.setYAxis(yAxis_); 
        } else if (axis == "z") {
          zAxis_ = map(currentOutputLevel,0,1023,0,255);
          Joystick.setZAxis(zAxis_); 
        } else if (axis == "rx") {
          rxAxis_ = map(currentOutputLevel,0,1023,0,255);
          Joystick.setRxAxis(rxAxis_); 
        } else if (axis == "ry") {
          ryAxis_ = map(currentOutputLevel,0,1023,0,255);
          Joystick.setRyAxis(ryAxis_); 
        } else if (axis == "rx") {
          rzAxis_ = map(currentOutputLevel,0,1023,0,255);
          Joystick.setRzAxis(rzAxis_); 
        }
    }

    //reads the potentiometer, if the reading changes by atleast 0.01, sends serialSignal
    void readPotentiometer() {
      double potentiometerOutOfOne = getAverageOutputOutOfOne();
      if(abs(potentiometerOutOfOne - lastSentOutputNum) >= 0.01) {
        serialSignal(potentiometerOutOfOne);
        lastSentOutputNum = potentiometerOutOfOne;
        updateJoystick();

        
      }


      


      
    }
};



void setup() {
  Serial.begin(9600);
  Joystick.begin();
  NUMENCODERS = 0;
  NUMPOTENTIOMETERS = 0;
  gamePadButtonNum = 0;
 
}


//DEFINE ENCODERS HERE!
RotaryEncoder RE0(6, 7, 13);
RotaryEncoder RE1(4, 5);
RotaryEncoder RE2(2, 3);
RotaryEncoder RE3(0, 1);
//PUT ENCODER NAMES IN FOLLOWING ARRAY
RotaryEncoder REArray[] = {RE0, RE1, RE2, RE3};

//DEFINE POTENTIOMETERS HERE
Potentiometer RP0(A4, "x");
Potentiometer RP1(A5, "z");
//PUT POTENTIOMETER NAMES IN FOLLOWING ARRAY
Potentiometer RPArray[] = {RP0, RP1};


/*void readAllEncoders() {
  int totalEncoders = sizeof(REArray)/sizeof(REArray[0]);
  for(int i = 0; i < totalEncoders; i++) {
    REArray[i].readEncoder();
  }
} */
void readAllEncoders() {
  int totalEncoders = sizeof(REArray)/sizeof(REArray[0]);

  for(int i = 0; i < totalEncoders; i++) {
    unsigned char result = REArray[i].readEncoder();


    if (result == DIR_CCW) {
      REArray[i].serialSignal("CC");

      Joystick.setButton(REArray[i].ccwButton, 1);
      delay(50);
      Joystick.setButton(REArray[i].ccwButton, 0);

    } else if(result == DIR_CW) {
      REArray[i].serialSignal("CW");

      Joystick.setButton(REArray[i].cwButton, 1);
      delay(50);
      Joystick.setButton(REArray[i].cwButton, 0);

    }
  }
}

void readAllPotentiometers() {
  int totalPotentiometers = sizeof(RPArray)/sizeof(RPArray[0]);
  for(int i = 0; i < totalPotentiometers; i++) {
    RPArray[i].readPotentiometer();
  }

}


void loop() {
  // put your main code here, to run repeatedly:
  readAllEncoders();
  readAllPotentiometers();

}
