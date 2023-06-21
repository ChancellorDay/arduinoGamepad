#include <RotaryEncoder.h>
#include <Keypad.h>
#include <Joystick.h>



unsigned long CURRENTMILLIS;
int NUMENCODERS;
int NUMPOTENTIOMETERS;
int GAMEPADBUTTONNUM;



Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_GAMEPAD,
  32, //number of buttons
  0, //number of hat switches
  //Set as many axis to "true" as you have potentiometers for
  true, // x axis
  true, // y axis
  true, // z axis
  true, // rx axis
  true, // ry axis
  true, // rz axis
  true, // rudder
  true, // throttle
  true, // accelerator
  true, // brake
  true); // steering wheel
const bool initAutoSendState = true;

void axisSet(int axisIdInput, int value) {
  switch(axisIdInput) {
    case 0:
      Joystick.setXAxis(value);
      break;
    case 1:
      Joystick.setYAxis(value);
      break;
    case 2:
      Joystick.setZAxis(value);
      break;
    case 3:
      Joystick.setRxAxis(value);
      break;
    case 4:
      Joystick.setRyAxis(value);
      break;
    case 5:
      Joystick.setRzAxis(value);
      break;
    case 6:
      Joystick.setRudder(value);
      break;
    case 7:
      Joystick.setThrottle(value);
      break;
    case 8:
      Joystick.setAccelerator(value);
      break;
    case 9:
      Joystick.setBrake(value);
      break;
    case 10:
      Joystick.setSteering(value);
      break;
    default:
      return;
  }
}



class Button {
  public:
    int pin;
    uint8_t gamePadButton;
    bool pressed;
    unsigned long lastButtonPress;

    Button(uint8_t pinInput) {
      pin = pinInput;
      pinMode(pin, INPUT);
      digitalWrite(pin, HIGH);
      gamePadButton = GAMEPADBUTTONNUM;
      GAMEPADBUTTONNUM++;
      pressed = false;
      lastButtonPress = millis();
    }

    void buttonLogic(bool tryToPress) {

      if (tryToPress && !pressed) {
        Joystick.pressButton(gamePadButton);
        pressed = true;
      } else if (!tryToPress && pressed) {
        if(lastButtonPress - CURRENTMILLIS > 100) {
          Joystick.releaseButton(gamePadButton);
          pressed = false;
        }
        
      }

    }

    void readButton() {
      int btnState = digitalRead(pin);
        //If we detect LOW signal, button is pressed
        if (btnState == LOW) {
          //if 50ms have passed since last LOW pulse, it means that the
          //button has been pressed, released and pressed again
          if (CURRENTMILLIS - lastButtonPress > 100) {
            buttonLogic(true);
        }
        // Remember last button press event
        lastButtonPress = CURRENTMILLIS;
        } else {
          buttonLogic(false);
        }
    }
    


};

class Potentiometer {
  public:
    int pin;
    int axisId;

    Potentiometer (int pinInput, int axisIdInput) {
      pin = pinInput;
      axisId = axisIdInput;
    }

    double getAverageOutput(){
      double total = 0.0;
      
      //20 is just a magic number for the number of readings to take, dont think about it :)
      for (int i = 0; i < 20; i++) {
        total = total + analogRead(pin);
      }
      return total / (20);
    }

    void gamepadAxisLogic() {
      axisSet(axisId, map(getAverageOutput(),0, 1023, -500, 500));
    }


};

class REWrapper {
  public:
    unsigned long lastButtonPress;
    int RENum;
    bool sw;
    int swPin;
    RotaryEncoder REWrapped = RotaryEncoder(6, 7);
    bool pressed;
    uint8_t gamePadButton;
    int axisId;
    int direction;
    uint8_t cwGamepadButton;
    uint8_t ccwGamepadButton;
    unsigned long lastTurn;
    bool turnPressed;
    long currentPosition;
    long lastPosition;

    REWrapper(int clkPinInput, int dtPinInput, int axisIdInput) {
      lastButtonPress = millis();
      lastTurn = millis();
      RotaryEncoder RE(dtPinInput, clkPinInput);
      RENum = NUMENCODERS;
      NUMENCODERS++;
      REWrapped = RE;
      sw = false;
      pressed = false;
      ccwGamepadButton = GAMEPADBUTTONNUM;
      cwGamepadButton = GAMEPADBUTTONNUM + 1;
      GAMEPADBUTTONNUM = GAMEPADBUTTONNUM + 2;
      turnPressed = false;
      lastPosition = 0;
      currentPosition = 0;

      direction = 1;

      axisId = axisIdInput;
      
    }


    REWrapper(int dtPinInput, int clkPinInput, int swPinInput, int axisIdInput) {
      lastButtonPress = millis();
      lastTurn = millis();
      RotaryEncoder RE(clkPinInput, dtPinInput);
      RENum = NUMENCODERS;
      NUMENCODERS++;
      REWrapped = RE;
      sw = true;
      swPin = swPinInput;
      pressed = false;
      gamePadButton = GAMEPADBUTTONNUM;
      ccwGamepadButton = GAMEPADBUTTONNUM + 1;
      cwGamepadButton = GAMEPADBUTTONNUM + 2;
      GAMEPADBUTTONNUM = GAMEPADBUTTONNUM + 3;
      turnPressed = false;
      lastPosition = 0;
      currentPosition = 0;

      direction = 1;

      axisId = axisIdInput;


      pinMode(swPin, INPUT_PULLUP);
    }

    int getDirectionalRPM() {

      auto directionSet = REWrapped.getDirection();
      if(directionSet == RotaryEncoder::Direction::CLOCKWISE) {
        direction = 1;
      } else if (directionSet == RotaryEncoder::Direction::COUNTERCLOCKWISE) {
        direction = -1;

      }
      int rpm = (int) REWrapped.getRPM();
      return rpm * direction;
    }

    void gamepadAxisLogic() {
      axisSet(axisId, getDirectionalRPM());
    }



    void REButtonLogic(bool tryToPress) {
      if (tryToPress && !pressed) {
        Joystick.pressButton(gamePadButton);
        pressed = true;
      } else if (!tryToPress && pressed) {
        if(lastButtonPress - CURRENTMILLIS > 100) {
          Joystick.releaseButton(gamePadButton);
          pressed = false;
        }
        
      }
    }

    void readREButton() {
      if (!sw) {
        return;
      }
      int btnState = digitalRead(swPin);
        //If we detect LOW signal, button is pressed
        if (btnState == LOW) {
          //if 50ms have passed since last LOW pulse, it means that the
          //button has been pressed, released and pressed again
          if (CURRENTMILLIS - lastButtonPress > 100) {
            REButtonLogic(true);
        }
        // Remember last button press event
        lastButtonPress = CURRENTMILLIS;
        } else {
          REButtonLogic(false);
        }
    }

    void RETurnLogic() {
      currentPosition = REWrapped.getPosition();

      if (turnPressed && (CURRENTMILLIS - lastTurn > 150)) {
        Joystick.releaseButton(cwGamepadButton);
        Joystick.releaseButton(ccwGamepadButton);
        turnPressed = false;
      }

      //Serial.println(currentPosition);

      if(currentPosition > lastPosition) {
        lastPosition = currentPosition;
        //Serial.println("dir 1");
        Joystick.pressButton(cwGamepadButton);
        lastTurn = CURRENTMILLIS;
        turnPressed = true;
      } else if (currentPosition < lastPosition) {
        lastPosition = currentPosition;
        //Serial.println("dir -1");
        Joystick.pressButton(ccwGamepadButton);
        lastTurn = CURRENTMILLIS;
        turnPressed = true;
      }
      
    }

    REWCall() {
      
      REWrapped.tick();
      if(sw) {
        readREButton();
      }
      gamepadAxisLogic();
      RETurnLogic();
    }

};

//Button B0(8);
REWrapper REW0(6, 7, 13, 0);
REWrapper REW1(4, 5, 1);
REWrapper REW2(2, 3, 2);
REWrapper REW3(0, 1, 3);
void setup() {
  CURRENTMILLIS = millis();
  // put your setup code here, to run once:
  NUMENCODERS = 0;
  NUMPOTENTIOMETERS = 0;
  GAMEPADBUTTONNUM = 0;
  Joystick.begin();
  Joystick.setXAxisRange(-500, 500);
  Joystick.setYAxisRange(-500, 500);
  Joystick.setZAxisRange(-500, 500);
  Joystick.setRxAxisRange(-500, 500);
  Joystick.setRyAxisRange(-500, 500);
  Joystick.setRzAxisRange(-500, 500);
  
}

void loop() {
  CURRENTMILLIS = millis();
  REW0.REWCall();
  REW1.REWCall();
  REW2.REWCall();
  REW3.REWCall();
  //B0.readButton();

}
