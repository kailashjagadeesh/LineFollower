#include <string.h>
#define button 2
#define redLED 14
#define yellowLED 15
// using namespace std;
enum sensor
{
  RB2 = 0,
  RB1,
  RM,
  RF,
  LF,
  LM,
  LB1,
  LB2
};
enum Junction
{
  R,
  L,
  X,
  T,
  LS,
  RS,
  Y,
  BL,
  END,
  FOLLOW,
  FINISH,
  INVALID
};
enum Algorithm
{
  LEFT_LOGIC,
  RIGHT_LOGIC
};

enum mode
{
  DRY_RUN,
  ACTUAL_RUN
}Mode;

char junctionNames[][8] = {"R", "L", "X", "T", "LS", "RS", "Y", "BLOCK","END", "FOLLOW","FINISH", "INVALID"};
volatile uint8_t backSensorState = 0; // Contains state of all sensors except the front extreme one at any time
volatile bool CFState = 0;            // 1-Means a white line under sensor
volatile bool blockDetectFlag = 0;
Algorithm algorithm;
char data[9];
uint8_t choiceJunction = 0;
char junctionsTraversed[25] = ""; // Holds all the choices taken so far
Ultrasonic ultrasonic;
void junctionControl(Junction J, mode m) // Take appropriate action based on the junction detected
{
  uint8_t backSensors;
  if (m == DRY_RUN)
  {
    if (algorithm == LEFT_LOGIC)
    {
      switch (J)
      {
      case BL: // Case for block detected in the way. Action : Take  U-turn
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (CFState == 1); // Bank left till the CF sensor gets out of line

        delay(50);

        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (!((backSensors == 0b00011000) && (CFState == 1))); // Keep turning until normal line follow condition is met
        strcat(junctionsTraversed, "B");

        break;
      case L:
        do
        {
          //  backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (!((backSensors == 0b00011000) && (CFState == 1)));
        break;
      case R:
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        } while (!((backSensors == 0b00011000) && (CFState == 1)));
        break;
      case T:

        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (!((backSensors == 0b00011000) && (CFState == 1)));
        strcat(junctionsTraversed, "L");

        break;
      case Y:

        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (!((backSensors == 0b00011000) && (CFState == 1)));
        strcat(junctionsTraversed, "L");

        break;

      case RS: // Simply move forward
        strcat(junctionsTraversed, "S");
        break;
      case LS:
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (CFState == 1); // Bank left till the CF sensor gets out of line

        delay(50);

        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (!((backSensors == 0b00011000) && (CFState == 1))); // Keep turning until normal line follow condition is met
        strcat(junctionsTraversed, "L");
        break;

      case X:

        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (CFState == 1);

        delay(50);

        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (!((backSensors == 0b00011000) && (CFState == 1)));
        strcat(junctionsTraversed, "L");
        break;
      case END:
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (!((backSensors == 0b00011000) && (CFState == 1)));

        strcat(junctionsTraversed, "B");
        break;

      case FINISH:
        motor.stopMotors();
        // bluetooth.println(junctionsTraversed);
        // ShortestPath(junctionsTraversed);
        // bluetooth.println(junctionsTraversed);
        digitalWrite(yellowLED, LOW);
        while (digitalRead(button))
        {
          digitalWrite(redLED, HIGH);
        }
        digitalWrite(yellowLED, HIGH);
        delay(1000);
        Mode = ACTUAL_RUN;
        return;
      }
    }
    else if (algorithm == RIGHT_LOGIC)
    {

      switch (J)
      {
      case BL: // Case for block detected in the way. Action : Take  U-turn
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (CFState == 1); // Bank left till the CF sensor gets out of line

        delay(50);

        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (!((backSensors == 0b00011000) && (CFState == 1))); // Keep turning until normal line follow condition is met
        strcat(junctionsTraversed, "B");

        break;
      case L:
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        } while (!((backSensors == 0b00011000) && (CFState == 1)));
        break;
      case R:
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        } while (!((backSensors == 0b00011000) && (CFState == 1)));
        break;
      case T:
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        } while (!((backSensors == 0b00011000) && (CFState == 1)));
        strcat(junctionsTraversed, "R");
        break;
      case Y:

        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (!((backSensors == 0b00011000) && (CFState == 1)));
        strcat(junctionsTraversed, "R");

        break;

      case RS:
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        } while (CFState == 1); // Bank right till the CF sensor gets out of line

        delay(50);

        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        } while (!((backSensors == 0b00011000) && (CFState == 1))); // Keep turning until normal line follow condition is met
        strcat(junctionsTraversed, "R");
        break;
      case LS: // Simply move forward
        strcat(junctionsTraversed, "S");

        break;

      case X:
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        } while (CFState == 1);

        delay(50);

        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        } while (!((backSensors == 0b00011000) && (CFState == 1)));
        strcat(junctionsTraversed, "R");
        break;

      case END:
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (!((backSensors == 0b00011000) && (CFState == 1)));

        strcat(junctionsTraversed, "B");
        break;

      case FINISH:
        motor.stopMotors();
        // bluetooth.println(junctionsTraversed);
        // ShortestPath(junctionsTraversed);
        // bluetooth.println(junctionsTraversed);
        digitalWrite(yellowLED, LOW);
        while (digitalRead(button))
        {
          digitalWrite(redLED, HIGH);
        }
        digitalWrite(yellowLED, HIGH);
        delay(1000);
        Mode = ACTUAL_RUN;
        return;
      }
    }
  }
  else if (m == ACTUAL_RUN)
  {
    switch (J)
    {
    case BL: // Case for block detected in the way. Action : Take  U-turn
      do
      {
        // backSensors = sensorValuesInBinary();

        motor.setLeftDirection(Motor::Back);
        motor.setRightDirection(Motor::Front);
        motor.setLeftSpeed(MOTOR_TURN_SPEED);
        motor.setRightSpeed(MOTOR_TURN_SPEED);

      } while (CFState == 1); // Bank left till the CF sensor gets out of line

      delay(50);

      do
      {
        // backSensors = sensorValuesInBinary();

        motor.setLeftDirection(Motor::Back);
        motor.setRightDirection(Motor::Front);
        motor.setLeftSpeed(MOTOR_TURN_SPEED);
        motor.setRightSpeed(MOTOR_TURN_SPEED);

      } while (!((backSensors == 0b00011000) && (CFState == 1))); // Keep turning until normal line follow condition is met
      strcat(junctionsTraversed, "B");

      break;
    case L:
      do
      {
        // backSensors = sensorValuesInBinary();

        motor.setLeftDirection(Motor::Back);
        motor.setRightDirection(Motor::Front);
        motor.setLeftSpeed(MOTOR_TURN_SPEED);
        motor.setRightSpeed(MOTOR_TURN_SPEED);

      } while (!((backSensors == 0b00011000) && (CFState == 1)));
      break;
    case R:
      do
      {
        // backSensors = sensorValuesInBinary();

        motor.setRightDirection(Motor::Back);
        motor.setLeftDirection(Motor::Front);
        motor.setLeftSpeed(MOTOR_TURN_SPEED);
        motor.setRightSpeed(MOTOR_TURN_SPEED);
      } while (!((backSensors == 0b00011000) && (CFState == 1)));
      break;
    case T:
      if (junctionsTraversed[choiceJunction] == 'L')
      {
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (!((backSensors == 0b00011000) && (CFState == 1)));
      }
      else if (junctionsTraversed[choiceJunction] == 'R')
      {
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        } while (!((backSensors == 0b00011000) && (CFState == 1)));
      }
      ++choiceJunction;
      break;
    case Y:
      if (junctionsTraversed[choiceJunction] == 'L')
      {
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);

        } while (!((backSensors == 0b00011000) && (CFState == 1)));
      }
      else if (junctionsTraversed[choiceJunction] == 'R')
      {
        do
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        } while (!((backSensors == 0b00011000) && (CFState == 1)));
      }
      ++choiceJunction;
      break;
    case RS:
      do
      {
        if (junctionsTraversed[choiceJunction] == 'R')
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        }

      } while ((CFState == 1) && (junctionsTraversed[choiceJunction] != 'S'));

      delay(50);

      do
      {
        if (junctionsTraversed[choiceJunction] == 'R')
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        }

      } while ((!((backSensors == 0b00011000) && (CFState == 1))) && (junctionsTraversed[choiceJunction] != 'S'));
      ++choiceJunction;
      break;
    case LS:
      do
      {
        if (junctionsTraversed[choiceJunction] == 'L')
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        }

      } while ((CFState == 1) && (junctionsTraversed[choiceJunction] != 'S'));

      delay(50);

      do
      {
        if (junctionsTraversed[choiceJunction] == 'L')
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        }
      } while ((!((backSensors == 0b00011000) && (CFState == 1))) && (junctionsTraversed[choiceJunction] != 'S'));
      ++choiceJunction;

      break;
    case X:

      do
      {

        if (junctionsTraversed[choiceJunction] == 'L')
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        }
        else if (junctionsTraversed[choiceJunction] == 'R')
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        }
      } while ((CFState == 1) && (junctionsTraversed[choiceJunction] != 'S'));

      delay(50);

      do
      {
        // backSensors = sensorValuesInBinary();
        if (junctionsTraversed[choiceJunction] == 'L')
        {
          // backSensors = sensorValuesInBinary();

          motor.setLeftDirection(Motor::Back);
          motor.setRightDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        }
        else if (junctionsTraversed[choiceJunction] == 'R')
        {
          // backSensors = sensorValuesInBinary();

          motor.setRightDirection(Motor::Back);
          motor.setLeftDirection(Motor::Front);
          motor.setLeftSpeed(MOTOR_TURN_SPEED);
          motor.setRightSpeed(MOTOR_TURN_SPEED);
        }

      } while ((!((backSensors == 0b00011000) && (CFState == 1))) && (junctionsTraversed[choiceJunction] != 'S'));
      choiceJunction++;
      break;
    case END:
      do
      {
        // backSensors = sensorValuesInBinary();

        motor.setLeftDirection(Motor::Back);
        motor.setRightDirection(Motor::Front);
        motor.setLeftSpeed(MOTOR_TURN_SPEED);
        motor.setRightSpeed(MOTOR_TURN_SPEED);

      } while (!(backSensors == 0b01111000));

      break;

    case FINISH:
      motor.stopMotors();
      while (1)
      {
        digitalWrite(redLED, HIGH);
        delay(750);
        digitalWrite(redLED, LOW);
        delay(750);
      }
      break;
    }
    if (junctionsTraversed[choiceJunction] == 0)
    {
      m = DRY_RUN;
    }
  }
}
/*
Calling this function detects junction and transfers control
to either PID or junctionControl() for turning in junctions. 
*/
int findAndAct(mode m)
{
  uint8_t backSensorStateInverted = (backSensorState * 0x0202020202ULL & 0x010884422010ULL) % 1023; // Contains backSensorState's mirror inversion
  blockDetectFlag =  ultrasonic.detectBlock();
  // Block detected condition
  if (blockDetectFlag == 1)
  {
    junctionControl(BL, m);
    blockDetectFlag = 0;
  }
  // Normal Line follow
  else if (((CFState == 1) && (backSensorState == 0b00011000)) || ((CFState == 0) && (backSensorState == 0b00110000) || (backSensorState == 0b00001100)))
  {
    return FOLLOW; // Return control back to PID
  }
  else if ((CFState == 0) && ((backSensorState == 0b00111100) || (backSensorState == 0b00111110) || (backSensorState == 0b01111100)))
  {
    junctionControl(Y, m);
  }
  else if ((CFState == 1) && (backSensorState == 255))
  {
    junctionControl(X, m);
  }
  else if ((CFState == 0) && (backSensorState == 255))
  {
    junctionControl(T, m);
  }
  // Straight Right detection
  else if ((CFState == 1) && (((backSensorState <= 0b00001111) && (backSensorState >= 0b00001100)) || ((backSensorState <= 0b00011111) && (backSensorState >= 0b00011100))))
  {
    junctionControl(RS, m);
  }
  // Straight Left detection
  else if ((CFState == 1) && (((backSensorStateInverted <= 0b00001111) && (backSensorStateInverted >= 0b00001100)) || ((backSensorStateInverted <= 0b00011111) && (backSensorStateInverted >= 0b00011100))))
  {
    junctionControl(LS, m);
  }
  //90 degree right
  else if ((CFState == 0) && (((backSensorState <= 0b00001111) && (backSensorState >= 0b00001100)) || ((backSensorState <= 0b00011111) && (backSensorState >= 0b00011100))))
  {
    junctionControl(R, m);
  }
  //90 degrees left
  else if ((CFState == 0) && (((backSensorStateInverted <= 0b00001111) && (backSensorStateInverted >= 0b00001100)) || ((backSensorStateInverted <= 0b00011111) && (backSensorStateInverted >= 0b00011100))))
  {
    junctionControl(L, m);
  }
  else
  {
    return INVALID;
  }
  return 100;
}
