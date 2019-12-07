#include <iostream>
#include <ctype.h>

using namespace std;
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
enum junctions
{
  R,
  L,
  X,
  T,
  SL,
  SR,
  Y,
  FOLLOW,
  INVALID
};
char junctionNames[][8] = {"R", "L", "X", "T", "SL", "SR", "Y", "FOLLOW", "INVALID"};
volatile uint8_t backSensorState = 0; // Contains state of all sensors except the front extreme one at any time
volatile bool CFState = 0;            // 1-Means a white line under sensor
int findState(uint8_t backSensorState = 0, bool CFState = 0)
{
  uint8_t backSensorStateInverted = (backSensorState * 0x0202020202ULL & 0x010884422010ULL) % 1023; // Contains backSensorState's mirror inversion
  // Normal Line follow
  if (((CFState == 1) && (backSensorState == 0b00011000)) || ((CFState == 0) && (backSensorState == 0b00110000) || (backSensorState == 0b00001100)))
  {
    return FOLLOW;
  }
  else if ((CFState == 0) && ((backSensorState == 0b00111100) || (backSensorState == 0b00111110) || (backSensorState == 0b01111100)))
  {
    return Y;
  }
  else if ((CFState == 1) && (backSensorState == 255))
  {
    return X;
  }
  else if ((CFState == 0) && (backSensorState == 255))
  {
    return T;
  }
  // Straight Right detection
  else if ((CFState == 1) && (((backSensorState <= 0b00001111) && (backSensorState >= 0b00001100)) || ((backSensorState <= 0b00011111) && (backSensorState >= 0b00011100))))
  {
    return SR;
  }
  // Straight Left detection
  else if ((CFState == 1) && (((backSensorStateInverted <= 0b00001111) && (backSensorStateInverted >= 0b00001100)) || ((backSensorStateInverted <= 0b00011111) && (backSensorStateInverted >= 0b00011100))))
  {
    return SL;
  }
  //90 degree right
  else if ((CFState == 0) && (((backSensorState <= 0b00001111) && (backSensorState >= 0b00001100)) || ((backSensorState <= 0b00011111) && (backSensorState >= 0b00011100))))
  {
    return R;
  }
  //90 degrees left
  else if ((CFState == 0) && (((backSensorStateInverted <= 0b00001111) && (backSensorStateInverted >= 0b00001100)) || ((backSensorStateInverted <= 0b00011111) && (backSensorStateInverted >= 0b00011100))))
  {
    return L;
  }
  else
  {
    return INVALID;
  }
}

char data[9];
int main()
{
  while (1)
  {
    cout << "Enter data\n";
    cin >> data;

    backSensorState = 0;
    CFState = data[8] - '0';
    for (int i = 0; i < 8; i++)
    {
      backSensorState |= (data[i] - '0') << (7 - i);
    }
    cout << junctionNames[findState(backSensorState, CFState)] << endl;
  }
}
