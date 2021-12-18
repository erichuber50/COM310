#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4LCD display;
// Zumo32U4OLED display;

Zumo32U4ButtonA buttonA;
Zumo32U4Buzzer buzzer;
Zumo32U4Motors motors;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;

unsigned int lineSensorValues[3];

const uint16_t lineSensorThreshold = 200;
const uint16_t reverseSpeed = 200;
const uint16_t turnSpeed = 200;
const uint16_t forwardSpeed = 200;
const uint16_t veerSpeedLow = 0;
const uint16_t veerSpeedHigh = 250;
const uint16_t rammingSpeed = 400;
const uint16_t reverseTime = 200;
const uint16_t scanTimeMin = 200;
const uint16_t scanTimeMax = 2100;
const uint16_t waitTime = 5000;
const uint16_t stalemateTime = 4000;

enum State
{
  StatePausing,
  StateWaiting,
  StateScanning,
  StateDriving,
  StateBacking,
};

State state = StatePausing;

enum Direction
{
  DirectionLeft,
  DirectionRight,
};

Direction scanDir = DirectionLeft;

uint16_t stateStartTime;

uint16_t displayTime;

bool justChangedState;

bool displayCleared;

void setup()
{
  //motors.flipLeftMotor(true);
  //motors.flipRightMotor(true);

  lineSensors.initThreeSensors();
  proxSensors.initThreeSensors();

  changeState(StatePausing);
}

void loop()
{
  bool buttonPress = buttonA.getSingleDebouncedPress();

  if (state == StatePausing)
  {
    motors.setSpeeds(0, 0);

    if (justChangedState)
    {
      justChangedState = false;
      display.print(F("Press A"));
    }

    if (displayIsStale(100))
    {
      displayUpdated();
      display.gotoXY(0, 1);
      display.print(readBatteryMillivolts());
    }

    if (buttonPress)
    {
      changeState(StateWaiting);
    }
  }
  else if (buttonPress)
  {
    changeState(StatePausing);
  }
  else if (state == StateWaiting)
  {
    motors.setSpeeds(0, 0);

    uint16_t time = timeInThisState();

    if (time < waitTime)
    {
      uint16_t timeLeft = waitTime - time;
      display.gotoXY(0, 0);
      display.print(timeLeft / 1000 % 10);
      display.print('.');
      display.print(timeLeft / 100 % 10);
    }
    else
    {
      changeState(StateScanning);
    }
  }
  else if (state == StateBacking)
  {
    if (justChangedState)
    {
      justChangedState = false;
      display.print(F("back"));
    }

    motors.setSpeeds(-reverseSpeed, -reverseSpeed);
    
    if (timeInThisState() >= reverseTime)
    {
      changeState(StateScanning);
    }
  }
  else if (state == StateScanning)
  {

    if (justChangedState)
    {
      justChangedState = false;
      display.print(F("scan"));
    }

    if (scanDir == DirectionRight)
    {
      motors.setSpeeds(turnSpeed, -turnSpeed);
    }
    else
    {
      motors.setSpeeds(-turnSpeed, turnSpeed);
    }

    uint16_t time = timeInThisState();

    if (time > scanTimeMax)
    {
      changeState(StateDriving);
    }
    else if (time > scanTimeMin)
    {
      proxSensors.read();
      if (proxSensors.countsFrontWithLeftLeds() >= 2
        || proxSensors.countsFrontWithRightLeds() >= 2)
      {
        changeState(StateDriving);
      }
    }
  }
  else if (state == StateDriving)
  {
    if (justChangedState)
    {
      justChangedState = false;
      display.print(F("drive"));
    }
    lineSensors.read(lineSensorValues);
    Serial.println(lineSensorValues[0]);
    Serial.println(lineSensorValues[1]);
    Serial.println(lineSensorValues[2]);
    Serial.println();
    if (lineSensorValues[0] < lineSensorThreshold)
    {
      scanDir = DirectionRight;
      changeState(StateBacking);
    }
    if (lineSensorValues[2] < lineSensorThreshold)
    {
      scanDir = DirectionLeft;
      changeState(StateBacking);
    }

    proxSensors.read();
    uint8_t sum = proxSensors.countsFrontWithRightLeds() + proxSensors.countsFrontWithLeftLeds();
    int8_t diff = proxSensors.countsFrontWithRightLeds() - proxSensors.countsFrontWithLeftLeds();

    if (sum >= 4 || timeInThisState() > stalemateTime)
    {
      motors.setSpeeds(rammingSpeed, rammingSpeed);
      ledRed(1);
    }
    else if (sum == 0)
    {
      motors.setSpeeds(forwardSpeed, forwardSpeed);

      if (proxSensors.countsLeftWithLeftLeds() >= 2)
      {
        scanDir = DirectionLeft;
        changeState(StateScanning);
      }

      if (proxSensors.countsRightWithRightLeds() >= 2)
      {
        scanDir = DirectionRight;
        changeState(StateScanning);
      }

      ledRed(0);
    }
    else
    {
      if (diff >= 1)
      {
        motors.setSpeeds(veerSpeedHigh, veerSpeedLow);
      }
      else if (diff <= -1)
      {
        motors.setSpeeds(veerSpeedLow, veerSpeedHigh);
      }
      else
      {
        motors.setSpeeds(forwardSpeed, forwardSpeed);
      }
      ledRed(0);
    }
  }
}

uint16_t timeInThisState()
{
  return (uint16_t)(millis() - stateStartTime);
}

void changeState(uint8_t newState)
{
  state = (State)newState;
  justChangedState = true;
  stateStartTime = millis();
  ledRed(0);
  ledYellow(0);
  ledGreen(0);
  display.clear();
  displayCleared = true;
}

bool displayIsStale(uint16_t staleTime)
{
  return displayCleared || (millis() - displayTime) > staleTime;
}

void displayUpdated()
{
  displayTime = millis();
  displayCleared = false;
}
