#include <Servo.h>

#define REG_MAX 780
#define POS_MAX 180

enum
{
  STATE_STOP = 0,
  STATE_RECORD,
  STATE_PLAY
};

Servo servo01;
int servo01Pos;
int posArm01;

int controlState;

void SetServoControl(Servo *servo, int position)
{
  if(position > POS_MAX)
  {
    position = POS_MAX;
  }
    servo->write(position);
}

int GetArmPosition(int regInput)
{
  int armPosition = map(regInput, 0, REG_MAX, 0, POS_MAX);
  return armPosition;
}

void setup() {
  // put your setup code here, to run once:  
  servo01.attach(5);
  delay(20);

  controlState = STATE_PLAY;
}

void loop() {
  // put your main code here, to run repeatedly:
  
  switch(controlState)
  {
    case STATE_STOP:
    break;

    case STATE_RECORD:
    break;

    case STATE_PLAY:
    posArm01 = GetArmPosition(analogRead(3));
    SetServoControl(&servo01, posArm01); 
    break;
    
    default:
    break;
  }
}
