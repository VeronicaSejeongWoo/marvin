#include <Servo.h>

#define REG_MAX 1023
#define POS_MAX 180

typedef struct
{
  int count;
  int posArm[100];
}ServoData;

enum
{
  STATE_STOP = 0,
  STATE_RECORD,
  STATE_PLAY
};

Servo servo01;
ServoData servoData01;

int controlState;
int count;

void SetServoControl(Servo *servo, int position)
{
  position = min(position, POS_MAX);
  servo->write(position);
}

int GetArmPosition(int regInput)
{
  int armPosition = map(regInput, 0, REG_MAX, 0, POS_MAX);
  return armPosition;
}

void SaveArmPosition(ServoData *data, int channel, int count)
{
  int regValue = analogRead(channel);
  data->posArm[count] = GetArmPosition(regValue);
//  Serial.println(regValue);
  Serial.println(data->posArm[count]);
}

void setup() {
  // put your setup code here, to run once: 
  Serial.begin(9600); 
  servo01.attach(5);
  delay(20);

  servoData01.count = 100;
  for(int i=0; i<100; i++)
  {
    servoData01.posArm[i] = 0;
  }

  count = 0;
  controlState = STATE_RECORD;
  Serial.println("Record Started!");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  switch(controlState)
  {
    case STATE_STOP:
    break;

    case STATE_RECORD:
    Serial.println(count);
    SaveArmPosition(&servoData01, 3, count);
    delay(200);
    count++;
    if(count == 50)
    {
      count = 0;
      controlState = STATE_PLAY;
      Serial.println("Play!");
    }
    break;

    case STATE_PLAY:
    SetServoControl(&servo01, servoData01.posArm[count]); 
    Serial.println(servoData01.posArm[count]);
    delay(100);
    count++;
    
    if(count == 100)
    {
      count = 0;
//      controlState = STATE_STOP;
      Serial.println("PLAY again!");
    }
    break;
    
    default:
    break;
  }
}
