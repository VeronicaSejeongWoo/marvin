#include <Servo.h>

#define REG_MAX 1023
#define REG_MAX1 734

#define POS_MAX 180

#define ARM_REG_CHANNEL1 8

typedef struct
{
  int count;
  int posArm[180];
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
  Serial.println(position);
}

int GetArmPosition(int regInput)
{
  int armPosition = map(regInput, 0, REG_MAX1, 0, POS_MAX);
  return armPosition;
}

void SaveArmPosition(ServoData *data, int channel, int count)
{
  int regValue = analogRead(channel);
  data->posArm[count] = GetArmPosition(regValue);
}

void UpdateCount(int savedCount)
{
  count++;
  
  if(controlState == STATE_RECORD)
  {
    count = min(count, 180);
    savedCount = count;
  }
  else if(controlState ==  STATE_PLAY)
  {
    if(count >= savedCount)
    {
      count = 0;
    }    
  }
}

void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR (PCINT2_vect)
{
  if(digitalRead(2) == 1)
  {
    digitalWrite(7, LOW);
    controlState = STATE_STOP;
    Serial.println("Stop");
  }

  if(digitalRead(3) == 1)
  {
    digitalWrite(7, LOW);
    controlState = STATE_RECORD;
    Serial.println("Record");
  }

  if(digitalRead(4) == 1)
  {
    digitalWrite(7, HIGH);
    controlState = STATE_PLAY;  
    Serial.println("Play");
  }
}

int pos;

void setup() {
  // put your setup code here, to run once: 
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);

  // Relay
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);

  pciSetup(2);
  pciSetup(3);
  pciSetup(4);
  
  Serial.begin(9600); 
  servo01.attach(6);
  delay(20);

  servoData01.count = 180;
  for(int i=0; i<180; i++)
  {
    servoData01.posArm[i] = 0;
  }

  count = 0;
  controlState = STATE_STOP;
  Serial.println("RobotArm Started!");
}

void loop() {
  int tempValue;
  // put your main code here, to run repeatedly:
  
  switch(controlState)
  {
    case STATE_STOP:
    tempValue = analogRead(ARM_REG_CHANNEL1);
    Serial.println(tempValue);
    delay(500);
    break;

    case STATE_RECORD:
    SaveArmPosition(&servoData01, ARM_REG_CHANNEL1, count);
    UpdateCount(servoData01.count);
    delay(200);
    break;

    case STATE_PLAY:
    SetServoControl(&servo01, servoData01.posArm[count]); 
    UpdateCount(servoData01.count);
    delay(15);
    break;
    
    default:
    break;
  }
}
