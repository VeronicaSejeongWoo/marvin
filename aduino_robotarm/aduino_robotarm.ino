#include <Servo.h>

#define REG_MAX 1023
#define REG_MAX1 1000
#define REG_MAX2 590

#define POS_MAX 180
#define POS_DATA_MAX 100

#define ARM_REG_CHANNEL0 5
#define ARM_REG_CHANNEL1 4
#define ARM_REG_CHANNEL2 3

typedef struct
{
  int min;
  int max;
}valueRange;

typedef struct
{
  int id;
  int posArm[POS_DATA_MAX];
}ServoData;

enum
{
  STATE_STOP = 0,
  STATE_RECORD,
  STATE_PLAY
};

const valueRange mappingTable[] = 
{
  {998, 14},
  {590, 0},
  {190, 920},
};

Servo servo0;
ServoData servoData0;
Servo servo1;
ServoData servoData1;
Servo servo2;
ServoData servoData2;

int controlState;
int count;
int savedCount;

volatile int stopKeyPressed;
volatile int recordKeyPressed;
volatile int playKeyPressed;

void SetServoControl(Servo *servo, int position)
{
  position = min(position, POS_MAX);
  servo->write(position);
  Serial.println(position);
}

int GetArmPosition(int regInput, int id)
{
  int armPosition = map(regInput, mappingTable[id].min, mappingTable[id].max, 20, POS_MAX-20);
  return armPosition;
}

void SaveArmPosition(ServoData *data, int channel, int count)
{
  int regValue = analogRead(channel);
  data->posArm[count] = GetArmPosition(regValue, data->id);
}

void UpdateCount()
{
  count++;
  
  if(controlState == STATE_RECORD)
  {
    count = min(count, POS_DATA_MAX);
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
    stopKeyPressed = 1;
  }

  if(digitalRead(3) == 1)
  {
    playKeyPressed = 1;
  }

  if(digitalRead(4) == 1)
  {
    recordKeyPressed = 1;
  }
}

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
  
  servo0.attach(8);
  servo1.attach(9);
  servo2.attach(10);
  delay(20);

  // Servo motor 0
  servoData0.id = 0;
  for(int i=0; i<POS_DATA_MAX; i++)
  {
    servoData0.posArm[i] = 0;
  }

  // Servo motor 1
  servoData1.id = 1;
  for(int i=0; i<POS_DATA_MAX; i++)
  {
    servoData1.posArm[i] = 0;
  }

  // Servo motor 2
  servoData2.id = 2;
  for(int i=0; i<POS_DATA_MAX; i++)
  {
    servoData2.posArm[i] = 0;
  }
  
  count = 0;
  savedCount = 0;
  controlState = STATE_STOP;
  Serial.println("RobotArm Started!");
}

void CheckKeys()
{
  if(stopKeyPressed)
  {
    stopKeyPressed = 0;
    digitalWrite(7, LOW);
    controlState = STATE_STOP;
    Serial.println("Stop");
  }
  else if(recordKeyPressed)
  {
    recordKeyPressed = 0;
    count = 0;
    savedCount = 0;
    digitalWrite(7, LOW);
    controlState = STATE_RECORD;  
    Serial.println("Record");
    for(int i=0; i<POS_DATA_MAX; i++)
    {
      servoData0.posArm[i] = 0;
      servoData1.posArm[i] = 0;
      servoData2.posArm[i] = 0;
    }
  }
    else if(playKeyPressed)
  {
    playKeyPressed = 0;
    count = 0;
    digitalWrite(7, HIGH);
    controlState = STATE_PLAY;  
    Serial.println("Play");
  }
}

void loop() {
  int tempValue;
  // put your main code here, to run repeatedly:

  CheckKeys();
  
  switch(controlState)
  {
    case STATE_STOP:
//    tempValue = analogRead(ARM_REG_CHANNEL1);
//    Serial.println(tempValue);
//    delay(500);
    break;

    case STATE_RECORD:
    SaveArmPosition(&servoData0, ARM_REG_CHANNEL0, count);
    SaveArmPosition(&servoData1, ARM_REG_CHANNEL1, count);
    SaveArmPosition(&servoData2, ARM_REG_CHANNEL2, count);
    UpdateCount();
    delay(200);
    break;

    case STATE_PLAY:
    SetServoControl(&servo0, servoData0.posArm[count]); 
    SetServoControl(&servo1, servoData1.posArm[count]); 
    SetServoControl(&servo2, servoData2.posArm[count]); 
    UpdateCount();
    delay(100);
    break;
    
    default:
    break;
  }
}
