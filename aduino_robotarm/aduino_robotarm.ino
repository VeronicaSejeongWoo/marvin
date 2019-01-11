#include <Servo.h>
#include <SoftwareSerial.h>

#define POS_MAX 180
#define POS_DATA_MAX 100

#define ARM_REG_CHANNEL0 3
#define ARM_REG_CHANNEL1 4
#define ARM_REG_CHANNEL2 1
#define ARM_REG_CHANNEL_HAND 2

#define ARM_SERVO_CHANNEL0 10
#define ARM_SERVO_CHANNEL1 9
#define ARM_SERVO_CHANNEL2 8
#define ARM_SERVO_CHANNEL_HAND 11

#define STOP_KEY_CHANNEL 2
#define RECORD_KEY_CHANNEL 3
#define PLAY_KEY_CHANNEL 4

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
  {190, 920},
  {590, 0}, 
  {998, 14},
  {300, 0}
};

Servo servo0;
ServoData servoData0;
Servo servo1;
ServoData servoData1;
Servo servo2;
ServoData servoData2;
Servo servoHand;
ServoData servoDataHand;

int controlState;
int requestedState;
int count;
int savedCount;

SoftwareSerial mySerial(12, 13); // TX, RX
String command = "";

void SetServoControl(Servo *servo, int position)
{
  position = min(position, POS_MAX);
  position = max(position, 0);
  servo->write(position);
}

int GetArmPosition(int regInput, int id)
{
  int armPosition = map(regInput, mappingTable[id].min, mappingTable[id].max, 10, POS_MAX-10);
  armPosition = min(armPosition, POS_MAX);
  armPosition = max(armPosition, 0);
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

void setup() {
  // put your setup code here, to run once: 
  mySerial.begin(9600);

  // Ley input
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);

  // Relay
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);

  Serial.begin(9600); 
  
  servo0.attach(ARM_SERVO_CHANNEL0);
  servo1.attach(ARM_SERVO_CHANNEL1);
  servo2.attach(ARM_SERVO_CHANNEL2);
  servoHand.attach(ARM_SERVO_CHANNEL_HAND);
  delay(20);

  // Initialize Servo motorS
  servoData0.id = 0;
  servoData1.id = 1;
  servoData2.id = 2;
  servoDataHand.id = 3;
  
  for(int i=0; i<POS_DATA_MAX; i++)
  {
    servoData0.posArm[i] = 90;
    servoData1.posArm[i] = 90;
    servoData2.posArm[i] = 90;
    servoDataHand.posArm[i] = 90;
  }
  
  count = 0;
  savedCount = 0;
  controlState = STATE_STOP;
  requestedState = STATE_STOP;
  Serial.println("RobotArm Started!");
}

void CheckStates()
{
  if((requestedState == STATE_STOP) && (controlState != STATE_STOP))
  {
    controlState = STATE_STOP;
    digitalWrite(7, LOW);
    Serial.println("Stop");
  }
  else if((requestedState == STATE_RECORD) && (controlState != STATE_RECORD))
  {
    controlState = STATE_RECORD;
    count = 0;
    savedCount = 0;
    digitalWrite(7, LOW);
    for(int i=0; i<POS_DATA_MAX; i++)
    {
      servoData0.posArm[i] = 0;
      servoData1.posArm[i] = 0;
      servoData2.posArm[i] = 0;
    }
    Serial.println("Record");
  }
  else if((requestedState == STATE_PLAY) && (controlState != STATE_PLAY))
  {
    controlState = STATE_PLAY;  
    count = 0;
    digitalWrite(7, HIGH);
    Serial.println("Play");
  }
}

void CheckKeys()
{
  if(digitalRead(STOP_KEY_CHANNEL) == 1)
  {
    requestedState = STATE_STOP;
  }
  else if(digitalRead(RECORD_KEY_CHANNEL) == 1)
  {
    requestedState = STATE_RECORD;
  }
  else if(digitalRead(PLAY_KEY_CHANNEL) == 1)
  {
    requestedState = STATE_PLAY;
  }
}

void ProcessBTInput()
{
   if(mySerial.available()) {
    while(mySerial.available()) {
      command += (char)mySerial.read();
      requestedState = command.toInt();
    }
    
    Serial.println(command);
    command = ""; // No repeats
  }
}

int temp;
void loop() {
  // put your main code here, to run repeatedly:
  ProcessBTInput(); 
  CheckKeys();
  CheckStates();
  
  switch(controlState)
  {
    case STATE_STOP:
    break;

    case STATE_RECORD:
    SaveArmPosition(&servoData0, ARM_REG_CHANNEL0, count);
    SaveArmPosition(&servoData1, ARM_REG_CHANNEL1, count);
//    SaveArmPosition(&servoData2, ARM_REG_CHANNEL2, count);
    SaveArmPosition(&servoDataHand, ARM_REG_CHANNEL_HAND, count);
    UpdateCount();
    delay(100);
    break;

    case STATE_PLAY:
    SetServoControl(&servo0, servoData0.posArm[count]); 
    SetServoControl(&servo1, servoData1.posArm[count]); 
//    SetServoControl(&servo2, servoData2.posArm[count]); 
    SetServoControl(&servoHand, servoDataHand.posArm[count]); 
    UpdateCount();
    delay(100);
    break;
    
    default:
    break;
  }
}
