/*
 * Use controller for buwizz device using custom made PCB
 * 
 * 20201111 Giba
 *    v00: first release
 */
#include <ArduinoBLE.h>

// Internal RGB led
#include <Arduino_APDS9960.h>
//https://create.arduino.cc/projecthub/tkhaled/arduino-nano-33-ble-sense-gestures-with-led-feedback-402a79
const int RED_LED = 22;
const int GREEN_LED = 23;
const int BLUE_LED = 24;

// Colors
const byte BLACK_COLOR = 0x00;
const byte WHITE_COLOR = 0x01;
const byte RED_COLOR = 0x02;
const byte GREEN_COLOR = 0x03;
const byte BLUE_COLOR = 0x04;
const byte MAGENTA_COLOR = 0x05;

// Buwizz device
const byte STARTUP = 0x00;
const byte SCANNING = 0X01;
const byte CONNECTING = 0X02;
const byte CONNECTED = 0x03;
const byte RUNNING = 0x04;

volatile byte internalState =  STARTUP;
const char STANDARD_SERVICE[] = "4e050000-74fb-4481-88b3-9919b1676e93";
byte readBuffer[50] = {};

BLEDevice buwizzDevice;
BLEService tempService;
BLECharacteristic tempCharacteristic;
BLEDescriptor tempDescriptor;

const byte LOW_SPEED = 0x01;
const byte NORMAL_SPEED = 0x02;
const byte FAST_SPEED = 0x03;
const byte LUDICROUS_SPEED = 0x04;

byte modeMessage[2] = {0x11, 0x00};
byte speedMessage[6] = {0x10, 0x00, 0x00, 0x00, 0x00, 0x00};
//byte speedMode[4] = {LOW_SPEED, NORMAL_SPEED, FAST_SPEED, LUDICROUS_SPEED};
byte actualMode = NORMAL_SPEED;
byte lastMode = NORMAL_SPEED;

// Button map
#define LB_TRIG_0 5   // Pin used for left trigger (normally HIGH) // External 
#define LB_TRIG_1 4   // Pin used for left trigger (normally HIGH) // Internal
#define RB_TRIG_0 7   // Pin used for right trigger (normally HIGH) // External
#define RB_TRIG_1 6   // Pin used for right trigger (normally HIGH) // Internal
#define LJ_TRIG 19   // Pin used for left joystick (normally HIGH)   
#define RJ_TRIG 17   // Pin used for right joystick (normally HIGH)
#define LV_JOY A7   // Pin used for left joystick
#define LH_JOY A6   // Pin used for left joystick
#define RV_JOY A4   // Pin used for right joystick
#define RH_JOY A2   // Pin used for right joystick


#define NODE_SELECT_0 15 // Jumper 1
#define NODE_SELECT_1 14 // Jumper 2
#define IR_LED_PIN 8  // IR LED OUTPUT
#define GENERIC_INPUT_0 2 // AVAILABLE
#define GENERIC_INPUT_1 3 // AVAILABLE

#define PWM_BRK 0
#define PWM_REV7 -100
#define PWM_REV6 -88
#define PWM_REV5 -76
#define PWM_REV4 -63
#define PWM_REV3 -50
#define PWM_REV2 -38
#define PWM_REV1 -25
#define PWM_FLT 0
#define PWM_FWD1 25
#define PWM_FWD2 38
#define PWM_FWD3 50
#define PWM_FWD4 63
#define PWM_FWD5 76
#define PWM_FWD6 88
#define PWM_FWD7 100

// Joystick map, default 8 bit reso
const double LOG_MAP_CONST = 188.980798; // 512/log10(512)
const double COEFF = 200.0;

const int P7 =  514; // 534
const int P6 =  499; // 519
const int P5 =  480; // 500
const int P4 =  456; // 476
const int P3 =  421; // 441
const int P2 =  364; // 384
const int P1 =  120; // 140
const int M1 = -1 * P1;
const int M2 = -1 * P2;
const int M3 = -1 * P3;
const int M4 = -1 * P4;
const int M5 = -1 * P5;
const int M6 = -1 * P6;
const int M7 = -1 * P7;

// Joystick variables
int leftVerZero = 0;
int rightVerZero = 0;
int leftHorZero = 0;
int rightHorZero = 0;

int leftVerStick = 0;
int rightVerStick = 0;
int leftHorStick = 0;
int rightHorStick = 0;

int leftValue = 0;
int rightValue = 0;
int leftPositive = 0;
int rightPositive = 0;

byte motor1speed = 0;
byte motor2speed = 0;
byte motor3speed = 0;
byte motor4speed = 0;

// *** SETUP ***
void setup()
{ 
  // Start serial
  Serial.begin(115200);

  
  //while (!Serial); // Lock until serial is not initialized
  
  // Init BLE hardware
  if (!BLE.begin())
  {
    Serial.println("Starting BLE failed!");
    SetLEDColor(RED_COLOR);
    while (true); // Lock execution
  }

  if (!APDS.begin())
  {
    Serial.println("Error initializing APDS9960 sensor!");
    SetLEDColor(RED_COLOR);
    while (true); // Lock execution
  }

  // Set RGB PIN
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  SetLEDColor(BLACK_COLOR);

  // Set IO config
  pinMode(LB_TRIG_0,INPUT_PULLUP); // Enable pullup resistor
  pinMode(LB_TRIG_1,INPUT_PULLUP); // Enable pullup resistor
  pinMode(RB_TRIG_0,INPUT_PULLUP); // Enable pullup resistor
  pinMode(RB_TRIG_1,INPUT_PULLUP); // Enable pullup resistor
  pinMode(LJ_TRIG,INPUT_PULLUP); // Enable pullup resistor
  pinMode(RJ_TRIG,INPUT_PULLUP); // Enable pullup resistor
  pinMode(NODE_SELECT_0,INPUT_PULLUP); // Enable pullup resistor
  pinMode(NODE_SELECT_1,INPUT_PULLUP); // Enable pullup resistor
  pinMode(GENERIC_INPUT_0,INPUT_PULLUP); // Enable pullup resistor
  pinMode(GENERIC_INPUT_1,INPUT_PULLUP); // Enable pullup resistor
  pinMode(IR_LED_PIN, OUTPUT);

  // Read axis zero values
  leftVerZero = analogRead(LV_JOY);
  rightVerZero = analogRead(RV_JOY);
  leftHorZero = analogRead(LH_JOY);
  rightHorZero = analogRead(RH_JOY);
  
  internalState = STARTUP;
  BLE.setConnectionInterval(10, 100); // Interval min max of 1.25 ms

  SetLEDColor(RED_COLOR);
  delay(500);
  SetLEDColor(GREEN_COLOR);
  delay(500);
  SetLEDColor(BLUE_COLOR);
  delay(500);
  SetLEDColor(BLACK_COLOR);
 
  Serial.println("Setup() completed");
}

// *** LOOP *** 
void loop()
{
  
  
  switch(internalState)
  {
    case STARTUP:
      Serial.println("STARTUP");
      Serial.println("Start scanning for BuWizz...");
      BLE.scanForUuid(STANDARD_SERVICE);
      internalState = SCANNING;
      Serial.println("Move to SCANNING");
      SetLEDColor(MAGENTA_COLOR);
      break;

    case SCANNING:
      //Serial.println("SCANNING");
      buwizzDevice = BLE.available();

      if(buwizzDevice)
      {
        Serial.print("Found ");
        Serial.print(buwizzDevice.address());
        Serial.print(" '");
        Serial.print(buwizzDevice.localName());
        Serial.print("' ");
        Serial.print(buwizzDevice.advertisedServiceUuid());
        Serial.println();
        BLE.stopScan();
        internalState = CONNECTING;
        Serial.println("Move to CONNECTING");
      }
      break;

    case CONNECTING:
      // Serial.println("CONNECTING");

      if(buwizzDevice.connect())
      {
        Serial.println("Buwizz connected");

        // Discover peripheral attributes
        Serial.println("Discovering attributes ...");
        
        if (buwizzDevice.discoverAttributes())
        {
          Serial.println("Attributes discovered");
        }
        else
        {
          Serial.println("Attribute discovery failed!");
          Serial.println("Restart everything");
          SetLEDColor(RED_COLOR);
          while (true); // Lock
        }
        
        internalState = CONNECTED;
        Serial.println("Move to CONNECTED");
      }
      else
      {
        Serial.println("Connecting to BuWizz failed!");
        Serial.println("Restart everything");
        while (true); // Lock
        internalState = STARTUP;
      }
      
      break;

    case CONNECTED:
      // Serial.println("CONNECTED");

      if(buwizzDevice.connected())
      {
        Serial.println("Start BuWizz debug");
    
        int serviceCount = buwizzDevice.serviceCount();
        int characteristicCount = 0;
        int descriptorCount = 0;
        int byteCount = 0;
        Serial.print(serviceCount);
        Serial.println(" services discovered:");

        for(int ii = 0; ii < serviceCount; ii++)
        {
          tempService = buwizzDevice.service(ii);
          Serial.print("Index ");
          Serial.print(ii);
          Serial.print(" UUID: ");
          Serial.println(tempService.uuid());

          characteristicCount = tempService.characteristicCount();
          Serial.print("   ");
          Serial.print(characteristicCount);
          Serial.println(" characteristics discovered:");

          for(int jj = 0; jj < characteristicCount; jj++)
          {
            tempCharacteristic = tempService.characteristic(jj);
            Serial.print("   ");
            Serial.print("Index ");
            Serial.print(ii);
            Serial.print(" UUID: ");
            Serial.println(tempCharacteristic.uuid());

            descriptorCount = tempCharacteristic.descriptorCount();
            for(int ll = 0; ll < descriptorCount; ll++)
            {
              tempDescriptor = tempCharacteristic.descriptor(ll);
              Serial.print("      descriptor ");
              Serial.print(ll);
              Serial.print(" : UUID ");
              Serial.print(tempDescriptor.uuid());
              Serial.print(" - ");
              byteCount = tempDescriptor.readValue(readBuffer, 50);
              for(int kk = 0; kk < byteCount; kk++)               
                Serial.print((char)readBuffer[kk]);
                
              Serial.println();
            }

            if(tempCharacteristic.canRead())
            {
              Serial.println("      can be read. Try to read:");
              byteCount = tempCharacteristic.readValue(readBuffer, 50);
              Serial.print("         STR: ");
              for(int kk = 0; kk < byteCount; kk++) // Print as string           
                Serial.print((char)readBuffer[kk]);                
              Serial.println();

              Serial.print("         HEX: ");
              for(int kk = 0; kk < byteCount; kk++) // Print as string
              {          
                Serial.print(readBuffer[kk], HEX);
                Serial.print(" ");
              }              
              Serial.println();
            }
            
            if(tempCharacteristic.canWrite())
              Serial.println("      can be written");

            if(tempCharacteristic.canSubscribe())
              Serial.println("      can be subscrived");
          }
        }

        actualMode = NORMAL_SPEED;
        lastMode = NORMAL_SPEED;
        SetBuWizzMode(actualMode);
        SetLEDColor(GREEN_COLOR);
        SetBuWizzSpeed(0, 0, 0, 0);
        
        Serial.println("Finish!");
        internalState = RUNNING;
      }
      else
      {
        Serial.println("Buwizz disconnected for some reason");
        internalState = STARTUP;
      }
      break;

    case RUNNING:
      //TestIO();
    
      if(buwizzDevice.connected())
      {
        if (APDS.gestureAvailable()) // a gesture was detected, read and print to serial monitor
        {        
          int gesture = APDS.readGesture();

          switch (gesture)
          {
            case GESTURE_UP:
              Serial.println("Detected UP gesture");         
              if(actualMode < LUDICROUS_SPEED)
                actualMode++;
              break;

            case GESTURE_DOWN:
              Serial.println("Detected DOWN gesture");
              if(actualMode > LOW_SPEED)
                actualMode--;
              break;

            default:
              // Do nothing...
              break;
          }

          delay(100);
        }
       
        if(actualMode != lastMode)
        {
          switch(actualMode)
          {
            case LOW_SPEED:
              SetBuWizzMode(LOW_SPEED);
              SetLEDColor(BLUE_COLOR);
              break;

            case NORMAL_SPEED:
              SetBuWizzMode(NORMAL_SPEED);
              SetLEDColor(GREEN_COLOR);
              break;

            case FAST_SPEED:
              SetBuWizzMode(FAST_SPEED);
              SetLEDColor(RED_COLOR);
              break;

            case LUDICROUS_SPEED:
              SetBuWizzMode(LUDICROUS_SPEED);
              SetLEDColor(WHITE_COLOR);
              break;
          }

          delay(50);
        }

        lastMode = actualMode;
        
        ReadInput();
        SetBuWizzSpeed(motor1speed, motor2speed, motor3speed, motor4speed);
        
        delay(100); // TODO
      }
      else
      {
        Serial.println("Buwizz disconnected for some reason");
        internalState = STARTUP;
      }
      break;
      
    default:
      Serial.println("default ???");
      break;
  }
}

// *** FUNCTIONS ***
void SetLEDColor(byte _color)
{
  // Pin are inverted, HIGH = OFF, LOW = ON
  switch(_color)
  {
    case BLACK_COLOR:
      digitalWrite(RED_LED, HIGH); 
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(BLUE_LED, HIGH); 
      break;

    case WHITE_COLOR:
      digitalWrite(RED_LED, LOW); 
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(BLUE_LED, LOW); 
      break;

    case RED_COLOR:
      digitalWrite(RED_LED, LOW); 
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(BLUE_LED, HIGH); 
      break;

    case GREEN_COLOR:
      digitalWrite(RED_LED, HIGH); 
      digitalWrite(GREEN_LED, LOW);
      digitalWrite(BLUE_LED, HIGH); 
      break;

    case BLUE_COLOR:
      digitalWrite(RED_LED, HIGH); 
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(BLUE_LED, LOW); 
      break;

    case MAGENTA_COLOR:
      digitalWrite(RED_LED, LOW); 
      digitalWrite(GREEN_LED, HIGH);
      digitalWrite(BLUE_LED, LOW); 
      break;
  }
}

bool SetBuWizzMode(byte _mode)
{
  tempCharacteristic = buwizzDevice.service(0).characteristic(0);
  modeMessage[1] = _mode;
  return(tempCharacteristic.writeValue(modeMessage, 2));
}

bool SetBuWizzSpeed(byte _speed1, byte _speed2, byte _speed3, byte _speed4) // speed -100 ... 100
{
  tempCharacteristic = buwizzDevice.service(0).characteristic(0);
  speedMessage[1] = _speed1;
  speedMessage[2] = _speed2;
  speedMessage[3] = _speed3;
  speedMessage[4] = _speed4;
  return(tempCharacteristic.writeValue(speedMessage, 6));
}

void ReadInput()
{
  // Read joystick button to set positive direction
  // Left
  if(digitalRead(LJ_TRIG) == false)
  {
    delay(500);

    if(leftPositive != 3)
      leftPositive += 1;
    else
      leftPositive = 0;

    // Catch the point
    while(digitalRead(LJ_TRIG) == false)
      delay(200);
  }

  // Right
  if(digitalRead(RJ_TRIG) == false)
  {
    delay(500);

    if(rightPositive != 3)
      rightPositive += 1;
    else
      rightPositive = 0;

    // Catch the point
    while(digitalRead(RJ_TRIG) == false)
      delay(200);
  }

  // Read joysticks and calculate value and map
  // Left
  switch(leftPositive)
  {
    // Up
    case 0:
      leftVerStick = analogRead(LV_JOY);
      leftValue = (leftVerStick - leftVerZero);
      break;

    // Right
    case 1:
      leftHorStick = analogRead(LH_JOY);
      leftValue = -(leftHorStick - leftHorZero);
      break;

    // Down
    case 2:
      leftVerStick = analogRead(LV_JOY);
      leftValue = -(leftVerStick - leftVerZero);
      break;

    // Left
    case 3:
      leftHorStick = analogRead(LH_JOY);
      leftValue = (leftHorStick - leftHorZero);
      break;
  }

  // Right
  switch(rightPositive)
  {
    // Up
    case 0:
      rightVerStick =  analogRead(RV_JOY);
      rightValue = (rightVerStick - rightVerZero);
      break;

    // Right
    case 1:
      rightHorStick =  analogRead(RH_JOY);
      rightValue = -(rightHorStick - rightHorZero);
      break;

    // Down
    case 2:
      rightVerStick =  analogRead(RV_JOY);
      rightValue = -(rightVerStick - rightVerZero);
      break;

    // Left
    case 3:
      rightHorStick =  analogRead(RH_JOY);
      rightValue = (rightHorStick - rightHorZero);
      break;
  }
  
  // Map here logaritmic 
  // Left and then right value
  if(leftValue > 0)
  {
    leftValue = ((int) (log10((double)leftValue) * COEFF));// LOG_MAP_CONST));
  }
  else  if(leftValue < 0)
  {
    leftValue =-1 * ((int) (log10((double)abs(leftValue)) * COEFF));// LOG_MAP_CONST));
  }
  else // leftValue == 0
  {
     leftValue = 0;
  }

  if(rightValue > 0)
  {
    rightValue = ((int) (log10((double)rightValue) * COEFF));// LOG_MAP_CONST));
  }
  else  if(rightValue < 0)
  {
    rightValue = -1 * ((int) (log10((double)abs(rightValue)) * COEFF));// LOG_MAP_CONST));
  }
  else // rightValue == 0
  {
     rightValue = 0;
  }
   
  // Map and set value to send to output
  // Left, send brake if left button is pressed
  if(digitalRead(LB_TRIG_0) == false)
  {
    motor1speed = PWM_BRK; // TODO
  }
  else
  {
    //motor1speed = (byte)(map((long)leftValue, -512, 512, -100, 100));

    if(leftValue < M7)
      motor1speed = PWM_REV7;
    else if(leftValue >= M7 && leftValue < M6)
      motor1speed = PWM_REV6;
    else if(leftValue >= M6 && leftValue < M5)
      motor1speed = PWM_REV5;
    else if(leftValue >= M5 && leftValue < M4)
      motor1speed = PWM_REV4;
    else if(leftValue >= M4 && leftValue < M3)
      motor1speed = PWM_REV3;
    else if(leftValue >= M3 && leftValue < M2)
      motor1speed = PWM_REV2;
    else if(leftValue >= M2 && leftValue < M1)
      motor1speed = PWM_REV1;
    else if(leftValue >= M1 && leftValue < P1)
      motor1speed = PWM_FLT;
    else if(leftValue >= P1 && leftValue < P2)
      motor1speed = PWM_FWD1;
    else if(leftValue >= P2 && leftValue < P3)
      motor1speed = PWM_FWD2;
    else if(leftValue >= P3 && leftValue < P4)
      motor1speed = PWM_FWD3;
    else if(leftValue >= P4 && leftValue < P5)
      motor1speed = PWM_FWD4;
    else if(leftValue >= P5 && leftValue < P6)
      motor1speed = PWM_FWD5;
    else if(leftValue >= P6 && leftValue < P7)
      motor1speed = PWM_FWD6;
    else
      motor1speed = PWM_FWD7; 
  }

  // Rigt, send brake if right button is pressed
  if(digitalRead(RB_TRIG_0) == false)
  {
    motor2speed = PWM_BRK;
  }
  else
  {
    //motor2speed = (byte)(map((long)rightValue, -512, 512, -100, 100));

    if(rightValue < M7)
      motor2speed = PWM_REV7;
    else if(rightValue >= M7 && rightValue < M6)
      motor2speed = PWM_REV6;
    else if(rightValue >= M6 && rightValue < M5)
      motor2speed = PWM_REV5;
    else if(rightValue >= M5 && rightValue < M4)
      motor2speed = PWM_REV4;
    else if(rightValue >= M4 && rightValue < M3)
      motor2speed = PWM_REV3;
    else if(rightValue >= M3 && rightValue < M2)
      motor2speed = PWM_REV2;
    else if(rightValue >= M2 && rightValue < M1)
      motor2speed = PWM_REV1;
    else if(rightValue >= M1 && rightValue < P1)
      motor2speed = PWM_FLT;
    else if(rightValue >= P1 && rightValue < P2)
      motor2speed = PWM_FWD1;
    else if(rightValue >= P2 && rightValue < P3)
      motor2speed = PWM_FWD2;
    else if(rightValue >= P3 && rightValue < P4)
      motor2speed = PWM_FWD3;
    else if(rightValue >= P4 && rightValue < P5)
      motor2speed = PWM_FWD4;
    else if(rightValue >= P5 && rightValue < P6)
      motor2speed = PWM_FWD5;
    else if(rightValue >= P6 && rightValue < P7)
      motor2speed = PWM_FWD6;
    else
      motor2speed = PWM_FWD7;
  }
}

void TestIO()
{
  Serial.print("T_L ");
  Serial.print(digitalRead(LJ_TRIG));
  Serial.print(" ; ALV ");
  Serial.print(analogRead(LV_JOY)); 
  Serial.print(" ; ALH ");
  Serial.print(analogRead(LH_JOY)); 

  Serial.print(" ; T_R ");
  Serial.print(digitalRead(RJ_TRIG));
  Serial.print(" ; ARV ");
  Serial.print(analogRead(RV_JOY)); 
  Serial.print(" ; ARH ");
  Serial.print(analogRead(RH_JOY));
  Serial.println(); 
}

/*
void Test()
{
  int selectMode = 0;
  Serial.println("Entering test mode, restart everything to stop test...");
  
  tempCharacteristic = buwizzDevice.service(0).characteristic(0);
  
  while (true) // Lock
  {
    // Set speed 0
    speedMessage[1] = 0x0;
    if(tempCharacteristic.writeValue(speedMessage, 6))
      Serial.println("Set speed 0 on channel 1 OK");
    else
      Serial.println("Set speed 0 on channel 1 failed!");
      
    // Set mode
    modeMessage[1] = speedMode[selectMode++];
  
    if(selectMode == 4)
      selectMode = 0;
    
    if(tempCharacteristic.writeValue(modeMessage, 2))
      Serial.println("Set mode OK");
    else
      Serial.println("Set mode failed!");
  
    delay(1000);
    
    // Set speed 100
    speedMessage[1] = 0x64;
    if(tempCharacteristic.writeValue(speedMessage, 6))
      Serial.println("Set speed 100 on channel 1 OK");
    else
      Serial.println("Set speed 100 on channel 1 failed!");
  
    delay(1000);
  
    // Set speed -100
    speedMessage[1] = 0x9C;
    if(tempCharacteristic.writeValue(speedMessage, 6))
      Serial.println("Set speed -100 on channel 1 OK");
    else
      Serial.println("Set speed -100 on channel 1 failed!");
  
    delay(1000);
  }
}
*/
