#include <SPI.h>
#include <Wire.h>
#include <MFRC522.h>
#include <LiquidCrystal_I2C.h>

//inlude the bounce library 
#include <Bounce.h>

#define RST_PIN 9
#define SS_PIN  10

#define STATE_STARTUP       0
#define STATE_STARTING      1
#define STATE_WAITING       2
#define STATE_SCAN_INVALID  3
#define STATE_SCAN_VALID    4
#define STATE_SCAN_MASTER   5
#define STATE_ADDED_CARD    6
#define STATE_REMOVED_CARD  7

#define PUSHBUTTON 4
/*********************************
*****EDITED by ALAKAZAM / 2016****
****TAMBAHAN DARI ZAM 22/4/2016***
**********************************/
#define KEY_TAB       0xB3
#define KEY_ENTER     0xB0

// Creating Bounce objects for each button makes detecting changes very easy.
Bounce button3 = Bounce(PUSHBUTTON, 10); 

const int cardArrSize = 10;
const int cardSize    = 4;
byte cardArr[cardArrSize][cardSize];
byte masterCard[cardSize] = {92,86,65,197};
byte readCard[cardSize];
byte cardsStored = 0;

// Create MFRC522 instance
MFRC522 mfrc522(SS_PIN, RST_PIN);
// Set the LCD I2C address
LiquidCrystal_I2C lcd(0x27 , 16, 2);

byte currentState = STATE_STARTUP;
unsigned long LastStateChangeTime;
unsigned long StateWaitTime;

//------------------------------------------------------------------------------------
int readCardState()
{
  int index;

  Serial.print("Card Data - ");
  for(index = 0; index < 4; index++)
  {
    readCard[index] = mfrc522.uid.uidByte[index];

    
    Serial.print(readCard[index]);
    if (index < 3)
    {
      Serial.print(",");
    }
  }
  Serial.println(" ");

  //Check Master Card
  if ((memcmp(readCard, masterCard, 4)) == 0)
  {
    return STATE_SCAN_MASTER;
  }

 return STATE_SCAN_INVALID;
}

//------------------------------------------------------------------------------------
void addReadCard()
{
  int cardIndex;
  int index;

  if (cardsStored <= 20)
  {
    cardsStored++;
    cardIndex = cardsStored;
    cardIndex--;
  }

  for(index = 0; index < 4; index++)
  {
    cardArr[cardIndex][index] = readCard[index];
  }
}

//------------------------------------------------------------------------------------
void updateState(byte aState)
{
  if (aState == currentState)
  {
    return;
  }

  // do state change
  switch (aState)
  {
    case STATE_STARTING:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("RFID Scanner");
      lcd.setCursor(0,1);
      lcd.print("Starting up");
      StateWaitTime = 1000;
      pinMode(PUSHBUTTON, OUTPUT);
      digitalWrite(PUSHBUTTON, HIGH);
      break;
      
    case STATE_WAITING:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Waiting for Card");
      lcd.setCursor(0,1);
      lcd.print("to be swiped");
      StateWaitTime = 0;
      pinMode(PUSHBUTTON, OUTPUT);
      digitalWrite(PUSHBUTTON, HIGH);
      break;

    case STATE_SCAN_INVALID:
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Maaf,..?");
        lcd.setCursor(0,1);
        lcd.print("Invalid Card!");
        StateWaitTime = 2000;
      pinMode(PUSHBUTTON, OUTPUT);
      digitalWrite(PUSHBUTTON, HIGH);
      break;
      
    case STATE_SCAN_MASTER:
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Valid Card");
      lcd.setCursor(0,1);
      lcd.print("Success!");
      break;
  }

  /*lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(aState);
  lcd.setCursor(0,1);
  lcd.print(currentState);*/

  currentState = aState;
  LastStateChangeTime = millis();
}

void setup() 
{
  SPI.begin();         // Init SPI Bus
  mfrc522.PCD_Init();  // Init MFRC522

  lcd.begin();

  LastStateChangeTime = millis();
  updateState(STATE_STARTING);

  pinMode(PUSHBUTTON, INPUT);
  pinMode(13, OUTPUT);
  digitalWrite(PUSHBUTTON, INPUT_PULLUP);
  digitalWrite(13, HIGH);

  Keyboard.begin();

  Serial.begin(9600);
}

void loop() 
{
  byte cardState;
  
  button3.update();

  if (currentState == STATE_SCAN_MASTER)
  {
    pinMode(PUSHBUTTON, INPUT);
    digitalWrite(PUSHBUTTON, INPUT_PULLUP);
 
   if (button3.fallingEdge())
     {
        Keyboard.print("bismillah");
        Keyboard.press(KEY_ENTER);
        Keyboard.print("tes..'");
    
        updateState(STATE_SCAN_INVALID);  
    }
      
      // release all the keys
      Keyboard.releaseAll();
  }

  if ((currentState != STATE_WAITING) &&
      (StateWaitTime > 0) &&
      (LastStateChangeTime + StateWaitTime < millis()))
  {
    updateState(STATE_WAITING);
  }

  // Look for new cards 
  if ( ! mfrc522.PICC_IsNewCardPresent()) 
  { 
    return; 
  } 
  
  // Select one of the cards 
  if ( ! mfrc522.PICC_ReadCardSerial()) 
  { 
    return; 
  }

  cardState = readCardState();
  updateState(cardState);
}
