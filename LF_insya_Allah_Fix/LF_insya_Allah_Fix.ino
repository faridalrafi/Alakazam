#include <LiquidCrystal.h>
#include <EEPROM.h>
//define port LCD
LiquidCrystal lcd(12,11,8,7,6,5);
#define BackLight 2

//define pin Push Button
#define IncButton 0
#define DecButton 9
#define SelectButton 8
#define Interupt 1

//DEFINE BEBAN SENSOR
#define WS8     0          // -WS7
#define WS9     5          // -WS6
#define WS10    10          // -WS5
#define WS11    13          // -WS4
#define WS12    15         // -WS3
#define WS13    17         // -WS2
#define WS14    25         // -WS1
#define WS15    100         // -WS0

//DEFINE MOTOR
#define DIR_R  13
#define DIR_L  12
#define PWM_L  11
#define PWM_R  10
#define MAJU    0
#define MUNDUR  1

//DEFINE PORT SENSOR
#define A0_1 14
#define A1_1 15
#define A2_1 16
#define A0_2 17
#define A1_2 18
#define A2_2 19
#define Y1 A6
#define Y2 A7
//#define THRESHOLD 998

//int r0_1,r1_1,r2_1,r0_2,r1_2,r2_2=0;
int nilaiDigital[16];
//int bebanSensor[16]={WS0, WS1, WS2, WS3, WS4, WS5, WS6, WS7, WS8, WS9, WS10, WS11, WS12, WS13, WS14, WS15};
int bebanSensor[8]={WS8, WS9, WS10, WS11, WS12, WS13, WS14, WS15};
int SensorID, SensorAktif;
int TotalNilaiDigital, SensorKanan, SensorKiri, PV;
int SampleTime=100, lastTime;
float  Kp=5, Ki=6, Kd=1, ITerm, APWM;
int outMax=255, outMin=0, lastPV;
byte Dir_R, Dir_L; 
int Case=0, nextCase, lastCase=0, totalCase=80;
int PWMlvl_R, PWMlvl_L;
//byte dir_R, dir_L;
unsigned char byteSensorL, byteSensorR;
int t=0;

//********EEPROM**************
int eNormPWM=1;
int eThreshold0=2;
int eThreshold1=3;
int eThreshold2=4;
int eThreshold3=5;
int eThreshold4=6;
int eThreshold5=7;
int eThreshold6=8;
int eThreshold7=9;
int eThreshold8=10;
int eThreshold9=11;
int eThreshold10=12;
int eThreshold11=13;
int eThreshold12=14;
int eThreshold13=15;
int eThreshold14=16;
int eThreshold15=17;

//init Mission
char Start;
unsigned char nextState=0;
unsigned char allState=82;
unsigned char CState=0;
unsigned char VState[82];

//Checkpoint
byte CP,Run=0;

//Init LCD
int MenuID,Pilih=0;
int NormPWM,tempPWM;

//********EEPROM**************
//Penambahan debounce untuk pushButton
int lastButtonState=HIGH;
//long lastDebounceTime = 0;
//long debounceDelay = 50;
int buttonState;//=HIGH;

int digitalReads(int reading)
{
  lastButtonState=HIGH;
  buttonState=HIGH;
  if (digitalRead(reading) == LOW)
  {
    lastButtonState=digitalRead(reading);
    while(digitalRead(reading) == lastButtonState)
    {
    buttonState=lastButtonState;
    }
    //lastDebounceTime = millis();
  }
  return buttonState;
}
/************************************
*********Varabel Kalibrasi***********
************************************/
int Rata2L[8], Rata2R[8];
int ThresholdR[8], ThresholdL[8];
int kalibrasi_HitamR[8], kalibrasi_HitamL[8], kalibrasi_PutihR[8], kalibrasi_PutihL[8];
int nilaiDigitalL[8], nilaiDigitalR[8];
int nilaiAnalogL[8], nilaiAnalogR[8];
/********************************
************Kalibrasi************
*********************************/
int kalibrasiKanan(int po)
{
  int nilai_kanan[8]={0,0,0,0,0,0,0,0}, analog_Kanan[8]={0,0,0,0,0,0,0,0};
  int k, BorW[8];
  float Rata[8]={0,0,0,0,0,0,0,0};
  for(k=0;k<16;k++)
  {   
      analog_Kanan[po]=analogRead(Y2)-900;
      nilai_kanan[po]+=analog_Kanan[po];
      BorW[po]=0;
  }

    Rata[po]=nilai_kanan[po]/16;
    BorW[po]=(Rata[po]);

  return BorW[po];
}
int kalibrasiKiri(int po)
{
  int nilai_kiri[8]={0,0,0,0,0,0,0,0}, analog_Kiri[8]={0,0,0,0,0,0,0,0};
  int k, BorW[8];
  float Rata[8]={0,0,0,0,0,0,0,0};
  for(k=0;k<16;k++)
  {  
      analog_Kiri[po]=analogRead(Y1)-900;
      nilai_kiri[po]+=analog_Kiri[po];
      BorW[po]=0;
  }

    Rata[po]=nilai_kiri[po]/16;
    BorW[po]=(Rata[po]);

  return BorW[po];
}

void cariThreshold(char T)
{    
    int i;
    
  if(T==0)
  {
    for(SensorID=0;SensorID<8;SensorID++)
    {
      digitalWrite(A0_1,bitRead(SensorID,0));
      digitalWrite(A1_1,bitRead(SensorID,1));
      digitalWrite(A2_1,bitRead(SensorID,2));
      digitalWrite(A0_2,bitRead(SensorID,0));
      digitalWrite(A1_2,bitRead(SensorID,1));
      digitalWrite(A2_2,bitRead(SensorID,2));

        kalibrasi_HitamL[SensorID] = kalibrasiKiri(SensorID);
        kalibrasi_HitamR[SensorID] = kalibrasiKanan(SensorID);
      
        for(i=0; i<8; i++)
      {
          Serial.print(kalibrasi_HitamL[i]);
          Serial.print(", ");  
      }
        for(i=0; i<8; i++)
      {
          Serial.print(kalibrasi_HitamR[i]);
          Serial.print(", ");  
      }
          Serial.println();
    }
  }
  else if(T==1)
  {
    for(SensorID=0;SensorID<8;SensorID++)
    {
      digitalWrite(A0_1,bitRead(SensorID,0));
      digitalWrite(A1_1,bitRead(SensorID,1));
      digitalWrite(A2_1,bitRead(SensorID,2));
      digitalWrite(A0_2,bitRead(SensorID,0));
      digitalWrite(A1_2,bitRead(SensorID,1));
      digitalWrite(A2_2,bitRead(SensorID,2));
      
      kalibrasi_PutihL[SensorID] = kalibrasiKiri(SensorID);
      kalibrasi_PutihR[SensorID] = kalibrasiKanan(SensorID);
        
        for(i=0; i<8; i++)
      {
          Serial.print(kalibrasi_PutihL[i]);
          Serial.print(", ");  
      }
        for(i=0; i<8; i++)
      {
          Serial.print(kalibrasi_PutihR[i]);
          Serial.print(", ");  
      }
          //Serial.print(PWMlvl_R);
          //Serial.print(", ");
          //Serial.println(PWMlvl_L);
          Serial.println();
      
    }
  }

  //int t;
  for(SensorID=0;SensorID<8;SensorID++)
  {
    Rata2L[SensorID]=(kalibrasi_HitamL[SensorID]+kalibrasi_PutihL[SensorID])/2;
    ThresholdL[SensorID]=Rata2L[SensorID];
    Rata2R[SensorID]=(kalibrasi_HitamR[SensorID]+kalibrasi_PutihR[SensorID])/2;
    ThresholdR[SensorID]=Rata2R[SensorID];
  }

}

/*********************************************************
************************Start Up**************************
*********************************************************/

void Menu()
{
  /********************
  *****MENU SENSOR*****
  ********************/
  while(MenuID==0)
  {
    lcd.setCursor(0,0);
    lcd.clear();
    //Serial.print("Kalibrasi sensor");
    lcd.print("Kalibrasi Sensor");
    lcd.setCursor(0,1);
    /****Select****/
    if(digitalReads(IncButton)==LOW && digitalRead(DecButton)==HIGH && digitalRead(SelectButton)==HIGH && digitalRead(Interupt)==HIGH)
    {
      Pilih++;
      if(Pilih>1)
      {
        Pilih=1;
      }
    }
    if(digitalReads(DecButton)==LOW && digitalRead(IncButton)==HIGH && digitalRead(SelectButton)==HIGH && digitalRead(Interupt)==HIGH)
    {
      Pilih--;
      if(Pilih<1)
      {
        Pilih=0;
      }
    }
    /****END Select****/
    if(Pilih==0)
    {
      //Serial.print("Tidak");
      lcd.print("Tidak");
      delay(50);
    
      if(digitalReads(SelectButton)==LOW && digitalRead(IncButton)==HIGH && digitalRead(DecButton)==HIGH && digitalRead(Interupt)==HIGH)
      {
        //******Baca nilai threshold di EEPROM*****
        ThresholdL[0]=EEPROM.read(eThreshold0);
        ThresholdL[1]=EEPROM.read(eThreshold1);
        ThresholdL[2]=EEPROM.read(eThreshold2);
        ThresholdL[3]=EEPROM.read(eThreshold3);
        ThresholdL[4]=EEPROM.read(eThreshold4);
        ThresholdL[5]=EEPROM.read(eThreshold5);
        ThresholdL[6]=EEPROM.read(eThreshold6);
        ThresholdL[7]=EEPROM.read(eThreshold7);
        ThresholdR[0]=EEPROM.read(eThreshold8);
        ThresholdR[1]=EEPROM.read(eThreshold9);
        ThresholdR[2]=EEPROM.read(eThreshold10);
        ThresholdR[3]=EEPROM.read(eThreshold11);
        ThresholdR[4]=EEPROM.read(eThreshold12);
        ThresholdR[5]=EEPROM.read(eThreshold13);
        ThresholdR[6]=EEPROM.read(eThreshold14);
        ThresholdR[7]=EEPROM.read(eThreshold15);
       
        for(int p=0;p<8;p++)
        {
          Serial.print(ThresholdL[p]);
          Serial.print(", ");  
        }
          for(int p=0;p<8;p++)
        {
          Serial.print(ThresholdR[p]);
          Serial.print(", ");  
        }
          Serial.println();
       
        MenuID++;
        break;
      }
    }
  
    else if(Pilih==1)
    {
      //Serial.print("Ya");
      lcd.print("Ya");
      delay(50);
   
      if(digitalReads(SelectButton)==LOW && digitalRead(IncButton)==HIGH && digitalRead(DecButton)==HIGH && digitalRead(Interupt)==HIGH)
      {
        /**************************
        *******KALIBRASI!!*********
        **************************/
        char T=0;
        while(digitalRead(SelectButton)==HIGH && digitalRead(IncButton)==HIGH && digitalRead(DecButton)==HIGH && digitalRead(Interupt)==HIGH)
        {
        
          lcd.setCursor(0,0);
          lcd.clear();
          //Serial.print("Kalibrasi Hitam");
          lcd.print("Kalibrasi Hitam");
          lcd.setCursor(0,1);
         // Serial.print("Mulai!");
          lcd.print("Mulai!");
          delay(50);
            if (digitalReads(SelectButton)==LOW  && digitalRead(IncButton)==HIGH && digitalRead(DecButton)==HIGH && digitalRead(Interupt)==HIGH)
          {
            cariThreshold(0);
            T++;
             //loop();
            delay(750);

            break;
          }
       }
        /*cariThreshold(T);
        T++;
        delay(100);
        */
      
        lcd.setCursor(0,0);
        lcd.clear();
       // Serial.print("Kalibrasi Hitam");
        lcd.print("Kalibrasi Hitam");
        lcd.setCursor(0,1);
        //Serial.print("Done!");
        lcd.print("Selesai!");
        delay(500);
      
        T=1;
        while(digitalRead(SelectButton)==HIGH && digitalRead(IncButton)==HIGH && digitalRead(DecButton)==HIGH && digitalRead(Interupt)==HIGH)
        {
        
          lcd.setCursor(0,0);
          lcd.clear();
          //Serial.print("Kalibrasi Putih");
          lcd.print("Kalibrasi Putih");
          lcd.setCursor(0,1);
          //Serial.print("Mulai!");
          lcd.print("Mulai!");
          delay(50);
         // break;
         if (digitalReads(SelectButton)==LOW  && digitalRead(IncButton)==HIGH && digitalRead(DecButton)==HIGH && digitalRead(Interupt)==HIGH)
          {
             cariThreshold(1);
             //loop();
             delay(750);
             break;
          }
        }
      
        lcd.setCursor(0,0);
        lcd.clear();
       // Serial.print("Kalibrasi Putih");
        lcd.print("Kalibrasi Putih");
        lcd.setCursor(0,1);
       // Serial.print("Done!");
        lcd.print("Selesai!");
        delay(500);
       
        //*******Save Threshold ke EEPROM******
        EEPROM.write(eThreshold0,ThresholdL[0]);
        EEPROM.write(eThreshold1,ThresholdL[1]);
        EEPROM.write(eThreshold2,ThresholdL[2]);
        EEPROM.write(eThreshold3,ThresholdL[3]);
        EEPROM.write(eThreshold4,ThresholdL[4]);
        EEPROM.write(eThreshold5,ThresholdL[5]);
        EEPROM.write(eThreshold6,ThresholdL[6]);
        EEPROM.write(eThreshold7,ThresholdL[7]);
        EEPROM.write(eThreshold8,ThresholdR[0]);
        EEPROM.write(eThreshold9,ThresholdR[1]);
        EEPROM.write(eThreshold10,ThresholdR[2]);
        EEPROM.write(eThreshold11,ThresholdR[3]);
        EEPROM.write(eThreshold12,ThresholdR[4]);
        EEPROM.write(eThreshold13,ThresholdR[5]);
        EEPROM.write(eThreshold14,ThresholdR[6]);
        EEPROM.write(eThreshold15,ThresholdR[7]);
       
        for(int p=0;p<8;p++)
        {
          Serial.print(ThresholdL[p]);
          Serial.print(", ");  
        }
          for(int p=0;p<8;p++)
        {
          Serial.print(ThresholdR[p]);
          Serial.print(", ");  
        }
          Serial.println();
       
        MenuID++;      
        break;

      }
    }
  }
  /***********************
  *******Menu Start*******
  ***********************/
  while(MenuID==1)
  {
  
    lcd.setCursor(0,0);
    lcd.clear();
    //Serial.print("Pilih Posisi");
    lcd.print("Pilih Posisi");
    lcd.setCursor(0,1);
    //Serial.print("Start Ke");
    lcd.print("Start ke ");
    lcd.print(Start+1);
    delay(20);
    if(digitalReads(IncButton)==LOW && digitalRead(DecButton)==HIGH && digitalRead(SelectButton)==HIGH && digitalRead(Interupt)==HIGH)
    {
      Start++;
      if(Start>3)
      {
        Start=3;
      }
    }
    if(digitalReads(DecButton)==LOW && digitalRead(IncButton)==HIGH && digitalRead(SelectButton)==HIGH && digitalRead(Interupt)==HIGH)
    {
      Start--;
      if(Start<0)
      {
        Start=0;
      }
    }
    if(digitalReads(SelectButton)==LOW && digitalRead(IncButton)==HIGH && digitalRead(DecButton)==HIGH && digitalRead(Interupt)==HIGH)
    {
      MenuID++;
      break;
    }
  }
 
  /*****************************
  *******Menu NormPWM********
  ******************************/
  while(MenuID==2)
  {
    lcd.setCursor(0,0);
    lcd.clear();
    //Serial.print("Pilih Nilai");
    lcd.print("Pilih Nilai");
    lcd.setCursor(0,1);
    //Serial.print("NormPWM");
    lcd.print("Normal PWM: ");
    lcd.print(NormPWM);
    delay(20);
    if(digitalReads(IncButton)==LOW && digitalRead(DecButton)==HIGH && digitalRead(SelectButton)==HIGH && digitalRead(Interupt)==HIGH)
    {
      NormPWM+=5;
      if(NormPWM>outMax)
      {
        NormPWM=outMax;
      }
    }
    if(digitalReads(DecButton)==LOW && digitalRead(IncButton)==HIGH && digitalRead(SelectButton)==HIGH && digitalRead(Interupt)==HIGH)
    {
      NormPWM-=5;
      if(NormPWM<0)
      {
        NormPWM=0;
      }
    }
    if(digitalReads(SelectButton)==LOW && digitalRead(IncButton)==HIGH && digitalRead(SelectButton)==HIGH && digitalRead(Interupt)==HIGH)
    {
      //******Merubah nilai NormPWM di EEPROM******
      if(NormPWM!=tempPWM)
      {
        EEPROM.write(eNormPWM, NormPWM);
      }
      MenuID++;
      break;
    }
  }
  /**********************************
  ***********Keluar Menu*************
  **********************************/
  while(MenuID==3)
  {
    lcd.setCursor(0,0);
    lcd.clear();
 
    lcd.setCursor(0,0);
    lcd.clear();
    char siap[]="Robot telah siap";
    for (int i=0; siap[i] !='\0' ;i++)
    {
      lcd.print(siap[i]);
      delay(10);
    }
    //lcd.clear();
    break;
   if(digitalReads(SelectButton)==LOW && digitalRead(IncButton)==HIGH && digitalRead(SelectButton)==HIGH && digitalRead(Interupt)==HIGH)
   {
     MenuID++;
     break;
   }
  }
 
}
void setup() {
  pinMode(BackLight,OUTPUT);
  digitalWrite(BackLight,HIGH);
  lcd.begin(16, 2); //
  // put your setup code here, to run once:
  pinMode(A0_1,OUTPUT);
  pinMode(A1_1,OUTPUT);
  pinMode(A2_1,OUTPUT);
  pinMode(Y1,INPUT);
  pinMode(A0_2,OUTPUT);
  pinMode(A1_2,OUTPUT);
  pinMode(A2_2,OUTPUT);
  pinMode(Y2,INPUT);

  Serial.begin(9600);
  
  pinMode(DIR_R,OUTPUT);
  pinMode(DIR_L,OUTPUT);
  pinMode(PWM_R,OUTPUT);
  pinMode(PWM_L,OUTPUT);
  
   pinMode(IncButton,INPUT);
   pinMode(DecButton,INPUT);
   pinMode(SelectButton,INPUT);
   pinMode(Interupt,INPUT);
   
    //*****Baca nilai NormPWM di EEPROM*****
    tempPWM=EEPROM.read(eNormPWM);
    NormPWM=tempPWM;

   lcd.begin(16,2); //Inisiasi lcd
   char Bismi[]="Here We GO!!!";
   for (int a=0;Bismi[a] !='\0' ;a++)
   {
     lcd.print(Bismi[a]);
     delay(5);
   }
     delay(50);
     lcd.setCursor(0,1);
     char nama[]="Bismillaah...";
     for (int i=0; nama[i] !='\0' ;i++)
    {
    lcd.print(nama[i]);
    delay(5);
    }
     delay(500);
     Menu();
 
     lcd.clear();
     lcd.setCursor(0,0);
     lcd.print("Konfigurasi");
     lcd.setCursor(0,1);
     lcd.print("Di Simpan");
     delay(100);
     lcd.clear();
}
void scanSensor()
{  
   TotalNilaiDigital=0; PV=0;
   SensorKanan=0; SensorKiri=0;
   PWMlvl_R=150; PWMlvl_L=150;
   
    for(SensorID=0;SensorID<8;SensorID++)
    {    
      digitalWrite(A0_1,bitRead(SensorID,0));
      digitalWrite(A1_1,bitRead(SensorID,1));
      digitalWrite(A2_1,bitRead(SensorID,2));
      digitalWrite(A0_2,bitRead(SensorID,0));
      digitalWrite(A1_2,bitRead(SensorID,1));
      digitalWrite(A2_2,bitRead(SensorID,2));
      
      nilaiAnalogL[SensorID]=analogRead(Y1)-900;
      //*********KOMPARATOR*********
      nilaiDigitalL[SensorID]=nilaiAnalogL[SensorID]/ThresholdL[SensorID];
      if(nilaiDigitalL[SensorID]==1)
        PV-=(nilaiDigitalL[SensorID]*bebanSensor[(7-SensorID)]);
      
      nilaiAnalogR[SensorID]=analogRead(Y2)-900; 
      //*********KOMPARATOR*********
      nilaiDigitalR[SensorID]=nilaiAnalogR[SensorID]/ThresholdR[SensorID];
       if(nilaiDigitalR[SensorID]==1)
        PV+=(nilaiDigitalR[SensorID]*bebanSensor[SensorID]);
    
      SensorKanan+=nilaiDigitalR[SensorID];
      SensorKiri+=nilaiDigitalL[SensorID];
    }
       TotalNilaiDigital=SensorKanan+SensorKiri;
       Serial.println(TotalNilaiDigital);
       byteSensorL= (nilaiDigitalL[7]*1)+(nilaiDigitalL[6]*2)+(nilaiDigitalL[5]*4)+(nilaiDigitalL[4]*8)+(nilaiDigitalL[3]*16)+(nilaiDigitalL[2]*32)+(nilaiDigitalL[1]*64)+(nilaiDigitalL[0]*128);
       byteSensorR= (nilaiDigitalR[0]*1)+(nilaiDigitalR[1]*2)+(nilaiDigitalR[2]*4)+(nilaiDigitalR[3]*8)+(nilaiDigitalR[4]*16)+(nilaiDigitalR[5]*32)+(nilaiDigitalR[6]*64)+(nilaiDigitalR[7]*128);

   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   lastTime = millis()-SampleTime;
   if(timeChange>=SampleTime)
   {
    //ITerm = (Ki*(ITerm+ PV));
    ITerm = (Ki*PV);
    if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;

    if (TotalNilaiDigital==0)
    {     
      PV= lastPV;
  
      APWM = (Kp*PV)+ ITerm -(Kd*(PV+lastPV));               //Proposional+Integral+Derivatif

      PWMlvl_R-=APWM;
      if(PWMlvl_R>(2*PWMlvl_R)) PWMlvl_R=(2*PWMlvl_R);
        else if(PWMlvl_R<outMin) {PWMlvl_R=(outMin-PWMlvl_R); Dir_R=MUNDUR;}
      PWMlvl_L+=APWM;
      if(PWMlvl_L>(2*PWMlvl_L)) PWMlvl_L=(2*PWMlvl_L);
        else if(PWMlvl_L<outMin) {PWMlvl_L=(outMin-PWMlvl_L); Dir_L=MUNDUR;}
       
      gerakMotor(PWMlvl_L, Dir_L, PWMlvl_R, Dir_R);
    }
     else  //if (TotalNilaiDigital<4)
    {  
      PV/=TotalNilaiDigital;
   
      APWM = (Kp*PV) + ITerm - (Kd*(PV+lastPV));              //Kendali PID
    
      PWMlvl_R-=APWM;
      if(PWMlvl_R>outMax) {PWMlvl_R=outMax; }
        else if(PWMlvl_R<outMin) PWMlvl_R=0 ;
      PWMlvl_L+=APWM;
      if(PWMlvl_L>outMax) {PWMlvl_L=outMax; }
        else if(PWMlvl_L<outMin) PWMlvl_L=0 ;
       
      gerakMotor(PWMlvl_L, Dir_L, PWMlvl_R, Dir_R);
    } 
       lastPV= PV;
       lastTime= now;
     
     if (nilaiDigitalL[7]==1 && nilaiDigitalR[0]==1) gerakMotor(150,0,150,0);
       
     if (nilaiDigitalR[7]==0 && nilaiDigitalL[0]==1)  { gerakMotor(5, 1, 245, 0); delay(250);} // motor belok
     if (nilaiDigitalR[7]==1 && nilaiDigitalL[0]==0)  { gerakMotor(245, 0, 5, 1); delay(250);} // tajam

     if(TotalNilaiDigital>8) Simpang();
    
   }
}

void loop() {
  /**************Scan Sensor***************/  
  SensorAktif= TotalNilaiDigital;
   unsigned char  BSR= byteSensorL;
   unsigned char  BSL= byteSensorR;
 // Serial.println(t);
 //t+=1;
scanSensor();
//Cekpoint1();
/*Cekpoint2();
Cekpoint3();
Cekpoint4();
Cekpoint5();
Cekpoint6();
KotakFinish();  */
  
  int i, u;
  int pos;
  lcd.setCursor(0,0);
  for(SensorID=0;SensorID<8;SensorID++)
    {lcd.print(nilaiDigitalL[SensorID]);}
  lcd.setCursor(8,0);
  for(SensorID=0;SensorID<8;SensorID++)
    {lcd.print(nilaiDigitalR[SensorID]);}
  lcd.setCursor(0,1);
  lcd.print("SR:");
  lcd.print(SensorKanan);
  lcd.setCursor(5,1);
  lcd.print("SL:");
  lcd.print(SensorKiri);
  lcd.setCursor(10,1);
  lcd.print("T:");
  lcd.print(t);
  /*
  for(i=0; i<8; i++)
  {
    Serial.print(nilaiAnalogL[i]);
    Serial.print(", ");  
  }
  for(i=0; i<8; i++)
  {
    Serial.print(nilaiAnalogR[i]);
    Serial.print(", ");  
  }
  Serial.println();
  delay(100);
  */
}

void Brake()
{
  digitalWrite(DIR_L, LOW);
  digitalWrite(DIR_R, LOW);
 
  analogWrite(PWM_L,255);
  analogWrite(PWM_R,255);
}

void Simpang()
{
  //Serial.println("Simpang!!");
  //gerakMotor(0,MUNDUR,0,MUNDUR);
  if(SensorKanan > SensorKiri)  {gerakMotor(250,0 ,15, 1);  delay(300);} 
   else if (SensorKiri > SensorKanan)  {gerakMotor(50,1, 250,0);  delay(300);} 
  else  {  gerakMotor(100, 0, 100, 0);  lastPV=PV;}

  if (TotalNilaiDigital>12)
    { 
      gerakMotor(200, 0, 193, 0); delay(200);
      PV=lastPV;
      if(SensorKanan > SensorKiri) {gerakMotor(250,0 ,15, 1);  delay(250);}
        else if (SensorKiri > SensorKanan)  {gerakMotor(50,1, 250,0);  delay(250);}
      if(TotalNilaiDigital=16)
      Brake();
    }
}
/*
void Cekpoint1()
{ 
  if(t==0)
  {
     MajuPID();
     if(SensorKiri > SensorKanan)
      {
        MajuPID();
        if(SensorKiri > SensorKanan)
      t+=1;
      }
  }
  if(t==1)
  {
    MajuPID();
    if(SensorKanan>5 && SensorKiri<5)
    {
      gerakMotor(240,0, 170,1); delay(450); 
      t+=1;
    }
  }
  if(t==2)
  {
    MajuPID();
    if(nilaiDigitalR[0]==0 && nilaiDigital[7]==0)
      MajuPID();
      else
      {
        if (SensorAktif<=2) t+=1; 
      }
  }
  if(t==3)
  {
    MajuPID();
    if(SensorKanan> SensorKiri || SensorKiri > SensorKanan)
    gerakMotor(250,1,250,0);delay(400);
    t+=1;
  }
  if(t==4)
  {
    MajuPID();
    t=4;
  }
}
*/

void gerakMotor(int PWMLevel_L, byte dir_L, int PWMLevel_R, byte dir_R)
{  
  digitalWrite(DIR_R, dir_R);
  digitalWrite(DIR_L, dir_L);
  analogWrite(PWM_R, (255-PWMLevel_R));
  analogWrite(PWM_L, (255-PWMLevel_L));
}

/*********************************************************
************************Menu Retry************************
*********************************************************/
void Retry()
{
  Run=0;
  //****MATIKAN MOTOR****
  analogWrite(PWM_L,255);
  analogWrite(PWM_R,255);

  while(!Run)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Retry CP: ");
    lcd.setCursor(0,1);
    lcd.print(CP);
    delay(20);
     if(digitalReads(IncButton)==LOW && digitalRead(DecButton)==HIGH && digitalRead(SelectButton)==HIGH && digitalRead(Interupt)==HIGH)
    {
      CP++;
      if(CP>5) CP=5;
    }
    if(digitalReads(DecButton)==LOW && digitalRead(IncButton)==HIGH && digitalRead(SelectButton)==HIGH && digitalRead(Interupt)==HIGH)
    {
      CP--;
      if(CP<0) CP=0;
    }
    if(digitalReads(SelectButton)==LOW && digitalRead(IncButton)==HIGH && digitalRead(SelectButton)==HIGH && digitalRead(Interupt)==HIGH)
    {
      break;
    }
  }
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Checkpoint Save");
  lcd.setCursor(0,1);
  lcd.print("Start!");
  //delay(20);
  while(!Run)
  {
    if(digitalReads(SelectButton)==LOW && digitalRead(IncButton)==HIGH && digitalRead(SelectButton)==HIGH && digitalRead(Interupt)==HIGH)
    {
      Run=1;
    }
  }
  if(CP==0)
  {
    CState=0;
  }
  else if(CP==1)
  {
    CState=7;
  }
  else if(CP==2)
  {
    CState=23;
  }
  else if(CP==3)
  {
    CState=42;
  }
  else if(CP==4)
  {
    CState=67;
  }
  else if(CP==5)
  {
    CState=75;
  }
  
}
