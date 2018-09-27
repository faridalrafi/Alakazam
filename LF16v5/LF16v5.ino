#include <LiquidCrystal.h>

//define port LCD
LiquidCrystal lcd(7,6,5,4,3,2);
#define BackLight 2

//DEFINE BEBAN SENSOR
#define WS8     0
#define WS9     3
#define WS10    6
#define WS11    9
#define WS12    12
#define WS13    17
#define WS14    20
#define WS15    22

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
#define THRESHOLD 999

//int r0_1,r1_1,r2_1,r0_2,r1_2,r2_2=0;
int nilaiDigital[16];
//int bebanSensor[16]={WS0, WS1, WS2, WS3, WS4, WS5, WS6, WS7, WS8, WS9, WS10, WS11, WS12, WS13, WS14, WS15};
int bebanSensor[8]={WS8, WS9, WS10, WS11, WS12, WS13, WS14, WS15};
int nilaiAnalogL[8], nilaiAnalogR[8];
int nilaiDigitalL[8], nilaiDigitalR[8];
int SensorID, SensorAktif;
int TotalNilaiDigital, SensorKanan, SensorKiri, PV;
int SampleTime=100, lastTime;
float  Kp=5, Ki=6, Kd=1, ITerm, APWM;
int outMax=255, outMin=0, lastPV;
byte Dir_R, Dir_L; 
int Case, nextCase, lastCase, totalCase=80;
int PWMlvl_R, PWMlvl_L;
unsigned char byteSensorL, byteSensorR;
//byte dir_R, dir_L;
int t=0;

void scanSensor()
{  
    SensorKiri=0; SensorKanan=0;
    TotalNilaiDigital=0; PV=0;
      
    for(SensorID=0;SensorID<8;SensorID++)
    {    
      digitalWrite(A0_1,bitRead(SensorID,0));
      digitalWrite(A1_1,bitRead(SensorID,1));
      digitalWrite(A2_1,bitRead(SensorID,2));
      digitalWrite(A0_2,bitRead(SensorID,0));
      digitalWrite(A1_2,bitRead(SensorID,1));
      digitalWrite(A2_2,bitRead(SensorID,2));
      
      nilaiAnalogL[SensorID]=analogRead(Y1);
      //*********KOMPARATOR*********
      nilaiDigitalL[SensorID]=nilaiAnalogL[SensorID]/THRESHOLD;
      if(nilaiDigitalL[SensorID]==1)
        PV-=(nilaiDigitalL[SensorID]*bebanSensor[(7-SensorID)]);
      
      nilaiAnalogR[SensorID]=analogRead(Y2); 
      //*********KOMPARATOR*********
      nilaiDigitalR[SensorID]=nilaiAnalogR[SensorID]/THRESHOLD;
       if(nilaiDigitalR[SensorID]==1)
        PV+=(nilaiDigitalR[SensorID]*bebanSensor[(SensorID)]);
    
      SensorKanan+=nilaiDigitalR[SensorID];
      SensorKiri+=nilaiDigitalL[SensorID];
    }
       TotalNilaiDigital=SensorKanan+SensorKiri;
       byteSensorL= (nilaiDigitalL[7]*1)+(nilaiDigitalL[6]*2)+(nilaiDigitalL[5]*4)+(nilaiDigitalL[4]*8)+(nilaiDigitalL[3]*16)+(nilaiDigitalL[2]*32)+(nilaiDigitalL[1]*64)+(nilaiDigitalL[0]*128);
       byteSensorR= (nilaiDigitalR[0]*1)+(nilaiDigitalR[1]*2)+(nilaiDigitalR[2]*4)+(nilaiDigitalR[3]*8)+(nilaiDigitalR[4]*16)+(nilaiDigitalR[5]*32)+(nilaiDigitalR[6]*64)+(nilaiDigitalR[7]*128);
}

void gerakMotor(int PWMLevel_L, byte dir_L, int PWMLevel_R, byte dir_R)
{  
  digitalWrite(DIR_R, dir_R);
  digitalWrite(DIR_L, dir_L);
  analogWrite(PWM_R, (255-(7+PWMLevel_R)));
  analogWrite(PWM_L, (255-PWMLevel_L));
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
}

void loop() {
  /**************Scan Sensor***************/
  //scanSensor();
//  Finish();
  
  SensorAktif= TotalNilaiDigital;
   unsigned char  BSR= byteSensorL;
   unsigned char  BSL= byteSensorR;
  Serial.println(t);
  //t+=1;
  scanSensor();
Cekpoint1();
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

void Simpang()
{
  //Serial.println("Simpang!!");
  //gerakMotor(0,MUNDUR,0,MUNDUR);
  if(SensorKanan > SensorKiri)  PutarKanan();//  delay(100); 
    else if (SensorKiri > SensorKanan)  PutarKiri();  //delay(100); 
  else  {  gerakMotor(100, 0, 100, 0);  lastPV=PV;}

  if (TotalNilaiDigital>12)
    { 
      gerakMotor(200, 0, 200, 0); delay(200);
      PV=lastPV;
      if(SensorKanan > SensorKiri) {PutarKanan();  delay(400);}
        else if (SensorKiri > SensorKanan) { PutarKiri();  delay(400);}
      if(TotalNilaiDigital=16)
      Brake();
    }
 lastPV=PV;
}

/*********************************
**********Misi Robot***********
*********************************/
void Maju()
{
  //scanSensor();
  digitalWrite(DIR_L, LOW);
  digitalWrite(DIR_R, LOW);
 
  analogWrite(PWM_L,(255-PWMlvl_L));
  analogWrite(PWM_R,(249-PWMlvl_R));
}

void PutarKiri()
{
  digitalWrite(DIR_L, HIGH);
  digitalWrite(DIR_R, LOW);
 
  if(3*PWMlvl_R<=255)
  analogWrite(PWM_R,(249-(3*PWMlvl_R)));
  else
  analogWrite(PWM_R,(249-PWMlvl_R));  
  analogWrite(PWM_L,(255-PWMlvl_L));

}

void PutarKanan()
{
  digitalWrite(DIR_L, LOW);
  digitalWrite(DIR_R, HIGH);
 
  if(3*PWMlvl_L<=255)
  analogWrite(PWM_L,(255-(3*PWMlvl_L)));
  else
  analogWrite(PWM_L,(255-PWMlvl_L));
  analogWrite(PWM_R,(249-PWMlvl_R));
}

void Mundur()
{
  digitalWrite(DIR_L, HIGH);
  digitalWrite(DIR_R, HIGH);
 
  analogWrite(PWM_L,(255-PWMlvl_L));
  analogWrite(PWM_R,(249-PWMlvl_R));
}

void Brake()
{
  digitalWrite(DIR_L, LOW);
  digitalWrite(DIR_R, LOW);
 
  analogWrite(PWM_L,255);
  analogWrite(PWM_R,255);
}

void Balik()
{   
      PV= lastPV;
  
      APWM = (Kp*PV)+ (Ki*(PV-lastPV)) +(Kd*(PV+lastPV)); //Proposional+Integral+Derivatif

      PWMlvl_R-=APWM;
      if(PWMlvl_R>(2*PWMlvl_R)) PWMlvl_R=(2*PWMlvl_R);
        else if(PWMlvl_R<outMin) PWMlvl_R=(outMin-PWMlvl_R);
      PWMlvl_L+=APWM;
      if(PWMlvl_L>(2*PWMlvl_L)) PWMlvl_L=(2*PWMlvl_L);
        else if(PWMlvl_L<outMin) PWMlvl_L=(outMin-PWMlvl_L);
       
      gerakMotor(PWMlvl_L, Dir_L, PWMlvl_R, Dir_R);
    lastPV=PV;
}
 void MajuPID()
 {   
      PV/=TotalNilaiDigital;
   
      APWM = (Kp*PV) + (Ki*(PV-lastPV)) + (Kd*(PV+lastPV));//Kendali PID
    
      PWMlvl_R-=APWM;
      if(PWMlvl_R>outMax) {PWMlvl_R=outMax; }
        else if(PWMlvl_R<outMin) PWMlvl_R=0 ;
      PWMlvl_L+=APWM;
      if(PWMlvl_L>outMax) {PWMlvl_L=outMax; }
        else if(PWMlvl_L<outMin) PWMlvl_L=5 ;
       
      gerakMotor(PWMlvl_L, Dir_L, PWMlvl_R, Dir_R);
    lastPV=PV;
}

void Finish()
{  SensorAktif= TotalNilaiDigital;
   unsigned char  BSR= byteSensorL;
   unsigned char  BSL= byteSensorR;
  Serial.println(t);
  //t+=1;
  scanSensor();
Cekpoint1();
/*Cekpoint2();
Cekpoint3();
Cekpoint4();
Cekpoint5();
Cekpoint6();
KotakFinish();  */
}

void Cekpoint1()
{
  if(SensorAktif != 0 && t==0)
  {
    gerakMotor(200, 0, 200, 0);
   // Serial.println("BABLAS");
    if(SensorKiri>7 && SensorKanan<2)
    {
      //lcd.setCursor(0,1);
     char nama[]="Bismillaah...";
     for (int i=0; i!='\0' ;i++)
    {
    Serial.print("gh");
    delay(30);
      }
    }
  }
  
  if(t==1){
  Brake();
  Serial.println("adadadasd");
  }
 /*
  if(SensorAktif != 0 && t==1)
  {
    MajuPID();
    if(nilaiDigitalL[0]==1 && nilaiDigitalR[7]==0)
      {
        for(int i=0; i != '\0'; i++)
        {
          gerakMotor(100, 1, 200, 0);
          if(SensorAktif<=2)
          break; 
        }
      }
         t+=1; //Serial.println(t);
  }
  
  if(SensorAktif != 0 && t==2)
  {
    if(SensorKanan> SensorKiri)
    {
      for(int i=0;i != '\0'; i++)
      {
        gerakMotor(200, 1, 5, 1);
        if(SensorAktif<=2)
        break; 
      }
    }
        t+=1; //Serial.println(t);
  }
 /* 
  if(SensorAktif != 0 && t==3)
  { 
    if(SensorAktif==0) Balik();
    else
    {
      for(int a=0; a!= '\0';a++)
       {
         MajuPID();
         if(SensorKanan>2 && SensorKiri==0)
         break;
       }
       t+=1; //Serial.println(t);
    }
  }
  
  if(SensorKanan> SensorKiri && t==4)
  {
     for(int a=0; a!= '\0';a++)
     {
        gerakMotor(250, 0, 0, 1);
        if(SensorKanan==SensorKiri)
        break;
     }  
     t+=1; //Serial.println(t);
  }
 /* 
  if(SensorAktif != 0 && t==5)
  {
    MajuPID();
    if(nilaiDigitalL[7]==1 && nilaiDigital[0]==1)
    {
      for(int i=0;i != '\0'; i++)
      {
        gerakMotor(200,0,200,0);
        if(SensorAktif<=2)
        break;
      }  
      t+=1; //Serial.println(t);
    }
  }
  if(SensorAktif != 0 && t==6)
  {
    MajuPID();
    for(int i=0; i!= '\0';i++)
    {
      if(nilaiDigitalR[7]==1)
      {
        gerakMotor(1,1, 240, 0);
        if(SensorKanan==SensorKiri)
        break;
       }  
       t+=1; //Serial.println(t);
    }
  }
  if(SensorAktif != 0 && t==7)
  {
    gerakMotor(0,0,0,0);
    Serial.println("STOP");
    t=7;
  }
*/
}

