#include <LiquidCrystal.h>

//define port LCD
LiquidCrystal lcd(7,6,5,4,3,2);
#define BackLight 2

//DEFINE BEBAN SENSOR
#define WS0    -21
#define WS1    -18
#define WS2    -15
#define WS3    -12
#define WS4    -9
#define WS5    -6
#define WS6    -3
#define WS7     0
#define WS8     0
#define WS9     3
#define WS10    6
#define WS11    9
#define WS12    12
#define WS13    15
#define WS14    18
#define WS15    21

//DEFINE MOTOR
#define DIR_R  12
#define DIR_L  13
#define PWM_L  10
#define PWM_R  11
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
#define THRESHOLD 998

//int r0_1,r1_1,r2_1,r0_2,r1_2,r2_2=0;
int nilaiDigital[16];
//int bebanSensor[16]={WS0, WS1, WS2, WS3, WS4, WS5, WS6, WS7, WS8, WS9, WS10, WS11, WS12, WS13, WS14, WS15};
int bebanSensor[8]={WS8, WS9, WS10, WS11, WS12, WS13, WS14, WS15};
int nilaiAnalogL[8], nilaiAnalogR[8], nilaiDigitalL[8], nilaiDigitalR[8];
int SensorID, SensorAktif;
int TotalNilaiDigital, SensorKanan, SensorKiri, PV;
int SampleTime=100, lastTime;
float  Kp=4, Ki=1, Kd=1, ITerm, APWM, NORMPWM;
int outMax=255, outMin=0, lastPV;
byte Dir_R, Dir_L;
int Case=0, nextCase, lastCase=0, totalCase=80, Counter_Simpang;
int PWMlvl_R, PWMlvl_L;
//byte dir_R, dir_L;

void Simpang()
{
  //Serial.println("Simpang!!");
  //gerakMotor(0,MUNDUR,0,MUNDUR);
  if(SensorKanan> SensorKiri){PutarKanan();}
  else if(SensorKanan< SensorKiri){PutarKiri();}
  else if (TotalNilaiDigital>=9)Brake();
 
}

/*********************************
**********Gerakan Robot***********
*********************************/
void Maju()
{
  scanSensor();
  digitalWrite(DIR_L, LOW);
  digitalWrite(DIR_R, LOW);
 
  analogWrite(PWM_L,(255-PWMlvl_L));
  analogWrite(PWM_R,(249-PWMlvl_R));
}

void PutarKiri()
{
  digitalWrite(DIR_L, HIGH);
  digitalWrite(DIR_R, LOW);
 
  analogWrite(PWM_R,(249-PWMlvl_R));
  analogWrite(PWM_L,(255-PWMlvl_L));

}

void PutarKanan()
{
  digitalWrite(DIR_L, LOW);
  digitalWrite(DIR_R, HIGH);
 
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


void scanSensor()
{
    NORMPWM=50;
    SensorKiri=0; SensorKanan=0;
    TotalNilaiDigital=0; PV=0;
    PWMlvl_R=NORMPWM, PWMlvl_L=NORMPWM;
    
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
      //if(SensorID==0)
        //nilaiDigitalR[SensorID]=nilaiAnalogR[SensorID]/940;
      if(nilaiDigitalR[SensorID]==1)
        PV+=(nilaiDigitalR[SensorID]*bebanSensor[(SensorID)]);
  
      SensorKanan+=nilaiDigitalR[SensorID];
      SensorKiri+=nilaiDigitalL[SensorID];
    }
    TotalNilaiDigital=SensorKanan+SensorKiri;
   
      /*******************************
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   lastTime = millis()-SampleTime;
   if(timeChange>=SampleTime)
   {
    //ITerm = (Ki*(ITerm+ PV));
    ITerm = (Ki*PV);
    if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
     /********************************/
    if (TotalNilaiDigital==0)
    {  //Serial.print("PUTIH");
      if(PV!=lastPV) gerakMotor(0, 0, 0, 0);
     
      PV= lastPV;
  
      APWM = (Kp*PV)+ (Kd*(PV-lastPV)) + (Kd*(PV+lastPV)); //Proposional+Integral+Derivatif

      PWMlvl_R+=APWM;
      if(PWMlvl_R>(2*PWMlvl_R)) PWMlvl_R=2*PWMlvl_R;
        else if(PWMlvl_R<outMin) PWMlvl_R=(outMin-PWMlvl_R);
      PWMlvl_L-=APWM;
      if(PWMlvl_L>(2*PWMlvl_L)) PWMlvl_L=2*PWMlvl_L;
        else if(PWMlvl_L<outMin) PWMlvl_L=(outMin-PWMlvl_L);
       
      if(APWM<0)
      {
        Dir_L=MAJU;
        Dir_R=MUNDUR;
      }
      if(APWM>=0)
      {
        Dir_L=MUNDUR;
        Dir_R=MAJU;
      }
       
      gerakMotor(PWMlvl_R, Dir_R, PWMlvl_L, Dir_L);
    }
    else //if(TotalNilaiDigital>0)
    {  //Serial.println("PID");
   
      /*if(SensorKiri>5 || SensorKanan>5)
      {
        Simpang();
        Counter_Simpang++;
      }*/
      PV/=TotalNilaiDigital;
   
      APWM = (Kp*PV)+ (Kd*(PV-lastPV)) + (Kd*(PV+lastPV));//Kendali PID

      PWMlvl_R+=APWM;
      if(PWMlvl_R>(1.5*NORMPWM)) {PWMlvl_R=(1.5*NORMPWM); Dir_R=MAJU;}
        else if(PWMlvl_R<outMin) PWMlvl_R=outMin;
      PWMlvl_L-=APWM;
      if(PWMlvl_L>(1.5*NORMPWM)) {PWMlvl_L=(1.5*NORMPWM); Dir_L=MAJU;}
        else if(PWMlvl_L<outMin) PWMlvl_L=outMin;
       
      gerakMotor(PWMlvl_R, Dir_R, PWMlvl_L, Dir_L);
     }
    
     lastPV= PV;
     //lastTime = now;
     //if (TotalNilaiDigital>=5) {Simpang(); lastPV=PV;}
  
}

void gerakMotor(int PWMLevel_L, byte dir_L, int PWMLevel_R, byte dir_R)
{
  digitalWrite(DIR_R, dir_R);
  digitalWrite(DIR_L, dir_L);
  analogWrite(PWM_R, (255-PWMLevel_R));
  analogWrite(PWM_L, (255-(0.93*PWMLevel_L)));

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
  // put your main code here, to run repeatedly:
 
  /****************************************
  *****************************************
  **************Scan Sensor***************
  ******************************************
  ****************************************/
   scanSensor();
    //Mission();
  int i;
 
  int pos;
  lcd.setCursor(0,0);
  //for(pos=7;pos>=0;pos--)
    //{lcd.print(pos);}
  lcd.setCursor(0,0);
  for(SensorID=0;SensorID<8;SensorID++)
    {lcd.print(nilaiDigitalL[SensorID]);}
  //lcd.setCursor(8,0);
  //for(pos=0;pos<8;pos++)
    //{lcd.print(pos);}
  lcd.setCursor(8,0);
  for(SensorID=0;SensorID<8;SensorID++)
    {lcd.print(nilaiDigitalR[SensorID]);}
  lcd.setCursor(0,1);
  /*lcd.print("R=");
  lcd.print(255-PWMlvl_R);
  lcd.setCursor(8,1);
  lcd.print("L=");
  lcd.print(255-PWMlvl_L);*/
  //lcd.print("R:");
  //lcd.print(SensorKiri);lcd.print(",");lcd.print(SensorKanan);
  //lcd.setCursor(4,1);
  lcd.print("Case:");
  lcd.print(Case);
  lcd.setCursor(10,1);
  lcd.print("LC:");
  lcd.print(lastCase);
  //Serial.println(255 -PWMlvl_R);
  //Serial.println(255 -PWMlvl_L);
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
  //Serial.print(PWMlvl_R);
  //Serial.print(", ");
  //Serial.println(PWMlvl_L);
  Serial.println();
  delay(500);
  */
//gerakMotor(PWM_Kiri, DIR_Kiri, PWM_Kanan, DIR_Kanan)
 // gerakMotor(100,MAJU,100,MAJU);
}
