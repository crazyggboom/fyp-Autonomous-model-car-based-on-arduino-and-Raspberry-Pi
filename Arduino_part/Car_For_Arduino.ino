
#include <SSD1306.h>         //Library for OLED
#include <Servo.h>          //Library for Servo
#include <PinChangeInt.h>    //external break
#include <MsTimer2.h>        //timing break
#include <PS2X_lib.h>        //library for PS2
Servo myservo;  //create a servo
PS2X ps2x; // create PS2 Controller Class
//////////PS2 pin//////////////////
#define PS2_DAT        57  //14
#define PS2_CMD        0  //15
#define PS2_SEL        56   //16
#define PS2_CLK        55  //17
////////OLED pin///////////
#define OLED_DC 10
#define OLED_CLK 59
#define OLED_MOSI 13
#define OLED_RESET 12
/////////TB6612 driver pin////
#define AIN1 11
#define AIN2 5
#define BIN1 6
#define BIN2 3
#define SERVO 9
/////////encoder pin////////
#define ENCODER_L 8
#define DIRECTION_L 62
#define ENCODER_R 7
#define DIRECTION_R 2
/////////key pin////////
#define KEY 58
#define T 0.156f
#define L 0.1445f
#define pi 3.1415926
//////////ultrasonic pin///////////
#define Trig 39 
#define Echo 37
SSD1306 oled(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, 0); //initial setting for oled
volatile long Velocity_L, Velocity_R ;   //encoder data
int Velocity_Left, Velocity_Right = 0, Velocity, Angle;   //wheel speeds
char Flag_Direction, Flag_Way = 1;
float Velocity_KP = 0.7, Velocity_KI =  0.3;
unsigned char Flag_Stop = 0, PID_Send, Flash_Send, Bluetooth_Velocity = 15;
float Target_A, Target_B;
int Battery_Voltage, ps2_data; // battery data
unsigned char servo, PS2_LY, PS2_RX;
unsigned char ADV[128] = {0};
void (* resetFunc) (void) = 0;// Reset func


/******function: mode choosen************************************/
unsigned char  select(void) {
  int count, AngleX = 130;
  static unsigned char flag = 1;
  oled_show_once();  //show OLED data
  count = abs(Velocity_R);
  Flag_Way = 1; //PS2 Mode
  if (digitalRead(KEY) == 0)oled.clear(), flag = 0; // Clear OLED
  return flag;
}
/**************************************************************************
  function：key condition
  input parameters：none
  output parameters：key condition 0：no click 1 click
**************************************************************************/
unsigned char My_click (void) {
  static byte flag_key = 1; //key released
  if (flag_key && (digitalRead(KEY) == 0))   { //if key is clicked
    flag_key = 0;
    if (digitalRead(KEY) == 0)  return 1;    //key clicked
  }
  else if (digitalRead(KEY) == 1)     flag_key = 1;
  return 0;//no key clicked
}
/**************************************************************************
  function: power calculation
  input parameters：m,n
  output parameters：m^n
**************************************************************************/
uint32_t oled_pow(uint8_t m, uint8_t n) {
  uint32_t result = 1;
  while (n--)result *= m;
  return result;
}
/**************************************************************************
  funtion：display parameters
  input parameters：x:x coordinate   y:row    num：parameters   len ：the length of parameters
**************************************************************************/
void OLED_ShowNumber(uint8_t x, uint8_t y, uint32_t num, uint8_t len) {
  uint8_t t, temp;
  uint8_t enshow = 0;
  for (t = 0; t < len; t++)  {
    temp = (num / oled_pow(10, len - t - 1)) % 10;
    oled.drawchar(x + 6 * t, y, temp + '0');
  }
}
/**************************************************************************
  function：save number into PWM register
  input parameter：PWM
**************************************************************************/
void Set_Pwm(int motora, int motorb) {
  if (motora > 0)       analogWrite(AIN2, motora), digitalWrite(AIN1, LOW); //
  else                 digitalWrite(AIN1, HIGH), analogWrite(AIN2, 255 + motora); //

  if (motorb > 0)        digitalWrite(BIN2, LOW), analogWrite(BIN1, motorb); //
  else                  analogWrite(BIN1, 255 + motorb), digitalWrite(BIN2, HIGH); //
}
/**************************************************************************
  fcuntion：close moter abnormally
  input parameter：voltage
  output parameter：1：abnormal  0：normal
  /**************************************************************************/
unsigned char  Turn_Off() {
  byte temp;
  if (Flag_Stop == 1 || Battery_Voltage < 700) { //Flag_Stop is 1 or voltage too low
    temp = 1;
    digitalWrite(AIN1, LOW);  //close motor driver
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
  }
  else      temp = 0;
  return temp;
}
/**************************************************************************
  function：Mathematical model of motion
  input parameters：velocity and angle
  //**************************************************************************/
void Kinematic_Analysis(float velocity, float angle) {
  char K = 1;
  Target_A = velocity * (1 + T * tan(angle * pi / 180) / 2 / L);
  Target_B = velocity * (1 - T * tan(angle * pi / 180) / 2 / L); //the rear wheels differential
  servo = 95 + angle * K;                      //servo steering
  //  if(servo>95)servo=servo*1.15;
  myservo.write(servo);                        // set the data for servo
}
/**************************************************************************
 funtion: Velocity PI controller
**************************************************************************/
int Incremental_PI_A (int Encoder, int Target)
{
  static float Bias, Pwm, Last_bias;
  Bias = Encoder - Target;                              //calculate bias
  Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; //incremental PID controller
  if (Pwm > 255)Pwm = 255;                              //amplitude limits
  if (Pwm < -255)Pwm = -255;                            //amplitude limits
  Last_bias = Bias;                                     //storage of last bias
  return Pwm;                                           //output incremental
}
int Incremental_PI_B (int Encoder, int Target)
{
  static float Bias, Pwm, Last_bias;
  Bias = Encoder - Target;                              //calculate bias
  Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias; //incremental PID controller
  if (Pwm > 255)Pwm = 255;                              //amplitude limits
  if (Pwm < -255)Pwm = -255;                            //amplitude limits
  Last_bias = Bias;                                     //storage of last bias
  return Pwm;                                           //output incremental
}
/*********function：5ms control core *******/
void control() {
  int Temp, Temp2, Motora, Motorb; //tempprary parameters
  static float Voltage_All; //sampled volatge data
  static unsigned char Position_Count, Voltage_Count; //position control parameters
  sei();//break
  Velocity_Left = Velocity_L;    Velocity_L = 0;  //read left wheel data and clear
  Velocity_Right = Velocity_R;    Velocity_R = 0; //read right wheel data and clear
  Get_RC();
  Get_data();
  Kinematic_Analysis(Velocity, Angle);
  Motora = Incremental_PI_A(Target_A, Velocity_Left); // PID speed controller
  Motorb = Incremental_PI_B(Target_B, Velocity_Right);
  if (Turn_Off() == 0) Set_Pwm(Motora, Motorb); //make motor work
  Temp2 = analogRead(0);  //sample the volatge
  Voltage_Count++;
  Voltage_All += Temp2;   //multiply sampled
  if (Voltage_Count == 200) Battery_Voltage = Voltage_All * 0.05371 / 2, Voltage_All = 0, Voltage_Count = 0; //calculate the mean
  Temp = My_click();   //click check
  if (Temp == 1)Flag_Stop = !Flag_Stop;
}

/***************function:get data**********/
char data;
int flag=0,i=0,j=0,cruise=0,cruise_data=0,collision=0,lane=0,way=0;
String comdata= "";
int numdata[5]={0};

void Get_data(){
j=0;
while(Serial3.available()>0)
{
  data = Serial3.read();
  if (data == '}') flag = 2; 
  if(flag == 1) {
    comdata += char(data);
    delay(2);
  }
  if (data == '{') flag = 1;
}

if(flag == 2){
   for(int i = 0; i < comdata.length() ; i++)
   {
      if(comdata[i] == ',')
      {
        j++;
      }
      else
      {
        numdata[j] = numdata[j] * 10 + (comdata[i] - '0');
      }
   }
  cruise = numdata[0];
  cruise_data = numdata[1];
  collision = numdata[2];
  lane = numdata[3];
  way = numdata[4];
  numdata[0]=0;
  numdata[1]=0;
  numdata[2]=0;
  numdata[3]=0;
  numdata[4]=0;
  comdata = String("");
  flag = 0;
  }
  if (cruise == 1) Velocity = cruise_data/2;
  if (collision ==1) Get_distance();
  if (lane == 1) {
    switch (way){
      case 0:
        Angle = -50;
        break;
      case 1:
        Angle = 0;
        break;
      case 2:
        Angle = 50;
        break;
      default:
        Angle = 0;
    }
  }
}
/***************function: get distance**********/
float cm;
float temp;
void Get_distance(){
  digitalWrite(Trig, LOW); //send a low to trig
  delayMicroseconds(2);    //wait 2ms
  digitalWrite(Trig,HIGH); //send a high to trig
  delayMicroseconds(10);    //wait 10ms
  digitalWrite(Trig, LOW); //send a low to trig
  
  temp = float(pulseIn(Echo, HIGH)); //save the waiting time of return wave,
  
  cm = (temp * 17 )/1000; //calculate the distance to the cm
  if (cm<=10) Velocity = 0;
}
/***************function：remote control**********/
void Get_RC(void) {
  char Yuzhi = 10;
  static float Last_Bias;
  float Bias, LY, RX;
  if (Flag_Way == 1)   {//PS2 control
    LY = PS2_LY - 125; //calculate bias
    RX = PS2_RX - 128;
    if (LY > -Yuzhi && LY < Yuzhi)LY = 0; //cencle little angle to avoid shaking
    if (RX > -Yuzhi && RX < Yuzhi)RX = 0;
    Velocity = -LY / 7; //relation between velocity and gear
    Angle = RX / 4;
  }
  if (Angle < -45)Angle = -45;//servo angle limits
  if (Angle > 45)Angle = 45;//servo angle limits
}
/********function：OLED Display*********/
void OLED()
{
  char  i, t;
  if (Flag_Way == 0)   {
    oled.drawstring(00, 00, "SPEED");
    OLED_ShowNumber(45, 00, Bluetooth_Velocity, 3); //PS2 data
    oled.drawstring(00, 01, "RX");
    OLED_ShowNumber(30, 01, Flag_Direction, 3); //PS2 data
  }
  else if (Flag_Way == 1)
  {
    oled.drawstring(00, 0, "LY");
    OLED_ShowNumber(15, 0, PS2_LY, 3); //PS2 data
    oled.drawstring(40, 0, "RX");
    OLED_ShowNumber(55, 0, PS2_RX, 3);
  }
  oled.drawstring(00, 02, "EncoLEFT");  //Encider data
  if ( Velocity_Left < 0)  oled.drawstring(80, 02, "-"),
    OLED_ShowNumber(95, 02, -Velocity_Left, 3);
  else                  oled.drawstring(80, 02, "+"),
                          OLED_ShowNumber(95, 02, Velocity_Left, 3);

  oled.drawstring(00, 03, "EncoRIGHT");
  if (Velocity_Right < 0)  oled.drawstring(80, 03, "-"),
    OLED_ShowNumber(95, 03, -Velocity_Right, 3);
  else                 oled.drawstring(80, 03, "+"),
                         OLED_ShowNumber(95, 03, Velocity_Right, 3);
  oled.drawstring(00, 4, "VOLTAGE:");
  oled.drawstring(71, 4, ".");
  oled.drawstring(93, 4, "V");
  OLED_ShowNumber(58, 4, Battery_Voltage / 100, 2);
  OLED_ShowNumber(81, 4, Battery_Voltage % 100, 2);
  if (Flag_Stop == 0)
    oled.drawstring(103, 04, "O-N");
  if (Flag_Stop == 1)
    oled.drawstring(103, 04, "OFF");
  oled.drawstring(00, 05, "MODE-");
  if (Flag_Way == 1)          oled.drawstring(40, 05, "PS2");

  oled.drawstring(80, 05, "S");
  OLED_ShowNumber(95, 05, servo, 3);
  oled.display();
}
//display content for fist openning
void OLED_Show_CCD(void)  {
  char t;
  for ( unsigned char j = 0; j < 128; j++)   {
    oled_show_shu(j, t);
  }
}
void oled_show_shu(unsigned char x, unsigned char t)  {
  for (unsigned char i = 0; i < 1; i++)    {
    if (t == 1)  oled.setpixel(x, i, WHITE);
    else oled.setpixel(x, i, 0);
  }
}
void oled_show_once(void) {
  oled.drawstring(0, 0, "Turn Right Wheel");
  oled.drawstring(0, 1, "TO Select Mode");
  oled.drawstring(0, 2, "Current Mode Is");
  if (Flag_Way == 0)         oled.drawstring(50, 3, "APP");
  if (Flag_Way == 1)         oled.drawstring(50, 3, "PS2");
  oled.drawstring(0, 4, "Press User Key");
  oled.drawstring(0, 5, "TO End Selection");
  OLED_ShowNumber(0, 6, abs(Velocity_R), 4);
  oled.display();
}
/*********** function: Initialization ************/
void setup()   {
  char error;
  int flag = 0;
  oled.ssd1306_init(SSD1306_SWITCHCAPVCC);
  oled.clear();   // clears the screen and buffer
  pinMode(AIN1, OUTPUT);          //motor control pin
  pinMode(AIN2, OUTPUT);          //motor control pin
  pinMode(BIN1, OUTPUT);          //motor speed control pin
  pinMode(BIN2, OUTPUT);          //motor speed control pin
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  myservo.attach(SERVO);          //initialize servo
  Serial3.begin(9600);
  pinMode(ENCODER_L, INPUT);        //left encoder pin
  pinMode(DIRECTION_L, INPUT);
  pinMode(ENCODER_R, INPUT);        //right encoder pin
  pinMode(DIRECTION_R, INPUT);
  pinMode(KEY, INPUT);              //key pin
  delay(200);
  attachInterrupt(0, READ_ENCODER_R, CHANGE);           //start enternal interrupt, encoder 1
  attachPinChangeInterrupt(62, READ_ENCODER_L, CHANGE);  //start enternal interrupt, encoder 2
  while (select())  { }
  MsTimer2::set(10, control);       //set 5ms interrupt
  MsTimer2::start();               //start the interrupt
  if (Flag_Way == 1)
  {
    error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  }
}
/******function：main loop*******/
void loop() {
  static char flag;
  int Voltage_Temp;
  OLED();
  if (Flag_Way == 1 && flag == 0)
  {
    ps2x.read_gamepad(false, 0); //read controller and set large motor to spin at 'vibrate' speed
    PS2_LY = ps2x.Analog(PSS_LY);
    PS2_RX = ps2x.Analog(PSS_RX);
    Voltage_Temp = (Battery_Voltage - 740) ;
    if (Voltage_Temp > 100)Voltage_Temp = 100;
    if (Voltage_Temp < 0)Voltage_Temp = 0;
    Serial3.print("{A");
    Serial3.print(Velocity_Left);   //left encoder
    Serial3.print(":");
    Serial3.print(Velocity_Right);  //right encoder
    Serial3.print(":");
    Serial3.print(Voltage_Temp);  //battery voltage
    Serial3.print(":");
    Serial3.print(servo - 90); //servo angle
    Serial3.print("}$");
  }
}
/*****function: external interrupt to read left encoder data********/
void READ_ENCODER_L() {
  if (digitalRead(ENCODER_L) == LOW) {     //falling edge trigger
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L--;  //judge direction
    else      Velocity_L++;
  }
  else {     //rising edge trigger
    if (digitalRead(DIRECTION_L) == LOW)      Velocity_L++; //judge direction
    else     Velocity_L--;
  }
}
/*****function: external interrupt to read left encoder data********/
void READ_ENCODER_R() {
  if (digitalRead(ENCODER_R) == LOW) {
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R++;
    else      Velocity_R--;
  }
  else {
    if (digitalRead(DIRECTION_R) == LOW)      Velocity_R--;
    else     Velocity_R++;
  }
}
