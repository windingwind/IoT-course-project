#define SERIAL_RX_BUFFER_SIZE 512 //修改串口发送缓冲区大小为512
#define SERIAL_TX_BUFFER_SIZE 512 //修改串口接收缓冲区大小为512

#include <Wire.h>
#include <Stepper.h>
#include <ArduinoJson.h>
#include <Adafruit_MLX90614.h>
#include <dht11.h>

//User Modified Part
#define wifi_ssid     "wxy"    
#define wifi_psw      "wxy12345678"     
//#define clientIDstr   "123456789abc"
//#define timestamp     "996"
//#define ProductKey    "a1v0RymPkXS"
//#define DeviceName    "test001"
//#define DeviceSecret  "5BYzFuKeS6zTJBadCIpWrWAVS5ZW6dk0"
//#define password      "9B0512B9655ACCCA9B20EC209881F33304467011"

#define clientIDstr   "test"
#define timestamp     "997"
#define ProductKey    "a1EquCiRo6F"
#define DeviceName    "ioT_test"
#define DeviceSecret  "mHwHytlM80XYH8wdse3f5BiCTb7rA68Q"
#define password      "E072FAD8158EE9AC03B1161FDA418C1F1348AAB6"


//Logic Preset
#define OFF           0
#define ON            1
#define MUTE          2
#define KEEP_OFF      2
#define KEEP_ON       3

#define AC_ON   digitalWrite(ACPin,HIGH)
#define AC_OFF  digitalWrite(ACPin,LOW)

#define Fan_ON      digitalWrite(FanPin,HIGH)
#define Fan_OFF     digitalWrite(FanPin,LOW)

#define Buzzer_ON   digitalWrite(BuzzerPin,HIGH)
#define Buzzer_OFF  digitalWrite(BuzzerPin,LOW)

#define Pump_ON     digitalWrite(PumpPin,HIGH)
#define Pump_OFF    digitalWrite(PumpPin,LOW)


//ATcmd Format
//ATcmd Format
#define AT                    "AT\r"
#define AT_OK                 "OK"
#define AT_REBOOT             "AT+REBOOT\r"
#define AT_ECHO_OFF           "AT+UARTE=OFF\r"
#define AT_MSG_ON             "AT+WEVENT=ON\r"

#define AT_WIFI_START         "AT+WJAP=%s,%s\r"
#define AT_WIFI_START_SUCC    "+WEVENT:STATION_UP"

#define AT_MQTT_AUTH          "AT+MQTTAUTH=%s&%s,%s\r"
#define AT_MQTT_CID           "AT+MQTTCID=%s|securemode=3\\,signmethod=hmacsha1\\,timestamp=%s|\r"
#define AT_MQTT_SOCK          "AT+MQTTSOCK=%s.iot-as-mqtt.cn-shanghai.aliyuncs.com,1883\r"

#define AT_MQTT_AUTOSTART_OFF "AT+MQTTAUTOSTART=OFF\r"
#define AT_MQTT_ALIVE         "AT+MQTTKEEPALIVE=500\r"
#define AT_MQTT_START         "AT+MQTTSTART\r"
#define AT_MQTT_START_SUCC    "+MQTTEVENT:CONNECT,SUCCESS"
#define AT_MQTT_PUB_SET       "AT+MQTTPUB=/sys/%s/%s/thing/event/property/post,1\r"
#define AT_MQTT_PUB_ALARM_SET "AT+MQTTPUB=/sys/%s/%s/thing/event/GasAlarm/post,1\r"
#define AT_MQTT_PUB_DATA      "AT+MQTTSEND=%d\r"
#define JSON_ENV_DATA_PACK        "{\"id\":\"100\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"IndoorTemperature\":%d.%01d,\"EnvHumidity\":%d.%01d,\"PhotoResistors\":%d,\"PM25\":%d,\"co2\":%d}}\r"
#define JSON_BODYTEMP_DATA_PACK      "{\"id\":\"110\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"BodyTemperature\":%d.%02d,\"bodyTempStatus\":%d}}\r"
#define JSON_DOORSTA_DATA_PACK      "{\"id\":\"120\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"doorStatus\":%d}}\r"
#define AT_MQTT_PUB_DATA_SUCC "+MQTTEVENT:PUBLISH,SUCCESS"
#define AT_MQTT_UNSUB         "AT+MQTTUNSUB=2\r"
#define AT_MQTT_SUB           "AT+MQTTSUB=1,/sys/%s/%s/thing/service/property/set,1\r"
#define AT_MQTT_SUB_SUCC      "+MQTTEVENT:1,SUBSCRIBE,SUCCESS"

#define AT_BUZZER_MUTE           "\"Buzzer\":2"

#define RPin 7
#define GPin 8
#define BPin 9

#define DEFAULT_TIMEOUT       10   //seconds
#define BUF_LEN               100
#define BUF_LEN_DATA          190

dht11 DHT11;

#define DHT11PIN 22

#define STEPS 64

Stepper stepper(STEPS, 47, 51, 49, 53);

Adafruit_MLX90614 mlx = Adafruit_MLX90614();

char      ATcmd[BUF_LEN];
char      ATbuffer[BUF_LEN];
char      ATdata[BUF_LEN_DATA];
#define BuzzerPin             3
int   Buzzer = OFF;

//Threshold Value Preset
#define BodyTemp_LOW   35
#define BodyTemp_HIGH   37.2//低于BodyTemp_LOW时发送请靠近一点，高于BodyTemp_HIGH时报警，此阈值调试时设定

double Frequency = 5;

//Status Pool
double RoomTemp;
double RoomMois;
double BodyTemp;
int   LightDetector = 0;
int   eCO2 = 0;
int   TVOC = 0;
int   Door = OFF;
int   norCount = 0;
int   bodyTempStatus = 1;
int   count = 0;
int   simCount = 5;
int   Start = 0;
String data;

void setup() {
//  Serial Initial
  Serial.begin(115200);
  Serial3.begin(115200);
  
  //Pin Initial
  Pin_init();
  BEEP(1);

  stepper.setSpeed(150);

  mlx.begin();  
  
  //Cloud Initial
  while(1)
  {
    if(!WiFi_init())continue;
    BEEP(2);
    if(!Ali_connect())continue;
    break;
  }
  BEEP(3);
}

void loop() {
  //sensorCollect
  RoomTemp = getEnvTemprature();
  delay(500);
  RoomMois = getEnvMoisture();
  delay(500);
//  eCO2 = getEnvCO2();
//  delay(500);
  TVOC = getEnvPM2_5();
  delay(500);
  LightDetector = getEnvBrightness();
  delay(500);

  String inString;
  delay(10);
  if(Serial3.available()>0){
    delay(1000);
    inString = Serial3.readString();
    if(inString!="" && inString.length()>50){
      data = inString;
      Serial.println(data);
      delay(10);
      Start = JsonParse(data);
    }
  }
  
  if(Start){
    BEEP(2);
    //模拟测体温
    BodyTemp = getRandomTemprature(false);
    count++;
  
    int count_2 = 0;
    if(count > simCount){
      while(count_2 < 5){
        BodyTemp = getRandomTemprature(true);
        if(getEnvBrightness()>500){
          BodyTemp = 38;
          Serial.println("warning");
        }
        judgeTemp();
        if(norCount == 3){
          BEEP(3);
          norCount = 0;
          UploadBodyTemp();
          doorMotion();
          setLED(GPin, false);
          setLED(RPin, false);
          setLED(BPin, false);
        }
        UploadEnv();
        count_2++;
        }
        simCount = rand() % 5 + 3;
        count = 0;
    }else{
      if(getEnvBrightness()>500){
        BodyTemp = 38;
        Serial.println("warning");
      }
      judgeTemp();
      if(norCount == 3){
        BEEP(3);
        norCount = 0;
        UploadBodyTemp();
        doorMotion();
        setLED(GPin, false);
        setLED(RPin, false);
        setLED(BPin, false);
      }
      UploadEnv();
    }
    
    //真实测体温
  //  BodyTemp = getBodyTemprature();
  //  judgeTemp();
  //  if(norCount == 3){
  //    setLED(GPin, true);
  //    setLED(RPin, false);
  //    setLED(BPin, false);
  //    norCount = 0;
  //    UploadBodyTemp();
  //    doorMotion();
  //  }
  //  UploadEnv();
  }else{
    setLED(GPin, false);
    setLED(RPin, false);
    setLED(BPin, false);
  }
}

bool Ali_connect()
{
  bool flag;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_AUTH,DeviceName,ProductKey,password);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_CID,clientIDstr,timestamp);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_SOCK,ProductKey);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MQTT_AUTOSTART_OFF,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MQTT_ALIVE,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MQTT_START,AT_MQTT_START_SUCC,20);
  if(!flag)return false;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_SET,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  //flag = check_send_cmd(AT_MQTT_UNSUB,AT_OK,DEFAULT_TIMEOUT);
  //if(!flag)return false;
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_SUB,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_MQTT_SUB_SUCC,DEFAULT_TIMEOUT);
  if(!flag)BEEP(4);
  return flag;
}

bool WiFi_init()
{
  bool flag;

  flag = check_send_cmd(AT,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;
  
  flag = check_send_cmd(AT_REBOOT,AT_OK,20);
  if(!flag)return false;
  delay(5000);

  flag = check_send_cmd(AT_ECHO_OFF,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;

  flag = check_send_cmd(AT_MSG_ON,AT_OK,DEFAULT_TIMEOUT);
  if(!flag)return false;
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_WIFI_START,wifi_ssid,wifi_psw);
  flag = check_send_cmd(ATcmd,AT_WIFI_START_SUCC,20);
  return flag;
}

bool check_send_cmd(const char* cmd,const char* resp,unsigned int timeout)
{
  int i = 0;
  unsigned long timeStart;
  timeStart = millis();
  cleanBuffer(ATbuffer,BUF_LEN);
  Serial3.print(cmd);
  Serial3.flush();
  while(1)
  {
    while(Serial3.available())
    {
      ATbuffer[i++] = Serial3.read();
      if(i >= BUF_LEN)i = 0;
    }
    if(NULL != strstr(ATbuffer,resp))break;
    if((unsigned long)(millis() - timeStart > timeout * 1000)) break;
  }
  
  if(NULL != strstr(ATbuffer,resp))return true;
  return false;
}

void cleanBuffer(char *buf,int len)
{
  for(int i = 0;i < len;i++)
  {
    buf[i] = '\0';
  } 
}


void Pin_init()
{
  pinMode(RPin,OUTPUT);
  digitalWrite(RPin,LOW);
  pinMode(GPin,OUTPUT);
  digitalWrite(GPin,LOW);
  pinMode(BPin,OUTPUT);
  digitalWrite(BPin,LOW);
}

void BEEP(int b_time)
{
  for(int i = 1;i <= b_time;i++)
  { 
    digitalWrite(BuzzerPin,HIGH);
    delay(100);
    digitalWrite(BuzzerPin,LOW);
    delay(100);
  }
}

void Buzzer_mute()
{
  Buzzer_OFF;
  Buzzer = MUTE;
}

int JsonParse(String data){
   StaticJsonBuffer<200> jsonBuffer;

    int commaPosition;  
    commaPosition = data.indexOf('{');
    data= data.substring(commaPosition, data.length());
    //Serial3.println(data);

    char datapointer[data.length()+1];
    strcpy(datapointer,data.c_str());
    
    JsonObject& root = jsonBuffer.parseObject(data);
    const char* method  = root["method"];
    const char* id      = root["id"];
    int         isStart  = root["params"]["Switch"];

//     Print values.
    Serial.println("==============Start================");
    Serial.println(method);
    Serial.println(id);
    Serial.println(isStart);
    
    return isStart;  
}

/*
 待修改硬件接口
*/

int getEnvPM2_5() {
  int EnvPM2_5=analogRead(A0);
  return EnvPM2_5;
}

double getEnvTemprature() {
  int chk = DHT11.read(DHT11PIN);

//  Serial.print("Read sensor: ");
  switch (chk)
  {
    case DHTLIB_OK: 
//                Serial.println("OK"); 
                break;
    case DHTLIB_ERROR_CHECKSUM: 
                Serial.println("Checksum error"); 
                break;
    case DHTLIB_ERROR_TIMEOUT: 
                Serial.println("Time out error"); 
                break;
    default: 
                Serial.println("Unknown error"); 
                break;
  }
  return DHT11.temperature;
}

double getEnvMoisture() {
  int chk = DHT11.read(DHT11PIN);

//  Serial.print("Read sensor: ");
  switch (chk)
  {
    case DHTLIB_OK: 
//                Serial.println("OK"); 
                break;
    case DHTLIB_ERROR_CHECKSUM: 
                Serial.println("Checksum error"); 
                break;
    case DHTLIB_ERROR_TIMEOUT: 
                Serial.println("Time out error"); 
                break;
    default: 
                Serial.println("Unknown error"); 
                break;
  }
  return DHT11.humidity;
}

int getEnvBrightness() {
  int EnvBrightness=analogRead(A2);
  return EnvBrightness;
}

double getBodyTemprature() {
  // 返回摄氏度值
  return mlx.readObjectTempC();
}

double getRandomTemprature(bool isok) {
  // 返回随机摄氏度值，用来糊弄，isok=true代表是合格体温，否则返回一个环境温度值
  if(!isok){
    return random(2200, 2800)*1.0/100;
  }
  else{
    return random(3580, 3650)*1.0/100;
  }
}

void setLED(int LEDNum, bool statu) {
  Serial.print("led set:");
  Serial.println(LEDNum, statu);
  if(statu) {
    digitalWrite(LEDNum,HIGH);
  }
  else{
    digitalWrite(LEDNum,LOW); 
  }
  return;
}

void startMotor(double rounds) {
  Serial.print("motor running");
  Serial.println(rounds);
  stepper.step(2048*rounds); //4步模式下旋转一周用2048 步。
  return;
}

void setMotorSpeed(int target) {
  stepper.setSpeed(target);
  return;
}

/*
 涉及过程函数请写在下面
*/

void judgeTemp(){
  if(BodyTemp < BodyTemp_LOW)
  {
    bodyTempStatus = 0;
  }
  if(BodyTemp > BodyTemp_HIGH)
  {
    bodyTempStatus = 1;
  }
  if((BodyTemp < BodyTemp_HIGH) && (BodyTemp > BodyTemp_LOW))
  {
    bodyTempStatus = 2;
  }
  
  switch(bodyTempStatus){
    case 0: 
    setLED(GPin, false);
    setLED(RPin, false);
    setLED(BPin, true);
    norCount = 0;
    UploadBodyTemp();
    Serial.println("请您靠近一点！");
    break;
    case 1:
    setLED(GPin, false);
    setLED(RPin, true);
    setLED(BPin, false);
    norCount = 0;
    UploadBodyTemp();
    Serial.println("case 1");
    delay(3000);
    break;
    case 2:
    setLED(GPin, true);
    setLED(RPin, false);
    setLED(BPin, false);
    Serial.println("case 2");
    norCount++;
    UploadBodyTemp();
  }
  return ;
}

void doorMotion(){
  //这里的延时即为绿灯亮的延时,下面delay为电机开关门转动时的延时
  Door = ON;
  Serial.println("door open");
  UploadDoorStatus();
  startMotor(1);
  delay(500);
  Serial.println("door close");
  startMotor(-1);
  Door = OFF;
  UploadDoorStatus();
  delay(500);
}

bool UploadEnv()
{
  bool flag;
  int intel_temp,fracl_temp;
  int intel_mois,fracl_mois;
  int len;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_SET,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  
  cleanBuffer(ATdata,BUF_LEN_DATA);
  intel_temp = (int)(RoomTemp);
  fracl_temp = (int)((RoomTemp - intel_temp) * 10);

  intel_mois = (int)(RoomMois);
  fracl_mois = (int)((RoomMois - intel_mois) * 10);
  
  len = snprintf(ATdata,BUF_LEN_DATA,JSON_ENV_DATA_PACK,intel_temp,fracl_temp,intel_mois,fracl_mois,LightDetector,TVOC,eCO2);
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
  flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
  if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
  if(flag) Serial.println("==============PUB ENV SUCC===============");
  
//  Serial.println(ATdata);
//  cleanBuffer(ATdata,BUF_LEN_DATA);
  return flag;
}

bool UploadBodyTemp()
{
  bool flag;
  int intel,fracl;
  int len;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_SET,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  
  cleanBuffer(ATdata,BUF_LEN_DATA);

  intel = (int)(BodyTemp);
  fracl = (int)((BodyTemp - intel) * 100);
  
  len = snprintf(ATdata,BUF_LEN_DATA,JSON_BODYTEMP_DATA_PACK,intel,fracl,bodyTempStatus);
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
  flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
  if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
  if(flag) Serial.println("==============PUB BODYTEMP SUCC===============");
  
//  Serial.println(ATdata);
//  cleanBuffer(ATdata,BUF_LEN_DATA);
  return flag;
}

bool UploadDoorStatus()
{
  bool flag;
  int inte1,frac1;
  int len;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_SET,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
  
  cleanBuffer(ATdata,BUF_LEN_DATA);
  
  len = snprintf(ATdata,BUF_LEN_DATA,JSON_DOORSTA_DATA_PACK,Door);
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
  flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
  if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
  if(flag) Serial.println("==============PUB DOORSTATUS SUCC===============");
  
//  Serial.println(ATdata);
//  cleanBuffer(ATdata,BUF_LEN_DATA);
  return flag;
}
