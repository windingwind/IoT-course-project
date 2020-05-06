#define SERIAL_RX_BUFFER_SIZE 512 //修改串口发送缓冲区大小为512
#define SERIAL_TX_BUFFER_SIZE 512 //修改串口接收缓冲区大小为512

#include <Wire.h>
#include <ArduinoJson.h>

//User Modified Part
#define wifi_ssid     "wxy"    
#define wifi_psw      "wxy12345678"     
#define clientIDstr   "123456789abc"
#define timestamp     "996"
#define ProductKey    "a1v0RymPkXS"
#define DeviceName    "test001"
#define DeviceSecret  "5BYzFuKeS6zTJBadCIpWrWAVS5ZW6dk0"
#define password      "9B0512B9655ACCCA9B20EC209881F33304467011"



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
#define JSON_DATA_PACK        "{\"id\":\"100\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"RoomTemp\":%d.%02d,\"AC\":%d,\"Fan\":%d,\"Buzzer\":%d,\"GasDetector\":%d}}\r"
#define JSON_DATA_PACK_2      "{\"id\":\"110\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"PhotoResistors\":%d}}\r"
#define JSON_DATA_PACK_3      "{\"id\":\"110\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":{\"LightSwitch\":%d}}\r"
#define JSON_DATA_PACK_ALARM  "{\"id\":\"110\",\"version\":\"1.0\",\"method\":\"thing.event.GasAlarm.post\",\"params\":{\"GasDetector\":%d}}\r"
#define AT_MQTT_PUB_DATA_SUCC "+MQTTEVENT:PUBLISH,SUCCESS"
#define AT_MQTT_UNSUB         "AT+MQTTUNSUB=2\r"
#define AT_MQTT_SUB           "AT+MQTTSUB=1,/sys/%s/%s/thing/service/property/set,1\r"
#define AT_MQTT_SUB_SUCC      "+MQTTEVENT:1,SUBSCRIBE,SUCCESS"

#define AT_BUZZER_MUTE           "\"Buzzer\":2"

#define Light3Pin             13

int red=0,green=0,blue=0;
#define RPin 7
#define GPin 8
#define BPin 9

bool LightSwitch = false;
#define LPin 7

#define DEFAULT_TIMEOUT       10   //seconds
#define BUF_LEN               100
#define BUF_LEN_DATA          190

char      ATcmd[BUF_LEN];
char      ATbuffer[BUF_LEN];
char      ATdata[BUF_LEN_DATA];
#define BuzzerPin             3
int   Buzzer = OFF;

double Frequency = 5;

void setup() {
  //Serial Initial
  //Serial.begin(115200);
  Serial3.begin(115200);
  
  //Pin Initial
  Pin_init();
  BEEP(1);
  
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
  //Upload
 // Upload();
//  int PhotoResistors=analogRead(A2);
//  int len;
//  bool flag;
//
//  cleanBuffer(ATdata,BUF_LEN_DATA);
//  len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_2,PhotoResistors);
//  Serial.println(ATdata);
//
//  cleanBuffer(ATcmd,BUF_LEN);
//  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
//  flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
//  if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
//  delay(3000);

  if(Serial3.available()>0){
    delay(100);
    String inString=Serial3.readString();
    if (inString!=""){
      BEEP(1);
      JsonObject& retJSON = JSONParse(inString);
      delay(200);
      // Deal With Different Operations
//      if(retJSON.containsKey("Frequency")){
//        if(retJSON["Frequency"]>=0){
//          Frequency = retJSON["Frequency"];
//        }
//        Serial.print("Frequency Set to: ");
//        Serial.println(Frequency);
//      }
//      if(retJSON.containsKey("ColorGreen")){
//        if(retJSON["ColorGreen"]>=0 && retJSON["ColorGreen"]<=255){
//          setColor(GPin, retJSON["ColorGreen"], green);
//          //green = retJSON["ColorGreen"]
//        }
//      }
//      if(retJSON.containsKey("ColorBlue")){
//        if(retJSON["ColorBlue"]>=0 && retJSON["ColorBlue"]<=255){
//          setColor(BPin, retJSON["ColorBlue"], blue);
//         // blue = retJSON["ColorBlue"]
//        }
//      }
//      if(retJSON.containsKey("ColorRed")){
//        if(retJSON["ColorRed"]>=0 && retJSON["ColorRed"]<=255){
//          setColor(RPin, retJSON["ColorRed"], red);
//          //red = retJSON["ColorRed"]
//        }
//      }
      if(retJSON.containsKey("LightSwitch")){
        if(retJSON["LightSwitch"]){
            digitalWrite(RPin,HIGH);
        }
        else{
            digitalWrite(RPin,LOW);
        }
        
        LightSwitch = retJSON["LightSwitch"];
        int len;
        bool flag;
        
        cleanBuffer(ATdata,BUF_LEN_DATA);
        len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_3,LightSwitch);
        Serial.println(ATdata);
        
        cleanBuffer(ATcmd,BUF_LEN);
        snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
        flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
        if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
//        delay(3000);
      }
    }
  }
//
////  Serial.println(Serial3.available());
//  while(Serial3.available()==0) {
//    int delay_seconds=1.0/Frequency*1000;
//    delay(delay_seconds/2);
//    digitalWrite(13,HIGH);
//    delay(delay_seconds/2);
//    digitalWrite(13,LOW);
//  }
  
  //MsgReceive
//  if(check_send_cmd(AT,AT_BUZZER_MUTE,DEFAULT_TIMEOUT))Buzzer_mute();
}

/* bool Upload()
{
  bool flag;
  int inte1,frac1;
  int len;

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_SET,ProductKey,DeviceName);
  flag = check_send_cmd(ATcmd,AT_OK,DEFAULT_TIMEOUT);
   
  
  cleanBuffer(ATdata,BUF_LEN_DATA);

  inte1 = (int)(RoomTemp);
  frac1 = (RoomTemp - inte1) * 100;
  
  len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK,inte1,frac1,AC,Fan,Buzzer,GasDetector);
  
  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
  flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
  if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);
  
  
//  delay(500);
  
  cleanBuffer(ATdata,BUF_LEN_DATA);
  len = snprintf(ATdata,BUF_LEN_DATA,JSON_DATA_PACK_2,LightDetector,Curtain,Light,SoilHumi,Pump,eCO2,TVOC);

  cleanBuffer(ATcmd,BUF_LEN);
  snprintf(ATcmd,BUF_LEN,AT_MQTT_PUB_DATA,len-1);
  flag = check_send_cmd(ATcmd,">",DEFAULT_TIMEOUT);
  if(flag) flag = check_send_cmd(ATdata,AT_MQTT_PUB_DATA_SUCC,20);

  return flag;
}
*/

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
  pinMode(LEDPin1,OUTPUT);
  digitalWrite(LEDPin1,LOW);
  pinMode(LEDPin2,OUTPUT);
  digitalWrite(LEDPin2,LOW);
  pinMode(LEDPin3,OUTPUT);
  digitalWrite(LEDPin3,LOW);
//  pinMode(ACPin,OUTPUT);
//  digitalWrite(ACPin,LOW);
//  pinMode(BuzzerPin,OUTPUT);
//  digitalWrite(BuzzerPin,LOW);
//  pinMode(PumpPin,OUTPUT);
//  digitalWrite(PumpPin,LOW);
//  pinMode(CurtainOpenPin,OUTPUT);
//  digitalWrite(CurtainOpenPin,LOW);
//  pinMode(CurtainClosePin,OUTPUT);
//  digitalWrite(CurtainClosePin,LOW);
//  pinMode(Light1Pin,OUTPUT);
//  digitalWrite(Light1Pin,LOW);
//  pinMode(Light2Pin,OUTPUT);
//  digitalWrite(Light2Pin,LOW);
//  pinMode(Light3Pin,OUTPUT);
//  digitalWrite(Light3Pin,LOW);
//
//  pinMode(RPin,OUTPUT);
//  digitalWrite(RPin,LOW);
//  pinMode(GPin,OUTPUT);
//  digitalWrite(GPin,LOW);
//  pinMode(BPin,OUTPUT);
//  pinMode(FanPin,OUTPUT);
//  digitalWrite(FanPin,LOW);
//  Curtain_ON();
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

JsonObject& JSONParse(String raw)
{
  // Remove Raw Data Head
  String JSONRaw = raw.substring(raw.indexOf('{'), raw.length());
  Serial.println("==============Parse Start================");
  Serial.println(JSONRaw);

  StaticJsonBuffer<200> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(JSONRaw);
  return root["params"];
}

void setColor(int ModifyPin, int target, int& origin)
{
  int t = origin-(origin - target)%30;
  int step_v = (origin - target)/30;

  analogWrite(ModifyPin, t);
  delay(30);
  while(t!=target){
    t-=step_v;
    analogWrite(ModifyPin, t);
    delay(30);
  }
  origin=target;
}

/*
 待修改硬件接口
*/

double getEnvPM2_5() {
  return 10;
}

double getEnvTemprature() {
  return 11;
}

double getEnvMoisture() {
  return 12;
}

double getEnvCO2() {
  return 13;
}

double getEnvBrightness() {
  return 15;
}

double getBodyTemprature() {
  return 14;
}

void setLED(int LEDNum, bool statu) {
  if(statu) {
    digitalWrite(LEDNum,HIGH);
  }
  else{
    digitalWrite(LEDNum,LOW); 
  }
  return;
}

/*
 涉及过程函数请写在下面
*/
