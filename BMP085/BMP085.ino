#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <BH1750FVI.h>
#include "aes.h"

AES_KEY key;   //加密

//传感器
BH1750FVI LightSensor;
Adafruit_BMP085 bmp;

//lora
#define M0 PA0
#define M1 PA1
#define LED PA8

unsigned char Node = 1; //节点编号
//unsigned char cs[6] = {0xC0, 0x00, 0x02, 0x1A, 0x04, 0xC7};//2
unsigned char cs[6] = {0xC0, 0x00, 0x01, 0x1A, 0x04, 0xC7};//1
unsigned char dz[3] =  {0x01, 0x01, 0x04};    //目标地址+信道

//加密
unsigned char temp[16]; //密文
unsigned char text[16]; //明文
//解密
unsigned char encrypted[16]; //密文                           
unsigned char decrypted[16] = {0}; //明文

const unsigned char master_key[16] = "1j8skl5031khsu";   //公共密钥
//掩码
const char KEY_k[16] = {'b','v','E','q','x','M','F','V','C','f','w','s','a','y','u','k',};
const char KEY_e[16] = {'W','L','t','z','T','K','G','n','Z','l','o','d','g','Y','B','I',};
const char KEY_y[16] = {'r','c','N','p','J','h','e','A','j','P','i','U','S','R','m','O',};
String DK_data;
String EK_data;
String data="";

int WD,YL,HB;
//从接收缓冲区中删除所有字节
void clearBuffer() {
  while (Serial2.available())
    Serial2.read();
}
void ReadData(){
  int number = 0;
  if (Serial2.available()>0) {
    delay(100);
    number = Serial2.available();
    while(number--) {
        data+=char(Serial2.read());
        delay(2);
      }  
  } 
  //Serial.println(data);
    if(data.length() != 32){
      data = "";
    }  
    if(data.length() == 32){
        Str_con(data,encrypted);
      AES_set_decrypt_key(master_key, 128, &key);
      AES_decrypt(encrypted, decrypted, &key);
      EK_data = String(decrypted[0]);
    } 
    if(EK_data == "1")
      digitalWrite(LED, HIGH);
    if(EK_data == "0") 
      digitalWrite(LED, LOW); 
    clearBuffer();    
}
void ReadPin(){
  if(digitalRead(LED) == 1){
    text[10] = (unsigned int)2;
  }else if(digitalRead(LED) == 0){
    text[10] = (unsigned int)1;
  }
}
void DK(){
  DK_data = "";
  delay(100);
  WD = (int)bmp.readTemperature();
  delay(100);
  YL = (int)bmp.readPressure();
  delay(100);
  HB = (int)bmp.readAltitude(101500);
  delay(100);
  uint16_t Lux = LightSensor.GetLightIntensity();// Get Lux value
  delay(2000);
  //Serial.println();
  text[0]  = (unsigned int)Node;
  text[1]  = (unsigned int)WD;
  
  text[2]  = (unsigned int)(YL/10000)%10+((YL/100000)%10)*10;
  text[3]  = (unsigned int)(YL/100)%10 +((YL/1000)%10)*10;
  text[4]  = (unsigned int)YL%10 + ((YL/10)%10)*10;
  
  text[5]  = (unsigned int)(HB/100)%10 +((HB/1000)%10)*10;
  text[6]  = (unsigned int)HB%10 + ((HB/10)%10)*10;
  
  text[7]  = (unsigned int)(Lux/10000)%10+((Lux/100000)%10)*10;
  text[8]  = (unsigned int)(Lux/100)%10 +((Lux/1000)%10)*10;
  text[9]  = (unsigned int)Lux%10 + ((Lux/10)%10)*10;
  
  
  text[11] = (unsigned int)0;
  text[12] = (unsigned int)0;
  text[13] = (unsigned int)0;
  text[14] = (unsigned int)0;
  text[15] = (unsigned int)0;

//  for (int i = 0; i < 16; i++) {
//    Serial.print(text[i],HEX);
//    Serial.print(" ");
//  }
//  Serial.println();

  AES_set_encrypt_key(master_key, 128, &key);
  AES_encrypt(text, temp, &key);

//for (int i = 0; i < 16; i++) {
//  Serial.print(temp[i],HEX);
//  Serial.print(" ");
//}
//  Serial.println();
  
  //获取字符串 HEX -> str
  unsigned int str_a,str_b,str_HEX[32];
  for (int i = 0; i < 16; i++) {
    str_a = temp[i]/16;
    str_b = temp[i] - str_a * 16;
    str_HEX[2*i] = str_a;
    str_HEX[2*i+1] = str_b;
  }
  char str_mask_temp;
  int str_mask_rand;
  for(int i = 0; i < 32; i++){
    str_mask_rand = rand()%3;
    if(0 == str_mask_rand)
      str_mask_temp = KEY_k[str_HEX[i]];
    else if(1 == str_mask_rand)
      str_mask_temp = KEY_e[str_HEX[i]];
    else
      str_mask_temp = KEY_y[str_HEX[i]];
    //printf("%c",str_mask_temp); 
    DK_data += str_mask_temp;
  }   
}
void Str_con(const String Str_Buf,unsigned char * encrypted){
  int i,j;
  char Str_Hex_Buf[32];
  char Str_Hex_Temp[32];                                   //得到的密文HEX字符串版本
  const char KEY_Source[16] = {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F};
    
  for(i = 0; i < 32; i++)
    for(j = 0; j < 16; j++)
      if(Str_Buf[i] == KEY_k[j] || Str_Buf[i] == KEY_e[j] || Str_Buf[i] == KEY_y[j] ){
        Str_Hex_Temp[i] = KEY_Source[j];
        break;
      }
       
  for(i = 0; i < 16; i++)
    encrypted[i] = Str_Hex_Temp[2*i]*16+Str_Hex_Temp[2*i+1]; 
}
void setup() {
  Serial2.begin(9600);
  //Serial.begin(9600);
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  delay(1000);
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  delay(1000);
  Serial2.write(cs,6);
  delay(1000);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);

  bmp.begin();
  LightSensor.begin();
  LightSensor.SetAddress(Device_Address_H);//Address 0x5C
  LightSensor.SetMode(Continuous_H_resolution_Mode);
  text[10] = (unsigned int)0;
} 
void loop() {
  ReadPin();
  DK();
  Serial2.write(dz,3);
  delay(3);
  Serial2.print(DK_data);
  //Serial.print(DK_data); 
  ReadData();
  delay(10000);
}
