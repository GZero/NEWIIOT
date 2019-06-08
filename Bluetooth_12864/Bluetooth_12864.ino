#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
#include "aes.h"

#define SERVER      "120.79.185.233"
#define PORT        "8088"
#define INTERVAL    5000   //间隔
String TEMP = "";   //要发送的数据
char cmd[64];       //wifi指令中间变量
unsigned long lastmillis;   //超时

const unsigned char master_key[16] = "1j8skl5031khsu"; //公共密钥
const char KEY_k[16] =      {'b','v','E','q','x','M','F','V','C','f','w','s','a','y','u','k',};
const char KEY_e[16] =      {'W','L','t','z','T','K','G','n','Z','l','o','d','g','Y','B','I',};
const char KEY_y[16] =      {'r','c','N','p','J','h','e','A','j','P','i','U','S','R','m','O',};
//加密
unsigned char temp[16]; //密文
unsigned char text[16]; //明文
//解密
unsigned char encrypted[16];                               
unsigned char decrypted[16] = {0};
String DK_data = "";   //接收的原始数据
String Flag = "";
String NODE = "";
AES_KEY key;

String ON = "";   //要发送的数据
String OFF = "";   //要发送的数据
String FLAG_NC = "0";

#define KEY_1 PB3
#define KEY_2 PB4

#define M0 PA0
#define M1 PA1
unsigned char cs[6] = {0xC0, 0x01, 0x01, 0x1A, 0x04, 0xC7};  //本机参数
unsigned char dz1[3] =  {0x00, 0x01, 0x04};    //目标地址+信道
unsigned char dz2[3] =  {0x00, 0x02, 0x04};    //目标地址+信道
  
//     __Signal__Maple_//__OLED 128x64___
#define OLED_DC   PB6   //   D/C   DC
#define OLED_RST  PB7   //   RST   RES
#define OLED_MOSI PB8   //   SDA   D1
#define OLED_CLK  PB9   //   SCL   D0
#define OLED_CS   PB5   //   ---   x Not Connected
Adafruit_SSD1306 OLED(OLED_MOSI, OLED_CLK, OLED_DC, OLED_RST, OLED_CS);

void KEY_FUN_1(){
  FLAG_NC = "1";
}
void KEY_FUN_2(){
  FLAG_NC = "2";
}
void DK(){
  DK_data = "";
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
void HariChord(String Data){
  //OLED.clearDisplay();
  OLED.setTextSize(1);             //设置字体大小  
  OLED.setTextColor(WHITE);        //设置字体颜色白色  
  OLED.setCursor(0,2);             //设置字体的起始位置 
  OLED.print(DK_data);   //输出字符并换行 
  Serial.print(DK_data);
  OLED.setCursor(0,20);
  OLED.print("-----------------");   //输出字符并换行 
  //OLED.print("-------------"+Flag);
  if(Flag == "1"){
    OLED.setCursor(106,15);
    OLED.print("OFF");
  }else if(Flag == "2"){
    OLED.setCursor(106,15);
    OLED.print("ON");
  }else{
    OLED.setCursor(106,15);
    OLED.print("---");
  }
  OLED.setCursor(0,28);
  OLED.print(Data);
  OLED.setCursor(0,40);
  OLED.print("N  C    Pa    M   Lux");
  OLED.display(); 
 }
//从接收缓冲区中删除所有字节
void clearBuffer() {
  while (Serial1.available())
    Serial1.read();
}
//发送AT命令
void sendCommand(String buff) {
  Serial1.println(buff);
  Serial1.flush();
}
//等待特定输入字符串，直到超时结束
bool waitForString(char* input, int length, unsigned int timeout) {

  unsigned long end_time = millis() + timeout;
  char current_byte = 0;
  int index = 0;

   while (end_time >= millis()) {
    
      if(Serial1.available()) {
        
        //Read one byte from serial port//从串口读取一个字节
        current_byte = Serial1.read();
//        if (_DEBUG_) Serial.print(current_byte);
        if (current_byte != -1) {
          //Search one character at a time //一次搜索一个字符
          if (current_byte == input[index]) {
            index++;
            
            //Found the string //找到了字符串
            if (index == length) {              
              return true;
            }
          //Restart position of character to look for //重新启动要查找的字符位置
          } else {
            index = 0;
          }
        }
      }
  }  
  //Timed out  //超时
  return false;
}
//获取IP地址
int getIpAddress(char *buf, int szbuf, int timeout) {
  int len=0;
  int pos=0;
  char line[128];
    
  long int time = millis();

  Serial1.print("AT+CIPSTA?\r\n");

  while( (time+timeout) > millis()) {
    while(Serial1.available())  {
      char c = Serial1.read(); //读下一个字符。
      if (c == 0x0d) {
          
      } else if (c == 0x0a) {
//        if (_DEBUG_) {
//          Serial.print("Read=[");
//          Serial.print(line);
//          Serial.println("]");
//        }
        int offset;
        for(offset=0;offset<pos;offset++) {
          if(line[offset] == '+') break;
        }
        if (strncmp(&line[offset],"+CIPSTA:ip:",11) == 0) {
          strcpy(buf,&line[12+offset]);
          len = strlen(buf) - 1;
          buf[len] = 0;
        }
        if (strcmp(line,"OK") == 0) return len;
        pos=0;
        line[pos]=0;
      } else {
        line[pos++]=c;
        line[pos]=0;
      }
    }  
  }
  return len;
}
//去掉掩码
void Str_con(const String Str_Buf,unsigned char * encrypted){
  int i,j;
  char Str_Hex_Buf[32];
  char Str_Hex_Temp[32];                                   //得到的密文HEX字符串版本
  /* 掩码 */
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
void setup()   {
  Serial.begin(9600);  //TX=PA9,RX=PA10
  Serial2.begin(9600);  //TX=PA9,RX=PA10
  Serial1.begin(115200);
  OLED.begin(SSD1306_SWITCHCAPVCC);
  OLED.display();                // Adafruit Splash
  OLED.clearDisplay();           // Clear the buffer.
  OLED.setTextSize(1);             //设置字体大小  
  OLED.setTextColor(WHITE);        //设置字体颜色白色  
  OLED.setCursor(32,0);             //设置字体的起始位置 
  OLED.print("Hello GZEU!");   //输出字符并换行
  OLED.display(); 
  
  //Debug(">> INIT UART 1 2");
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT); 
  pinMode(KEY_1,INPUT_PULLDOWN);
  pinMode(KEY_2,INPUT_PULLDOWN);
  digitalWrite(KEY_1, HIGH);
  digitalWrite(KEY_2, HIGH);
  attachInterrupt(KEY_1, KEY_FUN_1, FALLING);//RISING 0-1  FALLING 1-0  CHANGE 0-1/1-0
  attachInterrupt(KEY_2, KEY_FUN_2, FALLING);

  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);
  delay(1000);
  digitalWrite(M0, HIGH);
  digitalWrite(M1, HIGH);
  delay(1000);
  Serial2.write(cs,6);
  //Debug(">> INIT LORA");
  delay(1000);
  digitalWrite(M0, LOW);
  digitalWrite(M1, LOW);  
  OLED.display(); 
  delay(1000);
  text[0] = (unsigned int)0;
  DK();
  OFF = DK_data;
  //Serial.println(OFF);  //TX=PA9,RX=PA10
  text[0] = (unsigned int)1;
  DK();
  ON = DK_data;
  //Serial.println(ON);  //TX=PA9,RX=PA10
  DK_data = "";
}
void loop() {
  short sz_smsg;   //wifi发送数据长度
  String data="";
  while (Serial2.available()){
      data+=char(Serial2.read());
      delay(2);
    } 
    if(data.length() != 32){
      data = "";
    }
    if(data != ""){
      DK_data = data;
      OLED.clearDisplay();
      Str_con(data,encrypted);
    AES_set_decrypt_key(master_key, 128, &key);
    AES_decrypt(encrypted, decrypted, &key);
    NODE = String(decrypted[0]);
    Flag = String(decrypted[10]);
    data = String(decrypted[0])+" "+String(decrypted[1])+" "+String(decrypted[2]*10000+decrypted[3]*100+decrypted[4])+" "+String(decrypted[5]*100+decrypted[6])+" "+String(decrypted[7]*10000+decrypted[8]*100+decrypted[9]);
    }
  HariChord(data);
if(DK_data.length() == 32){
  if (((signed long)(millis() - lastmillis)) > 0) {
      lastmillis = millis() + INTERVAL;
      sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%s", SERVER,PORT);   //连接服务器
      sendCommand(cmd);
      if (!waitForString("OK", 2, 10000)) {
        //Debug("AT+CIPSTART Fail");   //连接失败
      }
      clearBuffer();
      //发送消息
      TEMP = "";
      TEMP = String(decrypted[0])+"|"+String(decrypted[1])+"|"+String(decrypted[2]*10000+decrypted[3]*100+decrypted[4])+"|"+String(decrypted[5]*100+decrypted[6])+"|"+String(decrypted[7]*10000+decrypted[8]*100+decrypted[9]);
      sz_smsg= TEMP.length();
      //Debug("AT+CIPSEND BUFF = " + String(sz_smsg));   //数据长度
      //HariChord(TEMP);
      delay(500);
      sprintf(cmd,"AT+CIPSEND=%d",sz_smsg);   //发送消息长度
      sendCommand(cmd);
      if (!waitForString(">", 1, 1000)) {
        //Debug("AT+CIPSEND Fail");       //发送失败‘
        delay(500);
      }
      //发送数据
      sendCommand(TEMP);
      if (!waitForString("SEND OK", 7, 1000)) {
         //Debug("AT+CIPSEND Fail");
         delay(500);
      }
      clearBuffer();
      sendCommand("AT+CIPCLOSE");
      //Debug("AT+CIPCLOSE");
      if (!waitForString("OK", 2, 1000)) {
        //Debug("AT+CIPCLOSE Fail");
        delay(500);
      }
      clearBuffer();
    }
}
  DK_data = "";
//  Serial.print(Flag+"b ");
//  Serial.print(NODE+"c ");
//  Serial.println(FLAG_NC);
  if(Flag == "2" && NODE == "1" && FLAG_NC == "1"){  //Flag == "2"代表节点是开
      Serial2.write(dz1,3);
      delay(3);
      Serial2.print(OFF);
      Serial.println(OFF);
      FLAG_NC = "0";
  }
  if(Flag == "1" && NODE == "1" && FLAG_NC == "1"){
      Serial2.write(dz1,3);
      delay(3);
      Serial2.print(ON);
      Serial.println(ON); 
      FLAG_NC = "0";   
  }

  if(Flag == "2" && NODE == "2" && FLAG_NC == "2"){
      Serial2.write(dz2,3);
      delay(3);
      Serial2.print(OFF);
      FLAG_NC = "0";
  }
  if(Flag == "1" && NODE == "2" && FLAG_NC == "2"){
      Serial2.write(dz2,3);
      delay(3);
      Serial2.print(ON);  
      FLAG_NC = "0";  
  }

}
