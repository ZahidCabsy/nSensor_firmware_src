#include <Arduino.h>
#include <Wire.h>
#include <esp32cam.h>

#define full_option 0x24
#define motion_ambient 0x25
#define motion 0x26

#define V_APP   0x1
#define V_Major 0x0
#define V_Minor 0x18

int counter = 0;

HardwareSerial Sender(1);

const int analogPin = 2;

uint8_t macbuff[6];
char sensor_mac[25];
char mac_length[3];
char ver_length[3];
char sensor_version[10];

static auto hiRes   = esp32cam::Resolution::find(800, 600);
static auto semisqr = esp32cam::Resolution::find(1280, 1024);
static auto XGA     = esp32cam::Resolution::find(1024, 768);
static auto HD      = esp32cam::Resolution::find(1280, 720);
static auto UHD     = esp32cam::Resolution::find(1600, 1200);

bool handshake(int timeout)
{
  char RX;
  auto startTime = millis();

  while(1)
  {
    if(Sender.available()) {
      RX = Sender.read();
      if( RX == 0x47 ) {
        Serial.println("G! ");
        return true;
      }
      if ( millis() - startTime > static_cast<unsigned long>(timeout) ) {
        break;
      }
    }
  }
  Serial.printf("Handshake Serial timeOut!!!");
  return false;
}

int get_response(int buff)
{
  char RX;
  char lbuff[20];
  sprintf(lbuff, "%d\0", buff);
  auto startTime = millis();
  Sender.write(lbuff);

  while(1)
  {
    if(Sender.available()) {
      RX = Sender.read();
      if( RX == 0x47 ) {
        return 0;
      }
      if ( millis() - startTime > static_cast<unsigned long>(1500) ) {
        break;
      }
    }
    if ( millis() - startTime > static_cast<unsigned long>(1500) ) {
        break;
    }
  }
  Serial.printf("Handshake Serial timeOut!!!");
  return 1;
}

bool ambient_read() {

  double ambVolt;
  char amb_len[2];
  char ambient[5];
  char RX;

  ambVolt = analogReadMilliVolts(analogPin);
  // Serial.printf("Ambient Voltage : %lf\n", ambVolt);
  double milAmb = (ambVolt / 1000);
  // Serial.printf("milAmb  : %lf\n", milAmb);
  int lux = ( (milAmb / 3) * 1125 );
  // Serial.printf("Ambient  : %i\n", lux);
  sprintf(ambient,"%d", lux);
  int length = strlen(ambient);
  sprintf(amb_len, "%d", length);
  // Serial.printf("Char Ambient  : %s\n", ambient);
  // Serial.printf("Char Amb len  : %s\n", amb_len);
  auto startTime = millis();
  Sender.write(0x41);

  while(1)
  {
    if(Sender.available()) {
      RX = Sender.read();
      if( RX == 0x5A ) { // Z
        Sender.write(ambient);
        return true;
      }
      if( RX == 0x44 ) { // D
          Sender.write(amb_len);
      }
      if ( millis() - startTime > static_cast<unsigned long>(2000) ) {
        break;
      }
    }
    if ( millis() - startTime > static_cast<unsigned long>(2000) ) {
      break;
    }
  }
  Serial.printf("Ambient Read timeOut!!!");
  return false;
}

bool send_version()
{
  char RX;
  auto startTime = millis();
  Sender.write(0x41);
  while(1)
  {
    if(Sender.available()) {
      RX = Sender.read();
      if( RX == 0x5A ) { // Z
        Sender.write(sensor_version);
        return true;
      }
      if( RX == 0x44 ) { // D
          Sender.write(ver_length);
      }
      if ( millis() - startTime > static_cast<unsigned long>(2000) ) {
        break;
      }
    }

    if ( millis() - startTime > static_cast<unsigned long>(2000) ) {
      break;
    }
  }

  Serial.printf("Version Sync timeOut!!!");
  return false;
  
}

bool sync_mac()
{
  char RX;
  auto startTime = millis();
  Sender.write(0x41);
  while(1)
  {
    if(Sender.available()) {
      RX = Sender.read();
      if( RX == 0x5A ) { // Z
        Sender.write(sensor_mac);
        return true;
      }
      if( RX == 0x44 ) { // D
          Sender.write(mac_length);
      }
      if ( millis() - startTime > static_cast<unsigned long>(2000) ) {
        break;
      }
    }

    if ( millis() - startTime > static_cast<unsigned long>(2000) ) {
      break;
    }
  }

  Serial.printf("Mac Sync timeOut!!!");
  return false;
  
}

void getframe()
{
  char buffer_size[20];
  int local_FS = 0;
  int FS_length = 0;
  char FS_buff_len[10];
  // bool rc;

  auto frame = esp32cam::capture();
  if (frame == nullptr) {
    Serial.println("CAPTURE FAIL");
    return;
  }

  local_FS = static_cast<int>(frame->size());

  Serial.printf("CAPTURE OK %dx%d %db\n", frame->getWidth(), frame->getHeight(), local_FS);

  sprintf(buffer_size, "%d\r\n", local_FS);

  FS_length = strlen(buffer_size);

  Serial.printf("FS Length : %d\n", FS_length);

  sprintf(FS_buff_len, "%d\r\n", FS_length);

  Sender.write(FS_buff_len);

  if(handshake(2000) != true) {
    Serial.println("FS_length Handshake Fail");
    return;
  }

  Sender.write(buffer_size);

  if(handshake(2000) != true) {
    Serial.println("buffer_size Handshake Fail");
    return;
  }

  int counter = 1;
  int inc = 4096;
  int remaining = local_FS - inc;
  int rc;

  while(1)
  {
    printf("Start %d : \n", (counter-1));
    rc = get_response(inc);
    if(rc == 0) {
      Sender.write(&frame->data()[counter-1], inc);

      if(remaining == 0) {
        printf("TS");
        break;
      }
      else {
        remaining = remaining - inc;
        printf("remaining : %d\n", remaining);
        if(remaining > 0) {
          inc = 4096;
          counter = counter + inc;
        }
        else {
          inc = 4096 + remaining;
          counter = counter + 4096;
          printf("Final : %d\n", inc);
          remaining = 0;
        }
      }
    }
    else {
      Serial.print("TE\n");
      break;
    }
  }
}

void Scanner ()
{
  Serial.println ();
  Serial.println ("Reading Mac");
  byte count = 0;
  uint8_t a = 94;
  uint8_t v = 154;
  uint8_t byt = 6;
  int length;
  
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);        // Begin I2C transmission Address (i)
    if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);     // PCF8574 7 bit address
      Serial.println (")");
      count++;
    }
  }
  Serial.print ("Found ");      
  Serial.print (count, DEC);        // numbers of devices
  Serial.println (" device(s).");
  Wire.beginTransmission (a);
  Wire.write(v);
  Wire.endTransmission ();
  Wire.requestFrom(a, byt);
  
  if (Wire.available() != 6) {
    Serial.print("Mac Readerror");
  }

  for(int x=0; x<6; x++) {
    macbuff[x] = Wire.read();
    Serial.print (macbuff[x], HEX);
  }

  sprintf(sensor_mac,"%x:%x:%x:%x:%x:%x", macbuff[0], macbuff[1], macbuff[2], macbuff[3], macbuff[4], macbuff[5]);
  length = strlen(sensor_mac);
  sprintf(mac_length, "%i", length);
  Serial.println(sensor_mac);
  Serial.print(" = ");
  Serial.println(mac_length);

  sprintf(sensor_version,"%i.%i.%i", V_APP, V_Major, V_Minor);
  length = strlen(sensor_version);
  sprintf(ver_length, "%i", length);
  Serial.println(sensor_version);

}

void setup()
{
  //Serial.begin(Baud Rate, Data Protocol, Txd pin, Rxd pin);

  Serial.begin(115200); // Terminal

  Sender.begin(2000000, SERIAL_8N1, 13, 12); // RX TX communication box - esp

  Wire.begin (4, 14);   // sda= GPIO_04 /scl= GPIO_14
  {
    using namespace esp32cam;
    Config cfg;
    cfg.setPins(pins::AiThinker);
    // cfg.setResolution(hiRes);
    cfg.setResolution(semisqr);
    cfg.setBufferCount(1);
    cfg.setJpeg(80);
    bool ok = Camera.begin(cfg);
    Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
  }
  Scanner();
}

void loop() 
{
  while (Sender.available())
  {
    char RxdChar = Sender.read();
    Serial.print("RX : ");
    Serial.println(RxdChar);

    if(RxdChar == 0x55)
    { // U
      Serial.println("OK!");
      Sender.write(0x50); // P
      getframe();
    }
    else if(RxdChar == 0x42)
    { //B//66
        sync_mac();
    }
    else if(RxdChar == 0x4C)
    { // L
        ambient_read();
    }
    else if(RxdChar == 0x5E)
    { // ^ sensor type
      Sender.write(full_option);
    }
    else if(RxdChar == 0x5D)
    { // ]
      send_version();
    }
    else if(RxdChar == 0x7E)
    { // ~
  
    }
    else if(RxdChar == 0x7F)
    { // [DEL]
      Serial.println("Change Resolution");
      ESP.restart();
    }
  }
}

  // buffer_size[strlen(buffer_size)] = "\n";
  // server.setContentLength(frame->size());
  // server.send(200, "image/jpeg");
  // WiFiClient client = server.client();
  // frame->writeTo(client);

  // while(1){
  //   if( 0x47 == Sender.read()){
  //     Serial.println("Sokpa");
  //     break;
  //   }
  // }

  // do{
  //     local_FS
  //   }while(x);
  // for ( size_t i = 0; i < m_size; i += os.write(&m_data[i], m_size - i) ) {
  // for ( size_t i = 0; i < frame->size(); i += Sender.write(&frame->data()[i], frame->size() - i) ) {

  // for ( size_t i = 0; i < frame->size(); i += Sender.write(&frame->data()[i], frame->size() - i) ) {
  //       // Sender.write(&frame->data()[i], frame->size() - i);
  // }

  // Sender.write(&frame->data()[0], frame->size())
  // Sender.write(&frame->data()[0], local_FS);

  // for (byte i = 8; i < 120; i++)
  // {
  //   Wire.beginTransmission (i);          // Begin I2C transmission Address (i)
  //   if (Wire.endTransmission () == 0)  // Receive 0 = success (ACK response) 
  //   {
  //     Serial.print ("Found address: ");
  //     Serial.print (i, DEC);
  //     Serial.print (" (0x");
  //     Serial.print (i, HEX);     // PCF8574 7 bit address
  //     Serial.println (")");
  //     count++;
  //   }
  // }

  // Serial.print ("Found ");      
  // Serial.print (count, DEC);        
  // numbers of devices
  // Serial.println (" device(s).");


  // int local_ctr= 0;
  // for ( int i = 0; i < local_FS; i++) {
  //   Sender.write(&frame->data()[i], 1);
  // }
  // uint8_t *data;
  // frame->writeTo(Sender);
  // Sender.write('\r\n\t');

  // while(1){
  //   if( 0x44 == Sender.read()){
  //     Serial.println("Ready ");
  //     break;
  //   }
  // }

  // if( 0x44 == Sender.read()){
  //   Serial.println("Re D!");
  //   return true;
  //   // break;
  // }