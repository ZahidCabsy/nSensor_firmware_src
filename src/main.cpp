#include <Arduino.h>
#include <Wire.h>
#include <esp32cam.h>
#include "nsensor.h"

#define HANDSHAKE_BYTE        0x47


HardwareSerial Sender(1);

static char sensor_mac[25];
static char mac_length[3];
static char ver_length[3];
static char sensor_version[10];

static auto hiRes   = esp32cam::Resolution::find(800, 600);
static auto semisqr = esp32cam::Resolution::find(1280, 1024);
static auto XGA     = esp32cam::Resolution::find(1024, 768);
static auto HD      = esp32cam::Resolution::find(1280, 720);
static auto UHD     = esp32cam::Resolution::find(1600, 1200);

bool handshake(int handshakeTimeout)
{
	char receivedByte;
  	auto startTime = millis();

  	while (true)
  	{
    	if (Sender.available())
    	{
      	receivedByte = Sender.read();
      	if (receivedByte == HANDSHAKE_BYTE)
      	{
        		Serial.println("G! ");
        		break;
      	}
    	}

    	if (millis() - startTime > static_cast<unsigned long>(handshakeTimeout))
    	{
      	break;
    	}
  	}

  	if (receivedByte == HANDSHAKE_BYTE)
  	{
    	return true;
  	}
  	else
  	{
    	Serial.printf("Handshake Serial timeOut!!!");
    	return false;
  	}
}

int getResponse(int bufferSize)
{
	char RX;
  	char bufferSizeStr[20];
  	sprintf(bufferSizeStr, "%d\0", bufferSize);
  	auto startTime = millis();
  	
	Sender.write(bufferSizeStr);

	while (true)
   {
   	if (Sender.available())
    	{
      	RX = Sender.read();
      	if (RX == 0x47)
      	{
        		return 0;
      	}
      	if (millis() - startTime > static_cast<unsigned long>(kResponseTimeout))
      	{
        		break;
      	}
    	}
    		
		if (millis() - startTime > static_cast<unsigned long>(kResponseTimeout))
    	{
      	break;
    	}
   }
	
	Serial.printf("Handshake serial timeout!");
  	return 1;
}

bool handleReadAmbient() 
{
	double ambVolt;
  	char ambLen[2];
  	char ambient[5];
  	char RX;
  	const int analogPin = 2;
	
	/*read and calculate ambient*/
  	ambVolt = analogReadMilliVolts(analogPin);
	double milAmb = (ambVolt / 1000);
  	int lux = ( (milAmb / 3) * 1125 );
  	sprintf(ambient,"%d", lux);
  
	int length = strlen(ambient);
   sprintf(ambLen, "%d", length);
  
	auto startTime = millis();
  	Sender.write(0x41);

  	while (true)
  	{
   	if (Sender.available())
    	{
      	RX = Sender.read();
      	if (RX == 0x5A) // Z
      	{
        		Sender.write(ambient);
        		break;
      	}

      	if (RX == 0x44) // D
      	{
         	Sender.write(ambLen);
      	}

      	if (millis() - startTime > static_cast<unsigned long>(kAmbientReadTimeout))
      	{
        		break;
      	}
    	}
    	
		if (millis() - startTime > static_cast<unsigned long>(kAmbientReadTimeout))
    	{
      	break;
    	}
  	 }
   
	Serial.printf("Ambient read timeout!");
  	return false;
}

bool handleGetSensorFirmwareVersion(char *sensor_version, char *ver_length)
{
	char RX;
  	auto startTime = millis();
  	Sender.write(0x41);
  	while (true)
  	{
   	if (Sender.available())
    	{
      	RX = Sender.read();
      	if (RX == 0x5A) // Z
      	{
        		Sender.write(sensor_version);
        		break;
      	}

      	if (RX == 0x44) // D
      	{
         	Sender.write(ver_length);
      	}

      	if (millis() - startTime > static_cast<unsigned long>(kVersionSyncTimeout))
      	{
        		break;
      	}
    	}

    	if (millis() - startTime > static_cast<unsigned long>(kVersionSyncTimeout))
    	{
      	break;
    	}
   }

  	Serial.printf("Version sync timeout!");
  	return false;
}

bool handleSyncMac(char *sensor_mac, char *mac_length)
{
	char RX;
	auto startTime = millis();
  	Sender.write(0x41);
  	while (true)
  	{
   	if (Sender.available())
    	{
      	RX = Sender.read();
      	if (RX == 0x5A) // Z
      	{
        		Sender.write(sensor_mac);
        		return true;
      	}
      	if (RX == 0x44) // D
      	{
         	Sender.write(mac_length);
      	}
    	}

    	if (millis() - startTime > static_cast<unsigned long>(kSyncMacTimeout))
    	{
      	break;
    	}
  	}

  	Serial.printf("Mac Sync timeout!");
  	return false;
}


void getFrame()
{
	/*To store the captured frame size*/
	char bufferSize[20];

  	int frameSize = 0;
  	int bufferSizeLength = 0;
  	char bufferSizeLengthBuffer[10];

	/*capture frame*/
	auto frame = esp32cam::capture();
  	if (frame == nullptr)
  	{
   		Serial.println("CAPTURE FAIL");
    		return;
  	}

	frameSize = static_cast<int>(frame->size());

  	Serial.printf("CAPTURE OK %dx%d %db\n", frame->getWidth(), frame->getHeight(), frameSize);

  	sprintf(bufferSize, "%d\r\n", frameSize);

  	bufferSizeLength = strlen(bufferSize);

  	Serial.printf("Buffer Size Length: %d\n", bufferSizeLength);

  	sprintf(bufferSizeLengthBuffer, "%d\r\n", bufferSizeLength);

  	/*First send the captured frame length to ncontroller*/
  	Sender.write(bufferSizeLengthBuffer);
  	if (handshake(2000) != true)
  	{
   		Serial.println("Buffer Size Length Handshake Fail");
    		return;
  	}
  	/*After handshake send the size of captured frame*/
  	Sender.write(bufferSize);
  	if (handshake(2000) != true)
  	{
   		Serial.println("Buffer Size Handshake Fail");
    		return;
  	}

  	constexpr int chunkSize = 4096;
  	int counter = 1;
  	int remaining = frameSize - chunkSize;
  	int rc;

	while (true)
  	{
   	printf("Start %d : \n", (counter - 1));
	 	
		/*Inform the ncontroller of outgoing frame chunk size and wait for response*/
    		rc = getResponse(chunkSize);
		
		/*If ncontroller sends acknowlegement start sending frame*/
		if (rc == 0)
    	{
   		Sender.write(&frame->data()[counter - 1], chunkSize);

   		if (remaining == 0)
   		{
     			printf("TS");
     			break;
   		}
   		else
   		{
   			remaining = remaining - chunkSize;
     			printf("remaining : %d\n", remaining);
     			if (remaining > 0)
     			{
      			counter += chunkSize;
     			}
     			else
     			{
      			int finalChunkSize = chunkSize + remaining;
       			counter += chunkSize;
       			printf("Final : %d\n", finalChunkSize);
       			remaining = 0;
     			}
   		}
    	}
    	else
    	{
      	Serial.print("TE\n");
      	break;
    	}
  	}
}

void getNsensorMacSerialno(char *sensor_mac, char *mac_length, char *ver_length)
{
  	Serial.println ();
  	Serial.println ("Reading Mac");
  	byte count = 0;
 
  	/*I2C address for MAC Eeprom**/
  	uint8_t a = 94;
  
  	/**I2C register address- Starting position for MAC byte*/
  	uint8_t v = 154;
  
  	/**Number of bytes (MAC)*/
  	uint8_t byt = 6;

  	uint8_t macbuff[6];

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
  
  	if (Wire.available() != 6) 
	{
    	Serial.print("Mac Readerror");
  	}

  	for(int x=0; x<6; x++) 
	{
    	macbuff[x] = Wire.read();
    	Serial.print (macbuff[x], HEX);
  	}

  	sprintf(sensor_mac,"%x:%x:%x:%x:%x:%x", macbuff[0], macbuff[1], macbuff[2], macbuff[3], macbuff[4], macbuff[5]);
  	length = strlen(sensor_mac);
  	sprintf(mac_length, "%i", length);
  	Serial.println(sensor_mac);
  	Serial.print(" = ");
  	Serial.println(mac_length);

  	sprintf(sensor_version,"%i.%i.%i", V_APP, V_MAJOR, V_MINOR);
  	length = strlen(sensor_version);
  	sprintf(ver_length, "%i", length);
  	Serial.println(sensor_version);

}

void handleGetRawCameraImage()
{
	Serial.println("OK!");
  	Sender.write(0x50);
  	getFrame();
}

/*Change here while compiling, based on the sensor device requirements*/
void handleGetSensorType()
{
  	Sender.write(FULL_OPTION_TYPE);
}

void handleChangeResolution()
{
  	Serial.println("Change Resolution");
  	ESP.restart();
}

/*This will run only once during start up*/
void setup()
{
  	/**Setting the baud rate for serial console*/
  	Serial.begin(115200); // Terminal

  	/*Setting the baud rate for serial data transmission **/
  	Sender.begin(BIT_RATE, SERIAL_8N1, 13, 12); // RX TX communication box - esp


 	/**Setting up camera
  	* sda= GPIO_04 
  	* scl= GPIO_14
  	*/
 	Wire.begin (GPIO_04, GPIO_14);  
  	{
    	using namespace esp32cam;
    	Config cfg;
 		cfg.setPins(pins::AiThinker);
 		cfg.setResolution(semisqr);
 		cfg.setBufferCount(1);
 		cfg.setJpeg(80);
 		bool ok = Camera.begin(cfg);
 		Serial.println(ok ? "CAMERA OK" : "CAMERA FAIL");
  	}

	/**Here it gets MAC address and serial number of the nsensor*/
  	getNsensorMacSerialno((sensor_mac, mac_length, ver_length);
}

/**This is the main app code, It loops here for the input from the 
 * ncontroller */
void loop() 
{
	while (Sender.available())
	{
		char ncontroller_cmd = Sender.read();
    	Serial.print("RX : ");
    	Serial.println(ncontroller_cmd);

    	switch (ncontroller_cmd)
    	{
    		case GET_RAW_CAMERA_IMAGE:
      		handleGetRawCameraImage();
      		break;
    		case SYNC_MAC_CMD:
      		handleSyncMac(sensor_mac, mac_length);
      		break;
    		case READ_AMBIENT_CMD:
      		handleReadAmbient();
      		break;
    		case GET_SENSOR_TYPE_CMD:
      		handleGetSensorType();
      		break;
    		case GET_SENSOR_FIRMWARE_VERSION:
      		handleGetSensorFirmwareVersion(sensor_version, ver_length);
      		break;
    		case CHANGE_RESOLUTION_CMD:
      		handleChangeResolution();
      		break;
    		default :
      		break;
    	}
  	}
}

