/* Heltec Automation LoRaWAN communication example
 *
 * Function:
 * 1. Upload node data to the server using the standard LoRaWAN protocol.
 * 2. The network access status of LoRaWAN is displayed on the screen.
 * 
 * Description:
 * 1. Communicate using LoRaWAN protocol.
 * 
 * HelTec AutoMation, Chengdu, China
 * 成都惠利特自动化科技有限公司
 * www.heltec.org
 *
 * this project also realess in GitHub:
 * https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
 * */

// the Arduino build environment automatically includes Arduino.h for .ino
// sketches, so we don’t need to #include it and avoid the “cannot open source
// file" error.
// add the core header explicitly so editors/linters with a broken includePath
// can still resolve the dependency
#include <Arduino.h>
#include "LoRaWan_APP.h"

/* OTAA para*/
uint8_t devEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appEui[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
uint8_t appKey[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/* ABP para*/
// Esample Data
uint8_t nwkSKey[] = { 0x15, 0xb1, 0xd0, 0xef, 0xa4, 0x63, 0xdf, 0xbe, 0x3d, 0x11, 0x18, 0x1e, 0x1e, 0xc7, 0xda,0x85 };
uint8_t appSKey[] = { 0xd7, 0x2c, 0x78, 0x75, 0x8c, 0xdc, 0xca, 0xbf, 0x55, 0xee, 0x4a, 0x77, 0x8d, 0x16, 0xef,0x67 };
uint32_t devAddr =  ( uint32_t )0x007e6ae1;


/*LoraWan channelsmask*/
uint16_t userChannelsMask[6] = { 0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000 };

/*LoraWan region, select in arduino IDE tools*/
LoRaMacRegion_t loraWanRegion = ACTIVE_REGION;

/*LoraWan Class, Class A and Class C are supported*/
DeviceClass_t loraWanClass = CLASS_A;

/*the application data transmission duty cycle.  value in [ms].*/
uint32_t appTxDutyCycle = 30000;	// Every 5 mins

/*OTAA or ABP*/
bool overTheAirActivation = false;

/*ADR enable*/
bool loraWanAdr = true;

/* Indicates if the node is sending confirmed or unconfirmed messages */
bool isTxConfirmed = false;
//bool isTxConfirmed = true;

/* Application port */
uint8_t appPort = 2;

/*!
* Number of trials to transmit the frame, if the LoRaMAC layer did not
* receive an acknowledgment. The MAC performs a datarate adaptation,
* according to the LoRaWAN Specification V1.0.2, chapter 18.4, according
* to the following table:
*
* Transmission nb | Data Rate
* ----------------|-----------
* 1 (first)       | DR
* 2               | DR
* 3               | max(DR-1,0)
* 4               | max(DR-1,0)
* 5               | max(DR-2,0)
* 6               | max(DR-2,0)
* 7               | max(DR-3,0)
* 8               | max(DR-3,0)
*
* Note, that if NbTrials is set to 1 or 2, the MAC will not decrease
* the datarate, in case the LoRaMAC layer did not receive an acknowledgment
*/
uint8_t confirmedNbTrials = 4;

RTC_DATA_ATTR bool firstrun = true;


// For the OLED display:
#include <Wire.h>
#include "HT_SSD1306Wire.h"
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);  // addr , freq , i2c group , resolution , rst

// For the GPS sensor
 // Define the RX and TX pins for Serial 2
#include "HT_TinyGPS++.h"
#define RXD2 47
#define TXD2 48
#define GPS_BAUD 9600
// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);
// The TinyGPS++ object
TinyGPSPlus gps;

// Buffer with data to send to TTNMapper
uint8_t txBuffer[9];

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port) {
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/

	build_packet();  // Sort out the GPS data

	appDataSize = 9;
	appData[0] = txBuffer[0];
	appData[1] = txBuffer[1];
	appData[2] = txBuffer[2];
	appData[3] = txBuffer[3];
	appData[4] = txBuffer[4];
	appData[5] = txBuffer[5];
	appData[6] = txBuffer[6];
	appData[7] = txBuffer[7];
	appData[8] = txBuffer[8];
}

void build_packet() {
	char s[16];  // used to sprintf for OLED display
	String toLog;

	uint32_t LatitudeBinary, LongitudeBinary;
	uint16_t altitudeGps;
	uint8_t hdopGps;

 // This sketch displays information every time a new sentence is correctly encoded.
  unsigned long start = millis();

  while (millis() - start < 1000) {
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isUpdated()) {
      Serial.print("LAT: ");
      Serial.println(gps.location.lat(), 6);
      Serial.print("LONG: "); 
      Serial.println(gps.location.lng(), 6);
      Serial.print("SPEED (km/h) = "); 
      Serial.println(gps.speed.kmph()); 
      Serial.print("ALT (min)= "); 
      Serial.println(gps.altitude.meters());
      Serial.print("HDOP = "); 
      Serial.println(gps.hdop.value() / 100.0); 
      Serial.print("Satellites = "); 
      Serial.println(gps.satellites.value()); 
      Serial.print("Time in UTC: ");
      Serial.println(String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second()));
      Serial.println("");
    }
  }

	double latitude = gps.location.lat();
	double longitude = gps.location.lng();
	double altitude = gps.altitude.meters();
	uint32_t hdop = gps.hdop.value();

	// double latitude = 52;
	// double longitude = 0;
	// double altitude = 34;
	// uint32_t hdop = 2;

	LatitudeBinary = ((latitude + 90) / 180.0) * 16777215;
	LongitudeBinary = ((longitude + 180) / 360.0) * 16777215;

	// display.drawString(0, 2, "Lat: ");
	// sprintf(s, "%f", latitude);
	// display.drawString(5, 2, s);
	// display.drawString(0, 3, "Lng: ");
	// sprintf(s, "%f", longitude);
	// display.drawString(5, 3, s);
	// // write the buffer to the display
	// display.display();

	// Serial.printf("Lat: %f", latitude);
	// Serial.printf(" Lng: %f", longitude);
	// Serial.printf(" alt: %f", altitude);
	// Serial.printf(" hdop: %d", hdop);
	// Serial.println();

	txBuffer[0] = (LatitudeBinary >> 16) & 0xFF;
	txBuffer[1] = (LatitudeBinary >> 8) & 0xFF;
	txBuffer[2] = LatitudeBinary & 0xFF;

	txBuffer[3] = (LongitudeBinary >> 16) & 0xFF;
	txBuffer[4] = (LongitudeBinary >> 8) & 0xFF;
	txBuffer[5] = LongitudeBinary & 0xFF;

	altitudeGps = altitude;
	txBuffer[6] = (altitudeGps >> 8) & 0xFF;
	txBuffer[7] = altitudeGps & 0xFF;

	hdopGps = hdop / 10;
	txBuffer[8] = hdopGps & 0xFF;

	// toLog = "";
	// for (size_t i = 0; i < sizeof(txBuffer); i++) {
	//   char buffer[3];
	//   sprintf(buffer, "%02x", txBuffer[i]);
	//   toLog = toLog + String(buffer);
	// }
	// Serial.println(toLog);
}


void setup() {

	Serial.begin(115200);
	Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  // Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("Serial 2 started at 9600 baud rate");

#ifdef WIFI_LORA_32_V4
	pinMode(Vext, OUTPUT);
	digitalWrite(Vext, LOW);
#endif

#ifdef WIFI_LORA_32_V3
	VextON();
#endif

	display.init();
	display.setFont(ArialMT_Plain_10);
	display.setColor(WHITE);
	// display.setTextAlignment(TEXT_ALIGN_LEFT);
	// display.clear();
	// display.drawString(0, 0, "TTN Mapper");
	// // write the buffer to the display
	// display.display();


	if (firstrun) {
		LoRaWAN.displayMcuInit();
		firstrun = false;
	}
}

void loop() {
	switch (deviceState) {
		case DEVICE_STATE_INIT:
			{
#if (LORAWAN_DEVEUI_AUTO)
				LoRaWAN.generateDeveuiByChipID();
#endif
				LoRaWAN.init(loraWanClass, loraWanRegion);
				//both set join DR and DR when ADR off
				LoRaWAN.setDefaultDR(3);
				break;
			}
		case DEVICE_STATE_JOIN:
			{
				LoRaWAN.displayJoining();
				LoRaWAN.join();
				break;
			}
		case DEVICE_STATE_SEND:
			{
				LoRaWAN.displaySending();
				prepareTxFrame(appPort);
				LoRaWAN.send();
				deviceState = DEVICE_STATE_CYCLE;
				break;
			}
		case DEVICE_STATE_CYCLE:
			{
				// Schedule next packet transmission
				txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
				LoRaWAN.cycle(txDutyCycleTime);
				deviceState = DEVICE_STATE_SLEEP;
				break;
			}
		case DEVICE_STATE_SLEEP:
			{
				LoRaWAN.displayAck();
				LoRaWAN.sleep(loraWanClass);
				break;
			}
		default:
			{
				deviceState = DEVICE_STATE_INIT;
				break;
			}
	}
}

void VextON(void) {
	pinMode(Vext, OUTPUT);
	digitalWrite(Vext, LOW);
}

void VextOFF(void)  //Vext default OFF
{
	pinMode(Vext, OUTPUT);
	digitalWrite(Vext, HIGH);
}
