/*
Code for a The Things Network GPS mapper
This sends the GPS latitude, longitude, altitude and hdop data to TTN
You must set up a n application and end device within TTN
This uses the Heltec ESP32 Lora V3 board


Libraries to install:

Heltec ESP32 Library from Library manager:
https://github.com/HelTecAutomation/Heltec_ESP32

This is for: #include "LoRaWan_APP.h" and #include "HT_SSD1306Wire.h" and #include "HT_TinyGPS++.h"

 * Heltec Automation LoRaWAN communication example
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
// Esample Data - this is junk example data - please add you own device data here:
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
//uint32_t appTxDutyCycle = 300000;	// Every 5 mins
RTC_DATA_ATTR uint32_t appTxDutyCycle = 60000;  // Every 1 mins
//RTC_DATA_ATTR uint32_t appTxDutyCycle = 15000;  // Every 1 mins

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

int previous_deviceState = deviceState;

// For the OLED display:
#include <Wire.h>
#include "HT_SSD1306Wire.h"
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);  // addr , freq , i2c group , resolution , rst

// For the GPS sensor
// Define the RX and TX pins for Serial 2
#include "HT_TinyGPS++.h"
#define RXD2 4
#define TXD2 3
#define GPS_BAUD 9600
// Create an instance of the HardwareSerial class for Serial 2
HardwareSerial gpsSerial(2);
// The TinyGPS++ object
TinyGPSPlus gps;

// For the switches
#include "driver/rtc_io.h"  // Required for RTC pin control
#define WAKEUP_PIN_1 GPIO_NUM_7
#define WAKEUP_PIN_2 GPIO_NUM_6
// Create a 64-bit mask of the combined pins for ext1 configuration
#define BUTTON_BITMASK ((1ULL << WAKEUP_PIN_1) | (1ULL << WAKEUP_PIN_2))
// #define BUTTON_PIN_BITMASK 0xC0  // 2^7+2^6 in hex
bool BUTTON_A_FLAG = false;
bool BUTTON_B_FLAG = false;
uint32_t button_timer = millis();
int display_time = 2000;  // Time to display in mS
bool UPDATE_Tx = true;

int displayDelay = 0;  // This value holds displays on the screen if required
int battVolts;

// Buffer with data to send to TTNMapper
uint8_t txBuffer[10];

/* Prepares the payload of the frame */
static void prepareTxFrame(uint8_t port) {
	/*appData size is LORAWAN_APP_DATA_MAX_SIZE which is defined in "commissioning.h".
	*appDataSize max value is LORAWAN_APP_DATA_MAX_SIZE.
	*if enabled AT, don't modify LORAWAN_APP_DATA_MAX_SIZE, it may cause system hanging or failure.
	*if disabled AT, LORAWAN_APP_DATA_MAX_SIZE can be modified, the max value is reference to lorawan region and SF.
	*for example, if use REGION_CN470, 
	*the max value for different DR can be found in MaxPayloadOfDatarateCN470 refer to DataratesCN470 and BandwidthsCN470 in "RegionCN470.h".
	*/
	appDataSize = 10;
	appData[0] = txBuffer[0];
	appData[1] = txBuffer[1];
	appData[2] = txBuffer[2];
	appData[3] = txBuffer[3];
	appData[4] = txBuffer[4];
	appData[5] = txBuffer[5];
	appData[6] = txBuffer[6];
	appData[7] = txBuffer[7];
	appData[8] = txBuffer[8];
	appData[9] = txBuffer[9];
}

bool build_packet() {

	uint32_t LatitudeBinary, LongitudeBinary;
	uint16_t altitudeGps;
	uint8_t hdopGps;

	// This sketch displays information every time a new sentence is correctly encoded.
	unsigned long start = millis();

	while (millis() < start + 1000) {
		while ((gpsSerial.available() > 0)) {
			gps.encode(gpsSerial.read());
		}
	}

	if (gps.location.isValid()) {
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

		double latitude = gps.location.lat();
		double longitude = gps.location.lng();
		double altitude = gps.altitude.meters();
		uint32_t hdop = gps.hdop.value();

		LatitudeBinary = ((latitude + 90) / 180.0) * 16777215;
		LongitudeBinary = ((longitude + 180) / 360.0) * 16777215;

		txBuffer[0] = (LatitudeBinary >> 16) & 0xFF;
		txBuffer[1] = (LatitudeBinary >> 8) & 0xFF;
		txBuffer[2] = LatitudeBinary & 0xFF;

		txBuffer[3] = (LongitudeBinary >> 16) & 0xFF;
		txBuffer[4] = (LongitudeBinary >> 8) & 0xFF;
		txBuffer[5] = LongitudeBinary & 0xFF;

		altitudeGps = altitude;
		txBuffer[6] = (altitudeGps >> 8) & 0xFF;
		txBuffer[7] = altitudeGps & 0xFF;

		// This links to quality of GPS data - lower than 2 is good.
		hdopGps = hdop / 10;
		txBuffer[8] = hdopGps & 0xFF;

		// Also send the battery voltage - can be used as recharge warning
		// Read the analog voltage in millivolts from pin 1:
		battVolts = (analogReadMilliVolts(1) * 490) / 2000;
		// battVolts is a value from 0-250, which is 0 - 5V (multiply by 2 and multiply by 10)
		// Print the scaled millivolts value (scaled by a factor of 490/100):
		Serial.printf("ADC millivolts value = %d\n", battVolts * 20);
		txBuffer[9] = battVolts;
		return (true);
	} else {
		Serial.println("GPS not valid");
		return (false);
	}
}

void setup() {

	Serial.begin(115200);
	Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

	// Switch ON external power for OLED
#ifdef WIFI_LORA_32_V4
	pinMode(Vext, OUTPUT);
	digitalWrite(Vext, LOW);
#endif
#ifdef WIFI_LORA_32_V3
	VextON();
	delay(100);
#endif

	if (firstrun) {
		BoardInitMcu();
		//LoRaWAN.displayMcuInit();	// This writes over the display - not useful!
		firstrun = false;
	}

	// Initialise the display
	display.init();
	display.setTextAlignment(TEXT_ALIGN_LEFT);
	display.setFont(ArialMT_Plain_10);
	display.clear();
	display.drawString(0, 0, "Starting...");
	display.display();
	delay(100);

	// Start Serial 2 with the defined RX and TX pins and a baud rate of 9600
	gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
	Serial.println("Serial 2 for GPS @9600 OK");

	// Sort out the Battery Voltage Read settings:
	// Set the resolution of the analog-to-digital converter (ADC) to 12 bits (0-4095):
	analogReadResolution(12);
	// Set pin 37 as an output pin (used for ADC control):
	pinMode(37, OUTPUT);
	// Set pin 37 to HIGH (enable ADC control):
	digitalWrite(37, HIGH);

	// Identify why the chip reset or woke up
	esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
	if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
		// Read the bitmask of the specific pin that triggered the wakeup event
		uint64_t pin_status = esp_sleep_get_ext1_wakeup_status();
		if (pin_status & (1ULL << WAKEUP_PIN_1)) {
			Serial.println("Wakeup caused by Button A!");
			BUTTON_A_FLAG = true;
			button_timer = millis();
			UPDATE_Tx = true;
		} else if (pin_status & (1ULL << WAKEUP_PIN_2)) {
			Serial.println("Wakeup caused by Button B!");
			BUTTON_B_FLAG = true;
			button_timer = millis();
		}
	} else {
		Serial.println("Normal power-on boot or hardware reset.");
	}
}

void loop() {
	if (BUTTON_A_FLAG == true) {
		if (UPDATE_Tx == true) {
			// Adjust the Tx
			if (appTxDutyCycle == 15000) {
				appTxDutyCycle = 30000;
			} else if (appTxDutyCycle == 30000) {
				appTxDutyCycle = 60000;
			} else if (appTxDutyCycle == 60000) {
				appTxDutyCycle = 300000;
			} else if (appTxDutyCycle == 300000) {
				appTxDutyCycle = 900000;
			} else {
				appTxDutyCycle = 15000;
			}
			UPDATE_Tx = false;  // only do this once!
			deviceState = DEVICE_STATE_CYCLE;
		}
	} else if (BUTTON_B_FLAG == true) {
		// Get the GPS data and display it
		build_packet();
		deviceState = DEVICE_STATE_CYCLE;
	}

	if (deviceState != previous_deviceState) {
		display.clear();
		previous_deviceState = deviceState;
	}
	switch (deviceState) {
		case DEVICE_STATE_INIT:
			{
				display.drawString(0, 0, "Initialising...");
				displayDelay = 100;
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
				//LoRaWAN.displayJoining();
				display.drawString(0, 0, "Joining...");
				displayDelay = 100;
				LoRaWAN.join();
				break;
			}
		case DEVICE_STATE_SEND:
			{
				//LoRaWAN.displaySending();
				display.drawString(0, 0, "Sending...");
				if (build_packet()) {
					prepareTxFrame(appPort);
					display.drawString(0, 10, "TTN SENT");
					LoRaWAN.send();
				} else {
					display.drawString(0, 10, "GPS ERROR");
				}
				displayDelay = 500;
				deviceState = DEVICE_STATE_CYCLE;
				break;
			}
		case DEVICE_STATE_CYCLE:
			{
				// Schedule next packet transmission
				display.drawString(0, 0, "Sleeping...");
				displayDelay = 500;
				txDutyCycleTime = appTxDutyCycle + randr(-APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND);
				LoRaWAN.cycle(txDutyCycleTime);
				deviceState = DEVICE_STATE_SLEEP;
				break;
			}
		case DEVICE_STATE_SLEEP:
			{
				//LoRaWAN.displayAck();
				// display.drawString(0, 10, "ACK RECEIVED");		// ***TO DO *****
				goToSleep();

				break;
			}
		default:
			{
				deviceState = DEVICE_STATE_INIT;
				break;
			}
	}

	// Highlight the LoRaWAN Tx Time
	display.drawString(0, 50, "LoRa Tx:");
	int tx_time_s = appTxDutyCycle / 1000;
	display.drawString(50, 50, (String)tx_time_s);
	// Show the battery voltage
	display.drawString(0, 20, "Batt V:");
	display.drawString(50, 20, (String)(battVolts * 20));

	// Highlight the GPS values (if available)
	if (gps.location.isValid()) {
		display.drawString(0, 30, (String(gps.date.year()) + "/" + String(gps.date.month()) + "/" + String(gps.date.day()) + "," + String(gps.time.hour()) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second())));
		display.drawString(0, 40, "Lat:");
		display.drawString(20, 40, String(gps.location.lat(), 4));
		display.drawString(64, 40, "Lon:");
		display.drawString(84, 40, String(gps.location.lng(), 4));
	} else {
		display.drawString(0, 40, "Lat:");
		display.drawString(20, 40, "ERR");
		display.drawString(64, 40, "Lon:");
		display.drawString(84, 40, "ERR");
	}
	display.display();
	delay(displayDelay);

	if (BUTTON_A_FLAG || BUTTON_B_FLAG) {
		// wait for a while before going to sleep
		if (millis() > (button_timer + display_time)) {
			goToSleep();
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

void goToSleep() {

	// Configure RTC pull-up to keep the pin HIGH during deep sleep
	rtc_gpio_init(WAKEUP_PIN_1);
	rtc_gpio_set_direction(WAKEUP_PIN_1, RTC_GPIO_MODE_INPUT_ONLY);
	rtc_gpio_pullup_en(WAKEUP_PIN_1);     // Enable internal pull-up resistor
	rtc_gpio_pulldown_dis(WAKEUP_PIN_1);  // Disable pull-down resistor

	rtc_gpio_init(WAKEUP_PIN_2);
	rtc_gpio_set_direction(WAKEUP_PIN_2, RTC_GPIO_MODE_INPUT_ONLY);
	rtc_gpio_pullup_en(WAKEUP_PIN_2);     // Enable internal pull-up resistor
	rtc_gpio_pulldown_dis(WAKEUP_PIN_2);  // Disable pull-down resistor

	//Register the multi - pin array mask to trigger when ANY target pin shifts LOW
	esp_sleep_enable_ext1_wakeup(BUTTON_BITMASK, ESP_EXT1_WAKEUP_ANY_LOW);

	// Enter deep sleep. Execution halts here and boots from setup() upon next wakeup.
	Serial.println("ZZzz..");

	LoRaWAN.sleep(loraWanClass);
}
