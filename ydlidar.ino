/*
 * Author: Emanuele Feola
 * Based on: https://github.com/EAIBOT/ydlidar_arduino
 * Last modified: 20/08/2021
 * Description: read ydlidar G2 data, print data on serial on separate task
 * Notes: serial output example in github repo
 * Notes: in github repo there is the serial output parsing and visualization screenshot compared with ydlidar's windows software
 * Please note: this parser is based on YDLIDAR G2 Development Manual, so it will not work with other devices
 */

 /// Lidar Parser
#include "LidarParser.h"
LidarParser lidar;
device_health healthinfo;
device_info deviceinfo;
freq_info freqinfo;
bool isScanning = false;

///  Tasks
#define MAIN_PERIOD 250
#define ANGLE_STEP 60
#define MSG_SIZE 500
TaskHandle_t TaskLoop;
bool inited = false;
char buffer[MSG_SIZE] = { 0 };
int mainInterval = 0;
uint16_t angleCounter = 0;

void TaskLoopRoutine(void* pvParameters)
{
	Serial.printf("[TaskLoopRoutine] running on core %d\n", xPortGetCoreID());

	while (1)
	{
		vTaskDelay(1);

		if (millis() - mainInterval < MAIN_PERIOD || !inited)
			continue;

		mainInterval = millis();

		angleCounter += ANGLE_STEP;
		angleCounter %= 360;

		memset(buffer, 0, sizeof(buffer));
		snprintf(buffer, MSG_SIZE, "H;%d;\n", angleCounter);

		for (int i = 0; i < ANGLE_STEP; i++)
			snprintf(buffer, MSG_SIZE, "%s%d;%d\n", buffer, lidar.distanceList[angleCounter + i], lidar.lightList[angleCounter + i]);

		Serial.println(buffer);
	}
}

void setup() {
	delay(3000);

	Serial.begin(921600);
	Serial.println("System start");
	Serial.printf("[setup] running on core %d\n", xPortGetCoreID());

	delay(500);

	/// lidar
	lidar.begin(Serial1);

	if (lidar.getHealth(healthinfo, 100) == RESULT_OK) {
		Serial.print("[YDLIDAR INFO] YDLIDAR running correctly! The health status:");
		Serial.println(healthinfo.status == 0 ? "well" : "bad");
	}
	else {
		Serial.println("cannot retrieve YDLIDAR health");
		while (1);
	}

	if (lidar.getDeviceInfo(deviceinfo, 100) == RESULT_OK) {
		String model;

		uint16_t maxv = (uint16_t)(deviceinfo.firmware_version >> 8);
		uint16_t midv = (uint16_t)(deviceinfo.firmware_version & 0xff) / 10;
		uint16_t minv = (uint16_t)(deviceinfo.firmware_version & 0xff) % 10;
		if (midv == 0) {
			midv = minv;
			minv = 0;
		}

		Serial.print("Firmware version:");
		Serial.print(maxv, DEC);
		Serial.print(".");
		Serial.print(midv, DEC);
		Serial.print(".");
		Serial.println(minv, DEC);

		Serial.print("Hardware version:");
		Serial.println((uint16_t)deviceinfo.hardware_version, DEC);

		Serial.printf("Model: %d\n", deviceinfo.model & 0xff);

		Serial.print("Serial:");
		for (int i = 0; i < 16; i++) {
			Serial.print(deviceinfo.serialnum[i] & 0xff, DEC);
		}
		Serial.println("");
	}

	if (lidar.startScan() == RESULT_OK) {
		Serial.println("Now YDLIDAR is scanning ......");
		isScanning = true;
		delay(1000);
	}
	else {
		Serial.println("start YDLIDAR is failed!  Continue........");
	}

	xTaskCreatePinnedToCore(
		TaskLoopRoutine,
		"TaskLoop",
		10000,
		NULL,
		1,
		&TaskLoop,
		0);
}

void loop() {
	/// to elaborate data:
	if (isScanning)
		if (lidar.waitScanDot(50) == RESULT_OK)
			if (!inited) inited = true;

	/// to print lidar's raw data:
	/*
	byte test = 0;
	Serial1.readBytes(&test, 1);
	Serial.printf("%x\n", test);
	*/
}