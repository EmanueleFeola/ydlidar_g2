#include "LidarParser.h"
#include <stdlib.h>     /* malloc, free, rand */
#include <math.h>       /* atan2 */

LidarParser::LidarParser()
	: _bined_serialdev(NULL)
{
	memset(lightList, 0, sizeof(lightList));
	memset(distanceList, 0, sizeof(distanceList));
	memset(temp, 0, sizeof(temp)); // needed to store temporary cloud points distance
}

// open the given serial interface and try to connect to the LidarParser
bool LidarParser::begin(HardwareSerial& serialobj)
{
	_bined_serialdev = &serialobj;
	_bined_serialdev->end();
	_bined_serialdev->begin(SERIAL_BAUDRATE, SERIAL_8N1, RX_SERIAL_ESP, TX_SERIAL_ESP);

	while (!_bined_serialdev);

	return true;
}

// check whether the serial interface is opened
bool LidarParser::isOpen(void)
{
	return _bined_serialdev ? true : false;
}

//send data to serial
result_t LidarParser::sendCommand(uint8_t cmd, const void* payload, size_t payloadsize) {
	cmd_packet pkt_header;
	cmd_packet* header = &pkt_header;

	header->syncByte = LIDAR_CMD_SYNC_BYTE;
	header->cmd_flag = cmd & 0xff;

	_bined_serialdev->write((uint8_t*)header, 2);
	return RESULT_OK;
}

// wait response header
result_t LidarParser::waitResponseHeader(lidar_ans_header* header, uint32_t timeout) {
	Serial.printf("[waitResponseHeader] running on core %d\n", xPortGetCoreID());

	int  recvPos = 0;
	uint32_t startTs = millis();
	uint8_t* headerBuffer = (uint8_t*)(header);
	uint32_t waitTime;

	while ((waitTime = millis() - startTs) <= timeout) {
		int currentbyte = _bined_serialdev->read();
		if (currentbyte < 0) continue;
		switch (recvPos) {
		case 0:
			if (currentbyte != LIDAR_ANS_SYNC_BYTE1) {
				continue;
			}
			break;
		case 1:
			if (currentbyte != LIDAR_ANS_SYNC_BYTE2) {
				recvPos = 0;
				continue;
			}
			break;
		}
		headerBuffer[recvPos++] = currentbyte;

		if (recvPos == sizeof(lidar_ans_header)) {
			Serial.println("[waitResponseHeader] reading complete\n");
			return RESULT_OK;
		}
	}
	return RESULT_TIMEOUT;
}

result_t LidarParser::getHealth(device_health& health, uint32_t timeout) {
	result_t  ans;
	uint8_t  recvPos = 0;
	uint32_t currentTs = millis();
	uint32_t remainingtime;
	uint8_t* infobuf = (uint8_t*)&health;
	lidar_ans_header response_header;
	if (!isOpen()) {
		return RESULT_FAIL;
	}

	{

		ans = sendCommand(LIDAR_CMD_GET_DEVICE_HEALTH, NULL, 0);
		if (ans != RESULT_OK) {
			return ans;
		}


		if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
			return ans;
		}

		if (response_header.type != LIDAR_ANS_TYPE_DEVHEALTH) {
			return RESULT_FAIL;
		}

		if (response_header.size < sizeof(device_health)) {
			return RESULT_FAIL;
		}

		while ((remainingtime = millis() - currentTs) <= timeout) {
			int currentbyte = _bined_serialdev->read();
			if (currentbyte < 0) continue;
			infobuf[recvPos++] = currentbyte;

			if (recvPos == sizeof(device_health)) {
				Serial.println("[getHealth] reading complete\n");
				return RESULT_OK;
			}
		}
	}

	return RESULT_TIMEOUT;
}

// ask the YDLIDAR for its device info 
result_t LidarParser::getDeviceInfo(device_info& info, uint32_t timeout) {
	result_t  ans;
	uint8_t  recvPos = 0;
	uint32_t currentTs = millis();
	uint32_t remainingtime;
	uint8_t* infobuf = (uint8_t*)&info;
	lidar_ans_header response_header;
	if (!isOpen()) {
		return RESULT_FAIL;
	}

	{

		ans = sendCommand(LIDAR_CMD_GET_DEVICE_INFO, NULL, 0);
		if (ans != RESULT_OK) {
			return ans;
		}


		if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
			return ans;
		}

		if (response_header.type != LIDAR_ANS_TYPE_DEVINFO) {
			return RESULT_FAIL;
		}

		if (response_header.size < sizeof(lidar_ans_header)) {
			return RESULT_FAIL;
		}

		while ((remainingtime = millis() - currentTs) <= timeout) {
			int currentbyte = _bined_serialdev->read();
			if (currentbyte < 0) continue;
			infobuf[recvPos++] = currentbyte;

			//Serial.printf("curByte: %x\n", currentbyte);

			if (recvPos == sizeof(device_info)) {
				return RESULT_OK;
			}
		}
	}

	return RESULT_TIMEOUT;
}

// stop the scanPoint operation
result_t LidarParser::stop(void)
{
	if (!isOpen()) return RESULT_FAIL;
	//result_t ans = sendCommand(LIDAR_CMD_FORCE_STOP, NULL, 0); 
	result_t ans = sendCommand(LIDAR_CMD_STOP, NULL, 0);
	return ans;
}

// get set scan frequency
result_t LidarParser::getScanFreq(freq_info& freq, uint32_t timeout) {
	result_t  ans;
	uint8_t  recvPos = 0;
	uint32_t currentTs = millis();
	uint32_t remainingtime;
	lidar_ans_header response_header;
	uint8_t* freqbuf = (uint8_t*)&freq;

	{
		if ((ans = sendCommand(LIDAR_SCAN_FREQ, NULL, 0)) != RESULT_OK) {
			return ans;
		}

		lidar_ans_header response_header;
		if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
			return ans;
		}

		if (response_header.type != LIDAR_ANS_TYPE_GETFREQ) {
			return RESULT_FAIL;
		}

		if (response_header.size < sizeof(freq_info)) {
			return RESULT_FAIL;
		}

		while ((remainingtime = millis() - currentTs) <= timeout) {
			int currentbyte = _bined_serialdev->read();
			if (currentbyte < 0) continue;
			freqbuf[recvPos++] = currentbyte;

			if (recvPos == sizeof(device_info)) {
				return RESULT_OK;
			}
		}
	}
	return RESULT_OK;
}

result_t LidarParser::increaseFreq(freq_info& freq, uint32_t timeout) {
	result_t  ans;
	uint8_t  recvPos = 0;
	uint32_t currentTs = millis();
	uint32_t remainingtime;
	lidar_ans_header response_header;
	uint8_t* freqbuf = (uint8_t*)&freq;

	{
		if ((ans = sendCommand(LIDAR_SCAN_FREQ_INC_01, NULL, 0)) != RESULT_OK) {
			return ans;
		}

		lidar_ans_header response_header;
		if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
			return ans;
		}

		if (response_header.type != LIDAR_ANS_TYPE_GETFREQ) {
			return RESULT_FAIL;
		}

		if (response_header.size < sizeof(freq_info)) {
			return RESULT_FAIL;
		}

		while ((remainingtime = millis() - currentTs) <= timeout) {
			int currentbyte = _bined_serialdev->read();
			if (currentbyte < 0) continue;
			freqbuf[recvPos++] = currentbyte;

			if (recvPos == sizeof(device_info)) {
				return RESULT_OK;
			}
		}
	}
	return RESULT_OK;
}

result_t LidarParser::decreaseFreq(freq_info& freq, uint32_t timeout) {
	result_t  ans;
	uint8_t  recvPos = 0;
	uint32_t currentTs = millis();
	uint32_t remainingtime;
	lidar_ans_header response_header;
	uint8_t* freqbuf = (uint8_t*)&freq;

	{
		if ((ans = sendCommand(LIDAR_SCAN_FREQ_RED_01, NULL, 0)) != RESULT_OK) {
			return ans;
		}

		lidar_ans_header response_header;
		if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
			return ans;
		}

		if (response_header.type != LIDAR_ANS_TYPE_GETFREQ) {
			return RESULT_FAIL;
		}

		if (response_header.size < sizeof(freq_info)) {
			return RESULT_FAIL;
		}

		while ((remainingtime = millis() - currentTs) <= timeout) {
			int currentbyte = _bined_serialdev->read();
			if (currentbyte < 0) continue;
			freqbuf[recvPos++] = currentbyte;

			if (recvPos == sizeof(device_info)) {
				return RESULT_OK;
			}
		}
	}
	return RESULT_OK;
}

// start the scanPoint operation
result_t LidarParser::startScan(uint32_t timeout) {
	result_t ans;

	if (!isOpen()) return RESULT_FAIL;

	stop(); //force the previous operation to stop

	{
		if ((ans = sendCommand(LIDAR_CMD_SCAN, NULL, 0)) != RESULT_OK) {
			return ans;
		}

		lidar_ans_header response_header;
		if ((ans = waitResponseHeader(&response_header, timeout)) != RESULT_OK) {
			return ans;
		}

		if (response_header.type != LIDAR_ANS_TYPE_MEASUREMENT) {
			return RESULT_FAIL;
		}

		if (response_header.size < sizeof(node_info)) {
			return RESULT_FAIL;
		}
	}
	return RESULT_OK;
}

result_t LidarParser::waitScanDot(uint32_t timeout) {
	/*
	 * Working: cloud point parsing is correct
	 * Not working: checksum calculation is wrong, help is needed
	 */
	int startTs = millis();
	int waitTime;
	uint16_t LSN = 0;
	uint16_t FSA = 0;
	float angleFSA = 0;
	uint16_t LSA = 0;
	float angleLSA = 0;
	int checkSum = 0;
	uint16_t fsmState = 0;
	uint16_t CT = 0;
	int checkSumCal = 0;

	byte scanResponse[200] = { 0 };

	while ((waitTime = millis() - startTs) <= timeout && fsmState < PackagePaidBytes) {
		if (!_bined_serialdev->available()) {
			continue;
		}

		byte cur = { 0 };
		_bined_serialdev->readBytes(&cur, 1);

		switch (fsmState)
		{
		case 0:
			if (cur != (PH & 0xFF)) {
				fsmState = 0;
				continue;
			}
			break;

		case 1:
			checkSumCal = PH;

			if (cur != (PH >> 8)) {
				fsmState = 0;
				continue;
			}
			break;

			/// CT
		case 2:
			CT = cur;

			//if (CT != 0x00 && CT != 0x01) {
			//	fsmState = 0;
			//	continue;
			//}
			break;

			/// LSN
		case 3:
			LSN = cur;
			break;

			/// FSA
		case 4:
			FSA = cur;
			break;

		case 5:
			FSA += (cur << 8);
			checkSumCal ^= FSA;
			angleFSA = (FSA >> 1) / 64.0f;
			break;

			/// LSA
		case 6:
			LSA = cur;
			break;

		case 7:
			LSA += (cur << 8);
			checkSumCal ^= LSA;
			angleLSA = (LSA >> 1) / 64.0f;
			break;

			/// checkSum
		case 8:
			checkSum = cur;
			break;

		case 9:
			checkSum += (cur << 8);
			break;

		default:
			continue;

			break;
		}

		scanResponse[fsmState] = cur;
		fsmState++;
	}

	startTs = millis();
	uint16_t totCloudPackets = LSN * 3;
	uint16_t packageIndex = 0;

	while ((waitTime = millis() - startTs) <= timeout && packageIndex < totCloudPackets) {

		if (!_bined_serialdev->available()) {
			continue;
		}

		byte cur = 0;
		_bined_serialdev->readBytes(&cur, 1);

		scanResponse[fsmState + packageIndex] = cur;
		packageIndex += 1;
	}

	/// parse data cloud points
	float diffAngle = angleLSA - angleFSA;
	if (angleLSA < angleFSA)
		diffAngle = 360 - angleFSA + angleLSA;

	memset(temp, 0, sizeof(temp));

	for (int pnt = 0; pnt < LSN; pnt++, fsmState += 3) {
		uint8_t Si[3] = { scanResponse[fsmState + 0], scanResponse[fsmState + 1], scanResponse[fsmState + 2] };

		uint16_t ligthtIntensity = Si[0] + (Si[1] & 0x03) * 256;
		uint16_t distance = (Si[2] << 6) + (Si[1] >> 2);

		float angleCorrect = distance ? atan(21.8 * (155.3 - distance) / (155.3 * distance)) * 180 / PI : 0;
		float angle = LSN > 1 ? angleFSA + pnt * diffAngle / (LSN - 1) : angleFSA;
		angle += angleCorrect;

		angle = angle <= 0 ? angle += 360 : angle;
		angle = angle > 360 ? angle -= 360 : angle;

		checkSumCal ^= (Si[0] & 0xff) + (0x00 << 8);
		checkSumCal ^= (Si[2] & 0xff) + (Si[1] << 8);

		// es: [{angle: 37.2, distance: 100},  {angle: 37.5, distance: 150},  {angle: 37.9, distance: 200}] --> {angle: 37, distance: 100} is what I want
		if (temp[(int)angle - 1] == 0) {
			temp[(int)angle - 1] = distance;

			distanceList[(int)angle - 1] = distance;
			lightList[(int)angle - 1] = ligthtIntensity;
		}
		else {
			if (distance >= temp[(int)angle - 1])
				continue;

			temp[(int)angle - 1] = distance;

			distanceList[(int)angle - 1] = distance;
			lightList[(int)angle - 1] = ligthtIntensity;
		}
	}

	uint16_t ctlsn = ((LSN & 0xff) + (CT << 8));
	//uint16_t ctlsn = (CT & 0xff) + (LSN << 8);
	checkSumCal ^= ctlsn;

	//Serial.printf("LSN = %d \n", LSN);
	//printf("%s \t ( %d %d )\n", checkSumCal == checkSum ? "checksum ok" : "checksum does not match", checkSumCal, checkSum);
	return RESULT_OK; // checksum not working so always return OK
}