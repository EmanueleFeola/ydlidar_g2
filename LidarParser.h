#include "v8stdint.h"
#include "Arduino.h"

#define LIDAR_CMD_SYNC_BYTE                 0xA5
#define LIDAR_CMD_STOP                      0x65
#define LIDAR_CMD_SCAN                      0x60
#define LIDAR_SCAN_FREQ						0X0D
#define LIDAR_SCAN_FREQ_RED_01              0X0A
#define LIDAR_SCAN_FREQ_INC_01				0X09

#define LIDAR_ANS_TYPE_DEVINFO              0x4
#define LIDAR_ANS_TYPE_DEVHEALTH            0x6
#define LIDAR_ANS_TYPE_GETFREQ              0x4
#define LIDAR_ANS_SYNC_BYTE1                0xA5
#define LIDAR_ANS_SYNC_BYTE2                0x5A
#define LIDAR_ANS_TYPE_MEASUREMENT          0x81

#define LIDAR_CMD_GET_DEVICE_HEALTH         0x91
#define LIDAR_CMD_GET_DEVICE_INFO           0x90

#define PackageSampleBytes 2
#define PackageSampleMaxLngth 0x80
#define Node_Default_Quality (10<<2)
#define Node_Sync 1
#define Node_NotSync 2
#define PackagePaidBytes 10
#define PH 0x55AA

#define LIDAR_RESP_MEASUREMENT_SYNCBIT        (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT  2
#define LIDAR_RESP_MEASUREMENT_CHECKBIT       (0x1<<0)
#define LIDAR_RESP_MEASUREMENT_ANGLE_SHIFT    1
#define LIDAR_RESP_MEASUREMENT_ANGLE_SAMPLE_SHIFT    8

struct lidar_ans_header {
	uint8_t  syncByte1;
	uint8_t  syncByte2;
	uint32_t size : 30;
	uint32_t subType : 2;
	uint8_t  type;
} __attribute__((packed));

struct cmd_packet {
	uint8_t syncByte;
	uint8_t cmd_flag;
	uint8_t size;
	uint8_t data;
} __attribute__((packed));

struct node_info {
	uint8_t    sync_quality;
	uint16_t   angle_q6_checkbit;
	uint16_t   distance_q2;
} __attribute__((packed));

struct freq_info {
	uint32_t   freq;
} __attribute__((packed));

struct device_health {
	uint8_t   status;
	uint16_t  error_code;
} __attribute__((packed));


struct device_info {
	uint8_t   model;
	uint16_t  firmware_version;
	uint8_t   hardware_version;
	uint8_t   serialnum[16];
} __attribute__((packed));

struct node_package {
	uint16_t  package_Head;
	uint8_t   package_CT;
	uint8_t   nowPackageNum;
	uint16_t  packageFirstSampleAngle;
	uint16_t  packageLastSampleAngle;
	uint16_t  checkSum;
	uint16_t  packageSampleDistance[PackageSampleMaxLngth];
} __attribute__((packed));


struct scanPoint {
	uint8_t quality;
	float 	angle;
	float 	distance;
	bool    startBit;
};

typedef enum {
	CT_Normal = 0,
	CT_RingStart = 1,
	CT_Tail,
}CT;

class LidarParser
{
public:
	enum {
		SERIAL_BAUDRATE = 230400,
		RX_SERIAL_ESP = 19,
		TX_SERIAL_ESP = 18,
		DEFAULT_TIMEOUT = 500,
	};
	LidarParser();

	uint16_t distanceList[360]; // used to store cloud points distance
	uint16_t lightList[360]; // used to store cloud points light intensity
	// needed to store temporary cloud points distance (in order to store in distanceList the minimum distance for each degree)
	// example: [{angle: 37.2, distance: 100},  {angle: 37.5, distance: 150},  {angle: 37.9, distance: 200}] --> {angle: 37, distance: 100} is what I want
	uint16_t temp[360]; 

	bool begin(HardwareSerial& serialobj);
	bool isOpen(void);

	result_t getHealth(device_health& health, uint32_t timeout = DEFAULT_TIMEOUT);
	result_t getDeviceInfo(device_info& info, uint32_t timeout = DEFAULT_TIMEOUT);
	result_t startScan(uint32_t timeout = DEFAULT_TIMEOUT * 2);
	result_t stop(void);
	result_t waitScanDot(uint32_t timeout = DEFAULT_TIMEOUT);
	result_t getScanFreq(freq_info& freq, uint32_t timeout);
	result_t increaseFreq(freq_info& freq, uint32_t timeout);
	result_t decreaseFreq(freq_info& freq, uint32_t timeout);

protected:
	result_t sendCommand(uint8_t cmd, const void* payload = NULL, size_t payloadsize = 0);
	result_t waitResponseHeader(lidar_ans_header* header, uint32_t timeout = DEFAULT_TIMEOUT);

protected:
	HardwareSerial* _bined_serialdev;
};