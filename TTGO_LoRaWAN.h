#ifndef TTGO_LoraWAN_h
#define TTGO_LoraWAN_h

#include <arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "./project_config/project_config.h"

#if SERIAL_DEBUG_ENABLED
	#define DEBUG_PRINT(str, args...) Serial.printf("%ld: " str "\n", millis(), ##args);
	#define DEBUG_PRINT_SESSION_KEY_TEMPLATE "%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X-%02X"
	#define DEBUG_PRINT_SESSION_KEY_ARGS(sKey) sKey[0], sKey[1], sKey[2], sKey[3], sKey[4], sKey[5], sKey[6], sKey[7], sKey[8], sKey[9], sKey[10], sKey[11], sKey[12], sKey[13], sKey[14], sKey[15]
#endif

typedef void send_data_callback_t(bool);

class TTGO_LoRaWAN {
	public:
		TTGO_LoRaWAN();
		void begin();
		#if DEEP_SLEEP_ENABLED
		bool canGoDeepSleep(int sleepTimeInSeconds);
		void goDeepSleep(int sleepTimeInSeconds);
		bool hasSlept();
		#endif
		void sendData(u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed, send_data_callback_t *callback);
		void waitForCriticalJobs(unsigned long timeInMilliseconds);
	private:
		#if DEEP_SLEEP_ENABLED
		void _saveLmicData(int sleepTimeInSeconds);
		void _goDeepSleep(int sleepTimeInSeconds);
		#endif
		static void _halFailureHandler(const char *file, u2_t line);
		static void _sendDataCallback(void *pUserData, int fSuccess);
};

#endif
