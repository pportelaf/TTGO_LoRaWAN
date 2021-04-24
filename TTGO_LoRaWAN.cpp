#include "TTGO_LoRaWAN.h"

RTC_DATA_ATTR lmic_t rtcDataLmic;
RTC_DATA_ATTR bool rtcDataHasSlept = false;

const lmic_pinmap lmic_pins = {
    .nss = NSS_GPIO,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = RESET_GPIO,
    .dio = {DIO0_GPIO, DIO1_GPIO, DIO2_GPIO}
};

#if SERIAL_DEBUG_LMIC_EVENTS_ENABLED && SERIAL_DEBUG_ENABLED
void onEvent (ev_t ev) {
	DEBUG_PRINT("Event received with os time %ld", os_getTime())

    switch(ev) {
        case EV_SCAN_TIMEOUT:
			DEBUG_PRINT("EV_SCAN_TIMEOUT")
            break;
        case EV_JOINING:
			DEBUG_PRINT("EV_JOINING")
            break;
        case EV_JOINED:
			{
				DEBUG_PRINT("EV_JOINED")
				u4_t netid = 0;
				devaddr_t devaddr = 0;
				u1_t nwkKey[16];
				u1_t artKey[16];
				LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
				DEBUG_PRINT("netid: %i", netid)
				DEBUG_PRINT("devaddr: %X", devaddr)
				DEBUG_PRINT("AppSKey: "DEBUG_PRINT_SESSION_KEY_TEMPLATE, DEBUG_PRINT_SESSION_KEY_ARGS(artKey))
				DEBUG_PRINT("NwkSKey: "DEBUG_PRINT_SESSION_KEY_TEMPLATE, DEBUG_PRINT_SESSION_KEY_ARGS(nwkKey))
      		}
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
     		// size, we don't use it in this example.
            // LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
        	DEBUG_PRINT("EV_JOIN_FAILED")
            break;
        case EV_REJOIN_FAILED:
        	DEBUG_PRINT("EV_REJOIN_FAILED")
            break;
        case EV_TXCOMPLETE:
        	DEBUG_PRINT("EV_TXCOMPLETE (includes waiting for RX windows)")
            if (LMIC.txrxFlags & TXRX_ACK)
        		DEBUG_PRINT("Received ack")
            if (LMIC.dataLen) {
            	DEBUG_PRINT("Received %i bytes of payload", LMIC.dataLen)
            }
            break;
        case EV_RESET:
            DEBUG_PRINT("EV_RESET")
            break;
        case EV_RXCOMPLETE:
        	DEBUG_PRINT("EV_RXCOMPLETE")
            break;
        case EV_LINK_DEAD:
            DEBUG_PRINT("EV_LINK_DEAD")
            break;
        case EV_LINK_ALIVE:
            DEBUG_PRINT("EV_LINK_ALIVE")
            break;
        case EV_TXSTART:
            DEBUG_PRINT("EV_TXSTART")
            break;
        case EV_TXCANCELED:
            DEBUG_PRINT("EV_TXCANCELED")
            break;
        case EV_RXSTART:
            DEBUG_PRINT("EV_RXSTART")
            break;
        case EV_JOIN_TXCOMPLETE:
        	DEBUG_PRINT("EV_JOIN_TXCOMPLETE: no JoinAccept")
            break;
        default:
        	DEBUG_PRINT("Unknown event: %i", ev)
            break;
    }
}
#endif

TTGO_LoRaWAN::TTGO_LoRaWAN() {
	hal_set_failure_handler(_halFailureHandler);
}

void TTGO_LoRaWAN::begin() {
	// LMIC init
	os_init();
	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

	if (hasSlept()) {
		LMIC = rtcDataLmic;
	}
}

#if DEEP_SLEEP_ENABLED
void TTGO_LoRaWAN::goDeepSleep(int sleepTimeInSeconds) {
	_saveLmicData(sleepTimeInSeconds);
	_goDeepSleep(sleepTimeInSeconds);
}

bool TTGO_LoRaWAN::canGoDeepSleep(int sleepTimeInSeconds) {
	return !((LMIC.opmode & OP_TXRXPEND) || os_queryTimeCriticalJobs(ms2osticksRound((sleepTimeInSeconds * 1000))));
}


bool TTGO_LoRaWAN::hasSlept() {
	return rtcDataHasSlept;
}
#endif

void TTGO_LoRaWAN::sendData(u1_t port, xref2u1_t data, u1_t dlen, u1_t confirmed, send_data_callback_t *callback) {
	#if SERIAL_DEBUG_ENABLED
		DEBUG_PRINT("Sending %i bytes of data using port number %i", dlen, port)
	#endif

	if (LMIC.opmode & OP_TXRXPEND) {
		#if SERIAL_DEBUG_ENABLED
			DEBUG_PRINT("OP_TXRXPEND, not sending")
		#endif

		callback(false);
		return;
	}

	LMIC_sendWithCallback(port, data, dlen, confirmed, _sendDataCallback, (void*)callback);
}

void TTGO_LoRaWAN::waitForCriticalJobs(unsigned long timeInMilliseconds) {
	#if SERIAL_DEBUG_ENABLED
		DEBUG_PRINT("Start wait for critical jobs %ld milliseconds", timeInMilliseconds)
	#endif

	while (os_queryTimeCriticalJobs(ms2osticksRound((timeInMilliseconds)))) {
		os_runloop_once();
	}

	#if SERIAL_DEBUG_ENABLED
		DEBUG_PRINT("End wait for critical jobs %ld milliseconds", timeInMilliseconds)
	#endif
}

void TTGO_LoRaWAN::_saveLmicData(int sleepTimeInSeconds) {
	#if SERIAL_DEBUG_ENABLED
		DEBUG_PRINT("Saving LMIC data in RTC")
	#endif

	rtcDataLmic = LMIC;

	// ESP32 can't track millis during DeepSleep and no option to advanced millis after DeepSleep.
    // Therefore reset DutyCyles
    unsigned long now = millis();

	for (int i = 0; i < MAX_BANDS; i++) {
		#if SERIAL_DEBUG_ENABLED
			DEBUG_PRINT("Band last channel %i, avail %ld", rtcDataLmic.bands[i].lastchnl, rtcDataLmic.bands[i].avail)
		#endif
        ostime_t correctedAvail = rtcDataLmic.bands[i].avail - ((now / 1000.0 + sleepTimeInSeconds) * OSTICKS_PER_SEC);
        if (correctedAvail < 0) {
            correctedAvail = 0;
        }
        rtcDataLmic.bands[i].avail = correctedAvail;
    }

    rtcDataLmic.globalDutyAvail = rtcDataLmic.globalDutyAvail - ((now / 1000.0 + sleepTimeInSeconds) * OSTICKS_PER_SEC);
    if (rtcDataLmic.globalDutyAvail < 0) {
        rtcDataLmic.globalDutyAvail = 0;
    }
}

void TTGO_LoRaWAN::_goDeepSleep(int sleepTimeInSeconds) {
	#if SERIAL_DEBUG_ENABLED
		DEBUG_PRINT("Entering deep sleep during %ld seconds", sleepTimeInSeconds)
	#endif
	rtcDataHasSlept = true;

    Serial.flush();
    esp_sleep_enable_timer_wakeup(sleepTimeInSeconds * 1000000);
    esp_deep_sleep_start();
}

void TTGO_LoRaWAN::_sendDataCallback(void *pUserData, int fSuccess) {
	#if SERIAL_DEBUG_ENABLED
		DEBUG_PRINT("Send data callback called with status %i", fSuccess != 0)
	#endif

	send_data_callback_t *callback = (send_data_callback_t *)pUserData;

	if (callback != NULL) {
		callback(fSuccess != 0);
	}
}

void TTGO_LoRaWAN::_halFailureHandler(const char *file, u2_t line) {
	#if SERIAL_DEBUG_ENABLED
		DEBUG_PRINT("FAILURE %s:%i", file, line)
		Serial.flush();
	#endif

	ESP.restart();
}
