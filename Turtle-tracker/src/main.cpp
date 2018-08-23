#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <CayenneLPP.h>
#include <LowPower.h>
#include <NewPing.h>

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8] = { 0x25, 0x0E, 0x01, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getArtEui (u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8] = { 0x60, 0x24, 0x16, 0x6C, 0xA2, 0xD0, 0x41, 0x00 };

void os_getDevEui (u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x41, 0xFD, 0xD9, 0xD8, 0x34, 0x2A, 0xB3, 0x74, 0x2E, 0x6C, 0xDF, 0xA4, 0xD4, 0x3D, 0xDC, 0xED };

void os_getDevKey (u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}


Adafruit_BMP085 bmp;

NewPing sonar(5, 6 , 400);

//static uint8_t mydata[] = "first value";
//static uint8_t mydata[] = 0x01670110;
CayenneLPP lpp(51);

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 30;

// Pin mapping Anarduino
const lmic_pinmap lmic_pins = {
  .nss = 10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 3,
  .dio = {2, 8, 7},
};


void do_send(osjob_t* j);

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));

      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.println(F("Received "));
        Serial.println(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    default:
      Serial.println(F("Unknown event"));
      break;
  }
}

void do_send(osjob_t* j) {
  lpp.reset();
  lpp.addTemperature(1, bmp.readTemperature());
  lpp.addBarometricPressure(2, bmp.readPressure());
  lpp.addBarometricPressure(3, bmp.readAltitude());
  lpp.addBarometricPressure(4, sonar.ping_cm());

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare upstream data transmission at the next possible time.
    //LMIC_setTxData2(1, mydata, sizeof(mydata) - 1, 0);
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    Serial.println(F("Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
  //LowPower.powerDown(SLEEP_2S, ADC_OFF, BOD_OFF);
}

void setup() {

  //while (!Serial);
  Serial.begin(9600);

  if (!bmp.begin()) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
	while (1) {}
  }

  Serial.println(F("Starting"));

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  LMIC_selectSubBand(1);
  //Disable FSB1, channels 0-7
  for (int i = 0; i < 7; i++) {
    if (i != 10)
      LMIC_disableChannel(i);

  }
  //Disable FSB2-8, channels 16-72
  for (int i = 16; i < 73; i++) {
    if (i != 10)
      LMIC_disableChannel(i);

  }
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  // delay(1000);

  os_runloop_once();
}
