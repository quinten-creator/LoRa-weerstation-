#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <CayenneLPP.h>

#include "DHT.h"
#define DHTPIN 5     //DHT pin 
#define DHTTYPE DHT11   // DHT22 or DHT11
DHT dht(DHTPIN, DHTTYPE); 

#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;

Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// LoRaWAN NwkSKey, network session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const PROGMEM u1_t NWKSKEY[16] = { 0x97, 0x75, 0x56, 0xCC, 0x21, 0x72, 0xE0, 0x51, 0x03, 0x27, 0x8C, 0x07, 0xD0, 0x0F, 0xAB, 0x78 };
// LoRaWAN AppSKey, application session key
// This is the default Semtech key, which is used by the prototype TTN
// network initially.
//ttn
static const u1_t PROGMEM APPSKEY[16] = { 0x9A, 0xCC, 0xF1, 0x10, 0x34, 0x70, 0x6A, 0x50, 0xA6, 0xE2, 0x52, 0xEF, 0x42, 0x57, 0x0C, 0x9A };
//
// LoRaWAN end-device address (DevAddr)
// See http://thethingsnetwork.org/wiki/AddressSpace
// ttn
static const u4_t DEVADDR = 0x26011DFC;


// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

static uint8_t mydata[] = "abcdefg";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7},
};

void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(": ");
      switch(ev) {
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
          os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
  
void payload() {
  CayenneLPP lpp(51);
  lpp.reset();   
//Fetch your sensor data
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  if (isnan(humidity) || isnan(temperature) ) {
  Serial.println("Failed to read from DHT sensor!");
    return; 
  } 
//End fetching your sensor data   

//Setup Cayenne LPP payload replace CHANNEL with the channel number and value with your sensor data
  //lpp.addDigitalInput(CHANNEL, uint8_t value);
  //lpp.addDigitalOutput(CHANNEL, uint8_t value);
  //lpp.addAbalogInput(CHANNEL, float value);
  //lpp.addAnalogOutput(CHANNEL, float value);
  //lpp.addLuminosity(CHANNEL, uint16_t lux);
  //lpp.addPresence(CHANNEL, uint8_tvalue);
  //lpp.addTemperature(CHANNEL, float celcius);
  //lpp.addRelativeHumidity(CHANNEL, float rh);
  //lpp.addAccelerometer(CHANNEL, float x, float y, float z);
  //lpp.addBarometricPressure(CHANNEL, float hpa);
  //lpp.addGyrometer(CHANNEL, floatx, float y, float z);
  //lpp.addGPS(CHANNEL, float latitude, float longitude, float meters);

//Sending your sensor data to the Cayenne dashboard    
  lpp.addTemperature(1, temperature);
  lpp.addRelativeHumidity(2, humidity);
  lpp.addTemperature(3, bmp.readTemperature());
  lpp.addBarometricPressure(4, bmp.readPressure()/100);
  LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
//End sending your sensor data to the Cayenne dashboard  

// Serial print Payload data for debugging  
  Serial.print("Humidity DHT11 = ");  
  Serial.print(humidity);  
  Serial.print(" %\t");
  Serial.print("Temperature DHT11 = ");
  Serial.print(temperature);
  Serial.println(" *C "); 

  Serial.print(F("Temperature bmp280 = "));
  Serial.print(bmp.readTemperature());
  Serial.println(" *C");

  Serial.print(F("Pressure bmp280 = "));
  Serial.print(bmp.readPressure()/100);
  Serial.println(" hPa");

  Serial.println();
  
  Serial.println(F("Packet queued"));
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {  
    payload();
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
   Serial.begin(115200);
   Serial.println(F("Starting"));
   //Start sensor setup
   dht.begin();

   if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1) delay(10);

}

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    // NA-US channels 0-71 are configured automatically   
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // Disable channels for testing on 1ch gateways uncomment the channels you dont use.
    // LMIC_disableChannel(0);    
    // LMIC_disableChannel(1);
    // LMIC_disableChannel(2);
    // LMIC_disableChannel(3);
    // LMIC_disableChannel(4);
    // LMIC_disableChannel(5);
    // LMIC_disableChannel(6);
    // LMIC_disableChannel(7);
    // LMIC_disableChannel(8);
     
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
