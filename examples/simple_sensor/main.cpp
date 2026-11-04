#include "SensorMesh.h"

#ifdef DISPLAY_CLASS
  #include "UITask.h"
  static UITask ui_task(display);
#endif

#ifndef SENSOR_READ_INTERVAL_SECS
#define SENSOR_READ_INTERVAL_SECS  60
#endif

class MyMesh : public SensorMesh {
public:
  MyMesh(mesh::MainBoard& board, mesh::Radio& radio, mesh::MillisecondClock& ms, mesh::RNG& rng, mesh::RTCClock& rtc, mesh::MeshTables& tables)
     : SensorMesh(board, radio, ms, rng, rtc, tables), 
       battery_data(12*24, 5*60)    // 24 hours worth of battery data, every 5 minutes
  {
  }

protected:
  /* ========================== custom logic here ========================== */
  Trigger low_batt, critical_batt, periodic;
  TimeSeriesData  battery_data;

  void onSensorDataRead() override {
    float batt_voltage = getVoltage(TELEM_CHANNEL_SELF);

    battery_data.recordData(getRTCClock(), batt_voltage);   // record battery
    alertIf(batt_voltage < 3.4f, critical_batt, HIGH_PRI_ALERT, "Battery is critical!");
    alertIf(batt_voltage < 3.6f, low_batt, LOW_PRI_ALERT, "Battery is low");
    alertIf(getRTCClock()->getCurrentTime() - last_read_timeJMF >= SENSOR_READ_INTERVAL_SECS, periodic, LOW_PRI_ALERT, "Periodic JMF");
  }

  int querySeriesData(uint32_t start_secs_ago, uint32_t end_secs_ago, MinMaxAvg dest[], int max_num) override {
    battery_data.calcMinMaxAvg(getRTCClock(), start_secs_ago, end_secs_ago, &dest[0], TELEM_CHANNEL_SELF, LPP_VOLTAGE);
    return 1;
  }

  bool handleCustomCommand(uint32_t sender_timestamp, char* command, char* reply) override {
    if (strcmp(command, "magic") == 0) {    // example 'custom' command handling
      strcpy(reply, "**Magic now done**");
      return true; // handled
    } else if (strcmp(command, "temp") == 0) {
      CayenneLPP telem(25);
      sensors.querySensors(TELEM_PERM_ENVIRONMENT, telem);
      uint8_t *buf = telem.getBuffer();

      while (*buf != LPP_TEMPERATURE) {
        buf++;
      }
      buf++;

      int16_t temperature = (*buf << 8) | *(buf + 1);

      sprintf(reply, "measured temperature : %d.%d °C", temperature / 10, temperature % 10);
      // strcpy(reply, "measured temperature: ");
      return true; // handled
    }
    return false;  // not handled
  }
  /* ======================================================================= */
};

StdRNG fast_rng;
SimpleMeshTables tables;

MyMesh the_mesh(board, radio_driver, *new ArduinoMillis(), fast_rng, rtc_clock, tables);

void halt() {
  while (1) ;
}

static char command[160];

static uint32_t send_temperature_timer = 0;

void setup() {
  Serial.begin(115200);
  delay(1000);

  board.begin();

#ifdef DISPLAY_CLASS
  if (display.begin()) {
    display.startFrame();
    display.print("Please wait...");
    display.endFrame();
  }
#endif

  if (!radio_init()) { halt(); }

  fast_rng.begin(radio_driver.getRngSeed());

  FILESYSTEM* fs;
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  InternalFS.begin();
  InternalFS.format();    // <--- L'ÉLECTROCHOC EST ICI
  fs = &InternalFS;
  IdentityStore store(InternalFS, "");
#elif defined(ESP32)
  SPIFFS.begin(true);
  fs = &SPIFFS;
  IdentityStore store(SPIFFS, "/identity");
#elif defined(RP2040_PLATFORM)
  LittleFS.begin();
  fs = &LittleFS;
  IdentityStore store(LittleFS, "/identity");
  store.begin();
#else
  #error "need to define filesystem"
#endif
  if (!store.load("_main", the_mesh.self_id)) {
    MESH_DEBUG_PRINTLN("Generating new keypair");
    the_mesh.self_id = radio_new_identity();   // create new random identity
    int count = 0;
    while (count < 10 && (the_mesh.self_id.pub_key[0] == 0x00 || the_mesh.self_id.pub_key[0] == 0xFF)) {  // reserved id hashes
      the_mesh.self_id = radio_new_identity(); count++;
    }
    store.save("_main", the_mesh.self_id);
  }

  Serial.print("Sensor ID: ");
  mesh::Utils::printHex(Serial, the_mesh.self_id.pub_key, PUB_KEY_SIZE); Serial.println();

  command[0] = 0;

  sensors.begin();

  the_mesh.begin(fs);

#ifdef DISPLAY_CLASS
  ui_task.begin(the_mesh.getNodePrefs(), FIRMWARE_BUILD_DATE, FIRMWARE_VERSION);
#endif

  // send out initial zero hop Advertisement to the mesh//Marwane
#if ENABLE_ADVERT_ON_BOOT == 1
  the_mesh.sendSelfAdvertisement(16000, true);
#endif

  // setup next time to send temperature
  send_temperature_timer = the_mesh.futureMillis(30000);
}

void loop() {
  int len = strlen(command);
  while (Serial.available() && len < sizeof(command)-1) {
    char c = Serial.read();
    if (c != '\n') {
      command[len++] = c;
      command[len] = 0;
    }
    Serial.print(c);
  }
  if (len == sizeof(command)-1) {  // command buffer full
    command[sizeof(command)-1] = '\r';
  }

  if (len > 0 && command[len - 1] == '\r') {  // received complete line
    command[len - 1] = 0;  // replace newline with C string null terminator
    char reply[160];
    the_mesh.handleCommand(0, command, reply);  // NOTE: there is no sender_timestamp via serial!
    if (reply[0]) {
      Serial.print("  -> ");
      Serial.println(reply);
    }

    command[0] = 0; // reset command buffer
  }

  // Test Code to send periodically temperature to neighbours
  if (send_temperature_timer && the_mesh.millisHasNowPassed(send_temperature_timer)) {

    CayenneLPP telem(25);
    sensors.querySensors(TELEM_PERM_ENVIRONMENT, telem);
    uint8_t *buf = telem.getBuffer();

    while (*buf != LPP_TEMPERATURE) {
      buf++;
    }
    buf++;

    int16_t temperature = (*buf << 8) | *(buf + 1);

//    uint8_t raw_data[] = {*buf, *(buf+1)};
//    uint8_t companion_path[] = {0xd9};
//    uint8_t companion_path_len = 1;
    uint8_t raw_data[10]={0,1,2,3,4,5,6,7,8,9};
    // mesh::Packet* packet = the_mesh.createRawData(raw_data, 10);

/// JMF
    mesh::Packet *packet = the_mesh.createControlData(raw_data, 10);
    Serial.println("Temperature sent");
      the_mesh.sendFlood(packet); // ,0,1); // path hash size = 1 byte ; current path hash count = 0
//    the_mesh.sendDirect(packet,companion_path,companion_path_len);

    send_temperature_timer = the_mesh.futureMillis(30000); // next temperature send in 30 sec
  }

  the_mesh.loop();
  sensors.loop();
#ifdef DISPLAY_CLASS
  ui_task.loop();
#endif
  rtc_clock.tick();
}
