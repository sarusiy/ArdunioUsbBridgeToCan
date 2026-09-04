/*
 * Arduino Uno + MCP2515/TJA1050 CAN module.
 *
 * Default build: simulated OBD-II ECU (see below).
 * Echo test build (previous Phase 1 HW connectivity test) is preserved
 * behind the ECHO_TEST_MODE build flag - build with the "uno_echo_test"
 * PlatformIO environment to get it back:
 *   platformio run -e uno_echo_test -t upload
 *
 * Wiring (see Doc/electrical drawing.vsdx in JC-ESP32P4-M3):
 *   MCP2515 SCK  -> Uno D13
 *   MCP2515 SI   -> Uno D11 (MOSI)
 *   MCP2515 SO   -> Uno D12 (MISO)
 *   MCP2515 CS   -> Uno D10
 *   MCP2515 VCC  -> 5V, GND -> GND
 *   CAN-H/CAN-L  -> shared bus with the ESP32-P4's MCP2515
 *   J1 jumper closed on this module for 120 ohm bus termination
 */
#include <SPI.h>
#include <SoftwareSerial.h>
#include <mcp_can.h>
#include <string.h>
#include <math.h>

#define CAN_CS_PIN 10

MCP_CAN CAN(CAN_CS_PIN);

/* ===================== Simulated GPS (NMEA 0183) =====================
 * Uno has only one hardware UART (Serial), already used by the CAN bridge,
 * so the simulated GPS feed goes out over SoftwareSerial instead.
 * Wiring (see Doc/GPS_SIMULATION_AND_INTEGRATION_REQUIREMENTS.md in
 * JC-ESP32P4-M3): D3 (TX) -> P4 GPIO34 (RX) through a 10k/15k divider,
 * since this is a 5V-logic TX into a 3.3V-only P4 input. D2 (RX) is wired
 * but reserved/unused for now. */
#define GPS_TX_PIN 3
#define GPS_RX_PIN 2
#define GPS_BAUD 9600
#define GPS_UPDATE_INTERVAL_MS 1000UL
#define GPS_NO_FIX_DURATION_MS 5000UL

/* Simulated route inside Tel Aviv. The coordinates are synthetic waypoints,
 * but the timing and reported speed are kept consistent so the map looks sane. */
#define GPS_ROUTE_PERIOD_S 240.0

static SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
static unsigned long s_gps_boot_time;

struct GpsFix {
    bool valid;
    double lat;
    double lon;
    float speed_kmh;
    float heading_deg;
};

struct GpsWaypoint {
    double lat;
    double lon;
};

static const GpsWaypoint GPS_ROUTE[] = {
    {32.085300, 34.781800},
    {32.086100, 34.783300},
    {32.087200, 34.784600},
    {32.088500, 34.783400},
    {32.089000, 34.781300},
    {32.087800, 34.779700},
    {32.086200, 34.778900},
    {32.085300, 34.781800},
};

static float bearing_deg(double lat1, double lon1, double lat2, double lon2)
{
    double phi1 = radians(lat1);
    double phi2 = radians(lat2);
    double delta_lon = radians(lon2 - lon1);
    double y = sin(delta_lon) * cos(phi2);
    double x = cos(phi1) * sin(phi2) - sin(phi1) * cos(phi2) * cos(delta_lon);
    double bearing = fmod(degrees(atan2(y, x)) + 360.0, 360.0);
    return (float)bearing;
}

/* Drives a point around a closed loop; no fix is reported for the first
 * GPS_NO_FIX_DURATION_MS so P4/Android "waiting for GPS" handling can be
 * exercised on every boot. */
static GpsFix simulate_gps_fix(unsigned long now)
{
    GpsFix fix;
    fix.valid = (now - s_gps_boot_time) >= GPS_NO_FIX_DURATION_MS;

    const size_t waypoint_count = sizeof(GPS_ROUTE) / sizeof(GPS_ROUTE[0]);
    const size_t segment_count = waypoint_count - 1;
    double route_s = fmod((double)(now - s_gps_boot_time) / 1000.0, GPS_ROUTE_PERIOD_S);
    double segment_s = GPS_ROUTE_PERIOD_S / segment_count;
    size_t segment = (size_t)(route_s / segment_s);
    if (segment >= segment_count) {
        segment = segment_count - 1;
    }
    double t = (route_s - segment * segment_s) / segment_s;
    const GpsWaypoint &a = GPS_ROUTE[segment];
    const GpsWaypoint &b = GPS_ROUTE[segment + 1];

    fix.lat = a.lat + (b.lat - a.lat) * t;
    fix.lon = a.lon + (b.lon - a.lon) * t;
    fix.speed_kmh = fix.valid ? (18.0f + 4.0f * (float)sin(route_s * 0.12)) : 0.0f;
    fix.heading_deg = bearing_deg(a.lat, a.lon, b.lat, b.lon);

    return fix;
}

/* Converts a signed decimal-degree value to NMEA "ddmm.mmmm"/"dddmm.mmmm"
 * plus its hemisphere letter. Uses only integer formatting: avr-libc's
 * snprintf does not support %f without extra linker flags, so any %f here
 * silently produces garbage instead of a number. */
static void format_nmea_coord(double value, bool is_lat, char *out, size_t out_len, char *hemi)
{
    double abs_value = fabs(value);
    int degrees_part = (int)abs_value;
    double minutes_double = (abs_value - degrees_part) * 60.0;
    int minutes_int = (int)minutes_double;
    int minutes_frac = (int)((minutes_double - minutes_int) * 10000.0 + 0.5);
    if (minutes_frac >= 10000) {
        minutes_frac -= 10000;
        minutes_int += 1;
    }
    if (minutes_int >= 60) {
        minutes_int -= 60;
        degrees_part += 1;
    }

    if (is_lat) {
        *hemi = (value >= 0) ? 'N' : 'S';
        snprintf(out, out_len, "%02d%02d.%04d", degrees_part, minutes_int, minutes_frac);
    } else {
        *hemi = (value >= 0) ? 'E' : 'W';
        snprintf(out, out_len, "%03d%02d.%04d", degrees_part, minutes_int, minutes_frac);
    }
}

/* Formats a float with one decimal place using only integer snprintf, for
 * the same reason as format_nmea_coord above. */
static void format_fixed1(float value, char *out, size_t out_len)
{
    int whole = (int)value;
    int tenths = (int)((value - whole) * 10.0f + 0.5f);
    if (tenths >= 10) {
        tenths -= 10;
        whole += 1;
    }
    snprintf(out, out_len, "%d.%d", whole, tenths);
}

static uint8_t nmea_checksum(const char *sentence)
{
    uint8_t checksum = 0;
    for (const char *p = sentence; *p != '\0'; p++) {
        checksum ^= (uint8_t)*p;
    }
    return checksum;
}

static void send_nmea_sentence(const char *body)
{
    char checksum_hex[3];
    snprintf(checksum_hex, sizeof(checksum_hex), "%02X", nmea_checksum(body));
    gpsSerial.print('$');
    gpsSerial.print(body);
    gpsSerial.print('*');
    gpsSerial.println(checksum_hex);
}

/* Emits one $GPRMC and one $GPGGA sentence for the current simulated fix,
 * using a fake UTC clock that starts at 12:00:00 on boot. */
static void send_gps_sentences(unsigned long now, const GpsFix &fix)
{
    unsigned long elapsed_s = now / 1000UL;
    unsigned long utc_seconds = (12UL * 3600UL + elapsed_s) % 86400UL;
    unsigned int hh = (unsigned int)(utc_seconds / 3600UL);
    unsigned int mm = (unsigned int)((utc_seconds % 3600UL) / 60UL);
    unsigned int ss = (unsigned int)(utc_seconds % 60UL);

    char lat_str[16];
    char lon_str[16];
    char lat_hemi;
    char lon_hemi;
    format_nmea_coord(fix.lat, true, lat_str, sizeof(lat_str), &lat_hemi);
    format_nmea_coord(fix.lon, false, lon_str, sizeof(lon_str), &lon_hemi);

    char speed_str[8];
    char heading_str[8];
    format_fixed1(fix.speed_kmh / 1.852f, speed_str, sizeof(speed_str));
    format_fixed1(fix.heading_deg, heading_str, sizeof(heading_str));

    char body[96];
    snprintf(body, sizeof(body), "GPRMC,%02u%02u%02u,%c,%s,%c,%s,%c,%s,%s,040926,,,",
             hh, mm, ss, fix.valid ? 'A' : 'V',
             lat_str, lat_hemi, lon_str, lon_hemi,
             speed_str, heading_str);
    send_nmea_sentence(body);

    uint8_t fix_quality = fix.valid ? 1 : 0;
    uint8_t sats = fix.valid ? 8 : 0;
    snprintf(body, sizeof(body), "GPGGA,%02u%02u%02u,%s,%c,%s,%c,%d,%d,0.9,50.0,M,17.0,M,,",
             hh, mm, ss, lat_str, lat_hemi, lon_str, lon_hemi, fix_quality, sats);
    send_nmea_sentence(body);
}

static void init_gps()
{
    gpsSerial.begin(GPS_BAUD);
    s_gps_boot_time = millis();
}

/* Call once per loop() iteration in every build mode; internally rate-limits
 * itself to GPS_UPDATE_INTERVAL_MS so callers don't need their own timer. */
static void tick_gps(unsigned long now)
{
    static unsigned long last_gps_update = 0;
    if (now - last_gps_update < GPS_UPDATE_INTERVAL_MS) {
        return;
    }
    last_gps_update = now;

    GpsFix fix = simulate_gps_fix(now);
    send_gps_sentences(now, fix);
}

static void init_can()
{
    pinMode(LED_BUILTIN, OUTPUT);

    /* Blink on every retry so the board looks alive even before the MCP2515 answers. */
    while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        Serial.println("MCP2515 init failed, retrying...");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(150);
        digitalWrite(LED_BUILTIN, LOW);
        delay(350);
    }
    CAN.setMode(MCP_NORMAL);
    Serial.println("MCP2515 ready, normal mode, 500 kbps.");
}

#if ECHO_TEST_MODE
/* ===================== Phase 1: HW connectivity echo test =====================
 * Every second, sends an 8-char test message as a single CAN frame on
 * TEST_CAN_ID, and logs it over USB serial. Logs any CAN frame received
 * (expected: the JC-ESP32P4-M3 board echoing the same frame back on the
 * bus) and reports whether the echoed message matches what was last sent.
 */
#define TEST_CAN_ID 0x100
#define SEND_INTERVAL_MS 1000
#define TEST_MESSAGE_LEN 8

static char s_test_message[TEST_MESSAGE_LEN];
static uint8_t s_send_counter = 0;

/* Shifts the digit window each call, e.g. "01234567", "12345678", ... */
static void build_test_message()
{
    for (uint8_t i = 0; i < TEST_MESSAGE_LEN; i++) {
        s_test_message[i] = '0' + ((s_send_counter + i) % 10);
    }
    s_send_counter = (s_send_counter + 1) % 10;
}

static void send_test_message()
{
    /* Pin 13/LED_BUILTIN doubles as SPI SCK, whose idle-low state instantly
     * overrides digitalWrite() once CAN polling resumes; hold it with a
     * blocking delay so the "on" phase is actually visible. */
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);

    build_test_message();
    byte result = CAN.sendMsgBuf(TEST_CAN_ID, 0, TEST_MESSAGE_LEN, (uint8_t *)s_test_message);
    Serial.print("TX id=0x100 data='");
    Serial.write((const uint8_t *)s_test_message, TEST_MESSAGE_LEN);
    Serial.print("' -> ");
    Serial.println(result == CAN_OK ? "OK" : "ERR");
}

static void poll_can_rx()
{
    if (CAN.checkReceive() != CAN_MSGAVAIL) {
        return;
    }

    long unsigned int rxId;
    uint8_t len = 0;
    uint8_t buf[8];
    if (CAN.readMsgBuf(&rxId, &len, buf) != CAN_OK) {
        return;
    }

    Serial.print("RX id=0x");
    Serial.print(rxId, HEX);
    Serial.print(" len=");
    Serial.print(len);
    Serial.print(" data='");
    for (uint8_t i = 0; i < len; i++) {
        Serial.print((char)buf[i]);
    }
    Serial.print("' ");

    bool match = (rxId == TEST_CAN_ID) && (len == TEST_MESSAGE_LEN) &&
                 (memcmp(buf, s_test_message, TEST_MESSAGE_LEN) == 0);
    Serial.println(match ? "(echo matches last TX)" : "(echo MISMATCH)");
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for USB serial on boards that need it
    }
    init_can();
    init_gps();
}

void loop()
{
    static unsigned long last_send = 0;
    unsigned long now = millis();

    if (now - last_send >= SEND_INTERVAL_MS) {
        last_send = now;
        send_test_message();
    }

    poll_can_rx();
    tick_gps(now);
}

#else
/* ===================== Simulated OBD-II ECU (SAE J1979) =====================
 * Responds to standard OBD-II "show current data" (mode 0x01) requests on
 * the functional (0x7DF) or physical (0x7E0) request IDs, replying as ECU 1
 * on 0x7E8, per ISO 15765-4 single-frame format. Supports PIDs 0x00 (PID
 * support bitmask), 0x05 (coolant temp), 0x0C (RPM), 0x0D (speed), and
 * 0x11 (throttle position), with values that evolve over time to look like
 * a real driving cycle instead of staying static.
 *
 * Testable with any real OBD-II scan tool/adapter wired to the same bus.
 * The JC-ESP32P4-M3 firmware queries these PIDs while independently
 * capturing the generic periodic broadcasts below.
 */
#define OBD_REQUEST_ID_FUNCTIONAL 0x7DF
#define OBD_REQUEST_ID_PHYSICAL   0x7E0
#define OBD_RESPONSE_ID           0x7E8
#define OBD_MODE_CURRENT_DATA     0x01

#define PID_SUPPORTED_01_20  0x00
#define PID_COOLANT_TEMP     0x05
#define PID_RPM              0x0C
#define PID_SPEED            0x0D
#define PID_THROTTLE         0x11

#define STATUS_PRINT_INTERVAL_MS 2000
#define DRIVE_CYCLE_PERIOD_MS 20000UL
#define WARMUP_DURATION_MS 60000UL

/* Generic simulated broadcast IDs. These are test IDs, not definitions for
 * any real vehicle manufacturer. */
#define CAN_ID_ENGINE_STATE  0x120
#define CAN_ID_VEHICLE_STATE 0x180
#define CAN_ID_BODY_STATE    0x220
#define ENGINE_PERIOD_MS     20
#define VEHICLE_PERIOD_MS    50
#define BODY_PERIOD_MS       500

static unsigned long s_boot_time;

struct EcuState {
    uint16_t rpm;
    uint8_t speed_kmh;
    uint8_t throttle_pct;
    uint8_t coolant_c;
};

/* Simulates a repeating accelerate/cruise/decelerate cycle, plus a coolant
 * warm-up ramp from cold start, so values look like a real running engine. */
static EcuState simulate_ecu_state(unsigned long now)
{
    EcuState s;

    float phase = (now % DRIVE_CYCLE_PERIOD_MS) * (2.0 * PI / DRIVE_CYCLE_PERIOD_MS);
    float speed_f = 55.0f + 55.0f * sin(phase);
    if (speed_f < 0) {
        speed_f = 0;
    }
    s.speed_kmh = (uint8_t)speed_f;
    s.rpm = (uint16_t)(900.0f + speed_f * 30.0f);
    s.throttle_pct = (uint8_t)(10.0f + 40.0f * fabs(sin(phase)));

    float warmup = (float)(now - s_boot_time) / (float)WARMUP_DURATION_MS;
    if (warmup > 1.0f) {
        warmup = 1.0f;
    }
    s.coolant_c = (uint8_t)(20.0f + warmup * 70.0f);

    return s;
}

static void send_obd_response(uint8_t len, const uint8_t *payload)
{
    uint8_t data[8] = {0};
    if (len > 7) {
        len = 7;
    }
    data[0] = len;
    memcpy(&data[1], payload, len);
    CAN.sendMsgBuf(OBD_RESPONSE_ID, 0, 8, data);
}

static void send_periodic_broadcasts(unsigned long now, const EcuState &state)
{
    static unsigned long last_engine;
    static unsigned long last_vehicle;
    static unsigned long last_body;

    if (now - last_engine >= ENGINE_PERIOD_MS) {
        last_engine = now;
        uint16_t rpm_raw = state.rpm * 4;
        uint8_t data[8] = {
            (uint8_t)(rpm_raw >> 8), (uint8_t)rpm_raw,
            (uint8_t)(state.coolant_c + 40),
            (uint8_t)((uint16_t)state.throttle_pct * 255 / 100),
            0, 0, 0, 0
        };
        CAN.sendMsgBuf(CAN_ID_ENGINE_STATE, 0, sizeof(data), data);
    }

    if (now - last_vehicle >= VEHICLE_PERIOD_MS) {
        last_vehicle = now;
        uint16_t speed_centi_kmh = (uint16_t)state.speed_kmh * 100;
        uint8_t data[8] = {
            (uint8_t)(speed_centi_kmh >> 8), (uint8_t)speed_centi_kmh,
            state.speed_kmh > 0 ? 1u : 0u,
            0, 0, 0, 0, 0
        };
        CAN.sendMsgBuf(CAN_ID_VEHICLE_STATE, 0, sizeof(data), data);
    }

    if (now - last_body >= BODY_PERIOD_MS) {
        last_body = now;
        uint8_t data[8] = {
            state.speed_kmh == 0 ? 1u : 0u, /* simulated driver door */
            state.speed_kmh > 0 ? 1u : 0u,  /* simulated ignition */
            0, 0, 0, 0, 0, 0
        };
        CAN.sendMsgBuf(CAN_ID_BODY_STATE, 0, sizeof(data), data);
    }
}

static void handle_obd_request(uint8_t mode, uint8_t pid, const EcuState &state)
{
    if (mode != OBD_MODE_CURRENT_DATA) {
        return;
    }

    switch (pid) {
        case PID_SUPPORTED_01_20: {
            /* Bitmask for PIDs 0x01-0x20: we support 0x05, 0x0C, 0x0D, 0x11. */
            uint8_t payload[6] = { 0x41, PID_SUPPORTED_01_20, 0x08, 0x18, 0x80, 0x00 };
            send_obd_response(6, payload);
            Serial.println("OBD RX mode=01 pid=00 -> TX supported-PIDs bitmask");
            break;
        }
        case PID_COOLANT_TEMP: {
            uint8_t payload[3] = { 0x41, PID_COOLANT_TEMP, (uint8_t)(state.coolant_c + 40) };
            send_obd_response(3, payload);
            Serial.print("OBD RX mode=01 pid=05 -> TX coolant=");
            Serial.print(state.coolant_c);
            Serial.println(" C");
            break;
        }
        case PID_RPM: {
            uint16_t raw = state.rpm * 4;
            uint8_t payload[4] = { 0x41, PID_RPM, (uint8_t)(raw >> 8), (uint8_t)(raw & 0xFF) };
            send_obd_response(4, payload);
            Serial.print("OBD RX mode=01 pid=0C -> TX rpm=");
            Serial.println(state.rpm);
            break;
        }
        case PID_SPEED: {
            uint8_t payload[3] = { 0x41, PID_SPEED, state.speed_kmh };
            send_obd_response(3, payload);
            Serial.print("OBD RX mode=01 pid=0D -> TX speed=");
            Serial.print(state.speed_kmh);
            Serial.println(" km/h");
            break;
        }
        case PID_THROTTLE: {
            uint8_t raw = (uint8_t)((uint16_t)state.throttle_pct * 255 / 100);
            uint8_t payload[3] = { 0x41, PID_THROTTLE, raw };
            send_obd_response(3, payload);
            Serial.print("OBD RX mode=01 pid=11 -> TX throttle=");
            Serial.print(state.throttle_pct);
            Serial.println(" %");
            break;
        }
        default:
            Serial.print("OBD RX mode=01 pid=0x");
            Serial.print(pid, HEX);
            Serial.println(" -> unsupported, no response");
            break;
    }
}

static void poll_can_rx(const EcuState &state)
{
    if (CAN.checkReceive() != CAN_MSGAVAIL) {
        return;
    }

    long unsigned int rxId;
    uint8_t len = 0;
    uint8_t buf[8];
    if (CAN.readMsgBuf(&rxId, &len, buf) != CAN_OK) {
        return;
    }

    if (rxId != OBD_REQUEST_ID_FUNCTIONAL && rxId != OBD_REQUEST_ID_PHYSICAL) {
        return;
    }
    if (len < 3) {
        return;
    }

    digitalWrite(LED_BUILTIN, HIGH);
    handle_obd_request(buf[1], buf[2], state);
    digitalWrite(LED_BUILTIN, LOW);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for USB serial on boards that need it
    }
    s_boot_time = millis();
    init_can();
    init_gps();
    Serial.println("Simulated OBD-II ECU ready (requests on 0x7DF/0x7E0, responses on 0x7E8).");
}

void loop()
{
    unsigned long now = millis();
    EcuState state = simulate_ecu_state(now);

    poll_can_rx(state);
    send_periodic_broadcasts(now, state);
    tick_gps(now);

    static unsigned long last_print = 0;
    if (now - last_print >= STATUS_PRINT_INTERVAL_MS) {
        last_print = now;
        Serial.print("Simulated state: rpm=");
        Serial.print(state.rpm);
        Serial.print(" speed=");
        Serial.print(state.speed_kmh);
        Serial.print("km/h throttle=");
        Serial.print(state.throttle_pct);
        Serial.print("% coolant=");
        Serial.print(state.coolant_c);
        Serial.println("C");
    }
}
#endif
