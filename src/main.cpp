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
#include <mcp_can.h>
#include <string.h>
#include <math.h>

#define CAN_CS_PIN 10

MCP_CAN CAN(CAN_CS_PIN);

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
 * The JC-ESP32P4-M3 side currently just echoes frames (it isn't a scan
 * tool), so it won't query this on its own.
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
    data[0] = len;
    memcpy(&data[1], payload, len);
    CAN.sendMsgBuf(OBD_RESPONSE_ID, 0, 8, data);
}

static void handle_obd_request(uint8_t mode, uint8_t pid, const EcuState &state)
{
    if (mode != OBD_MODE_CURRENT_DATA) {
        return;
    }

    switch (pid) {
        case PID_SUPPORTED_01_20: {
            /* Bitmask for PIDs 0x01-0x20: we support 0x05, 0x0C, 0x0D, 0x11. */
            uint8_t payload[5] = { 0x41, PID_SUPPORTED_01_20, 0x08, 0x18, 0x80 };
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
    Serial.println("Simulated OBD-II ECU ready (requests on 0x7DF/0x7E0, responses on 0x7E8).");
}

void loop()
{
    unsigned long now = millis();
    EcuState state = simulate_ecu_state(now);

    poll_can_rx(state);

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
