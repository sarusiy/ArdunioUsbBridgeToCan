/*
 * Arduino Uno + MCP2515/TJA1050 CAN module.
 *
 * Phase 1 (current): HW connectivity test only.
 *   - Every second, sends a fixed 8-char test message (TEST_MESSAGE) as a
 *     single CAN frame (8 bytes is the max per frame) on TEST_CAN_ID, and
 *     logs it over USB serial.
 *   - Logs any CAN frame received (expected: the JC-ESP32P4-M3 board
 *     echoing the same frame back on the bus) and reports whether the
 *     echoed message matches what was last sent.
 *
 * Wiring (see Doc/electrical drawing.vsdx in JC-ESP32P4-M3):
 *   MCP2515 SCK  -> Uno D13
 *   MCP2515 SI   -> Uno D11 (MOSI)
 *   MCP2515 SO   -> Uno D12 (MISO)
 *   MCP2515 CS   -> Uno D10
 *   MCP2515 VCC  -> 5V, GND -> GND
 *   CAN-H/CAN-L  -> shared bus with the ESP32-P4's MCP2515
 *   J1 jumper closed on this module for 120 ohm bus termination
 *
 * Phase 2 (future): replace the test payload with an actual USB<->CAN
 * bridge protocol (e.g. SLCAN) so a PC can inject/observe real frames.
 */
#include <SPI.h>
#include <mcp_can.h>
#include <string.h>

#define CAN_CS_PIN 10
#define TEST_CAN_ID 0x100
#define SEND_INTERVAL_MS 1000
#define TEST_MESSAGE_LEN 8

MCP_CAN CAN(CAN_CS_PIN);

static const char s_test_message[TEST_MESSAGE_LEN] = { '0', '1', '2', '3', '4', '5', '6', '7' };

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        ; // wait for USB serial on boards that need it
    }

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

static void send_test_message()
{
    /* Pin 13/LED_BUILTIN doubles as SPI SCK, whose idle-low state instantly
     * overrides digitalWrite() once CAN polling resumes; hold it with a
     * blocking delay so the "on" phase is actually visible. */
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);

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
