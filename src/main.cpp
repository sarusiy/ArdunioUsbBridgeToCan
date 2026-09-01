/*
 * Arduino Uno + MCP2515/TJA1050 CAN module.
 *
 * Phase 1 (current): HW connectivity test only.
 *   - Sends a CAN frame every second on TEST_CAN_ID, data byte cycling
 *     through the ASCII digits '0'-'9', and logs it over USB serial.
 *   - Logs any CAN frame received (expected: the JC-ESP32P4-M3 board
 *     echoing the same frame back on the bus) and reports whether the
 *     echoed byte matches the last one sent.
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

#define CAN_CS_PIN 10
#define TEST_CAN_ID 0x100
#define SEND_INTERVAL_MS 1000

MCP_CAN CAN(CAN_CS_PIN);

static uint8_t s_last_sent = '0';

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

static void send_test_frame()
{
    /* Pin 13/LED_BUILTIN doubles as SPI SCK, whose idle-low state instantly
     * overrides digitalWrite() once CAN polling resumes; hold it with a
     * blocking delay so the "on" phase is actually visible. */
    digitalWrite(LED_BUILTIN, HIGH);
    delay(150);
    digitalWrite(LED_BUILTIN, LOW);

    uint8_t data[1] = { s_last_sent };
    byte result = CAN.sendMsgBuf(TEST_CAN_ID, 0, 1, data);
    Serial.print("TX id=0x100 data='");
    Serial.print((char)s_last_sent);
    Serial.print("' -> ");
    Serial.println(result == CAN_OK ? "OK" : "ERR");

    s_last_sent = (s_last_sent == '9') ? '0' : s_last_sent + 1;
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

    if (rxId == TEST_CAN_ID && len >= 1) {
        uint8_t expected = (s_last_sent == '0') ? '9' : s_last_sent - 1;
        Serial.println(buf[0] == expected ? "(echo matches last TX)" : "(echo MISMATCH)");
    } else {
        Serial.println();
    }
}

void loop()
{
    static unsigned long last_send = 0;
    unsigned long now = millis();

    if (now - last_send >= SEND_INTERVAL_MS) {
        last_send = now;
        send_test_frame();
    }

    poll_can_rx();
}
