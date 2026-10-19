#include "GM65_scanner.h"

GM65_Scanner::GM65_Scanner(Stream* serial) : _serial(serial) {
    _utf16_pending = false;
    _utf16_low_byte = 0;
    
    _tx_buf[0] = HEADER;
    _tx_buf[2] = CMD_WRITE;
    _tx_buf[3] = 0x01;
    _tx_buf[7] = TAIL_HI;
    _tx_buf[8] = TAIL_LO;
}

bool GM65_Scanner::begin() {
    flushInput();
    bool success = true;

    // Set Output Format: UNICODE + Serial (0x04)
    if (!writeRegister(REG_OUTPUT_CFG, 0x04)) {
        success = false;
    }
    delay(100);

    // Set Scan Mode: Induction + LED/Buzzer On (0xD7)
    if (!writeRegister(REG_MODE, 0xD7)) {
        success = false;
    }
    delay(100);

    writeRegister(REG_SETTING_CODE, 0x01);
    flushInput();
    return success;
}

String GM65_Scanner::read() {
    if (!_serial->available()) return "";

    // 1. Read all available raw data with timeout
    String rawData = "";
    unsigned long lastByteTime = millis();
    const uint32_t IDLE_TIMEOUT = 30;

    while (true) {
        while (_serial->available()) {
            rawData += (char)_serial->read();
            lastByteTime = millis();
        }
        if (rawData.length() > 0 && (millis() - lastByteTime > IDLE_TIMEOUT)) break;
        if (rawData.length() == 0 && (millis() - lastByteTime > IDLE_TIMEOUT)) return "";
        delay(2);
    }

    size_t len = rawData.length();
    if (len == 0) return "";

    std::vector<uint8_t> buf;
    buf.reserve(len);
    for (size_t i = 0; i < len; ++i) buf.push_back((uint8_t)rawData[i]);

    String output = "";
    size_t i = 0;

    // 2. Skip BOM (FE FF or FF FE) if present
    if (len >= 2) {
        if ((buf[0] == 0xFE && buf[1] == 0xFF) || (buf[0] == 0xFF && buf[1] == 0xFE)) {
            i = 2; 
        }
    }

    // 3. Decode Hybrid (GM65 Special Mode)
    // Detection rules based on Hex Log:
    // - ASCII: 1 byte (e.g., 4E, 67...)
    // - Unicode: 2 byte Big Endian (e.g., ễ = 1E C5)
    // Key point: The 2nd byte of Unicode is usually >= 0x80 (C5 > 80).
    // While the next byte of ASCII is usually < 0x80.
    
    while (i < len) {
        uint8_t b1 = buf[i];
        
        // Check if this byte pairs with the next byte to form a Unicode character
        bool isPair = false;
        if (i + 1 < len) {
            uint8_t b2 = buf[i+1];
            // Heuristic: If the next byte is greater than or equal to 0x80, 
            // it's likely that b1 and b2 form a Unicode character (Big Endian)
            // e.g., ễ (1E C5). b1=1E, b2=C5 (197 >= 128). -> Pair.
            // e.g., n| (6E 1E). b1=6E, b2=1E (30 < 128). -> No pair.
            if (b2 >= 0x80) {
                isPair = true;
            }
        }

        if (isPair) {
            // This is a 2-byte Unicode character (Big Endian)
            uint16_t cp = (uint16_t)buf[i] << 8 | buf[i+1];
            appendCodepointToUtf8(cp, output);
            i += 2; // Skip 2 bytes
        } else {
            // This is a 1-byte ASCII character
            if (b1 == 0x1E || b1 == 0x1D) {
                output += '|'; // Replace separator character
            } else if (b1 >= 0x20 || b1 == 0x09) {
                // Only take printable characters and tab, ignore control codes
                output += (char)b1;
            }
            i += 1;
        }
    }

    return output;
}

// --- Private Helpers ---

bool GM65_Scanner::writeRegister(uint16_t address, uint8_t value) {
    _tx_buf[4] = (address >> 8) & 0xFF;
    _tx_buf[5] = address & 0xFF;
    _tx_buf[6] = value;
    flushInput();
    _serial->write(_tx_buf, 9);
    return waitForAck();
}

bool GM65_Scanner::waitForAck(uint16_t timeout_ms) {
    unsigned long start = millis();
    while (millis() - start < timeout_ms) {
        if (_serial->available() >= 7) {
            for (int i = 0; i < 7; i++) _rx_buf[i] = _serial->read();
            if (_rx_buf[0] == 0x02 && _rx_buf[5] == 0x33 && _rx_buf[6] == 0x31) return true;
        }
        delay(5);
    }
    return false;
}

void GM65_Scanner::flushInput() {
    while (_serial->available()) _serial->read();
}

void GM65_Scanner::appendCodepointToUtf8(uint16_t cp, String &out) {
    if (cp < 0x80) {
        out += (char)cp;
    } else if (cp < 0x800) {
        out += (char)(0xC0 | (cp >> 6));
        out += (char)(0x80 | (cp & 0x3F));
    } else {
        out += (char)(0xE0 | (cp >> 12));
        out += (char)(0x80 | ((cp >> 6) & 0x3F));
        out += (char)(0x80 | (cp & 0x3F));
    }
}