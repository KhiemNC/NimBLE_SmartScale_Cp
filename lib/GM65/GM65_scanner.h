#ifndef GM65_SCANNER_H
#define GM65_SCANNER_H

#include <Arduino.h>
#include <Stream.h>
#include <vector>

// --- Constants & Enums ---

// Scanner Operation Modes
enum class GM65Mode : uint8_t {
    MANUAL          = 0b00,
    COMMAND_TRIGGER = 0b01,
    CONTINUOUS      = 0b10,
    INDUCTION       = 0b11
};

// Character Encoding Formats
enum class GM65Format : uint8_t {
    GBK     = 0b00,
    UNICODE = 0b01, // Required for Vietnamese
    BIG5    = 0b10
};

class GM65_Scanner {
public:
    /**
     * @brief Constructor
     * @param serial Pointer to the HardwareSerial or SoftwareSerial instance
     */
    GM65_Scanner(Stream* serial);

    /**
     * @brief Initialize the scanner with production settings.
     * Configures: Induction Mode, Unicode Output, Serial Interface.
     * @return true if scanner responded and configured successfully.
     */
    bool begin();

    /**
     * @brief Non-blocking read function.
     * Checks for incoming data, handles the "Fake BOM" / UTF-16 quirk of GM65,
     * and returns a clean UTF-8 string.
     * @return String Decoded text (empty if no data).
     */
    String read();

private:
    Stream* _serial;
    
    // Internal buffers
    uint8_t _tx_buf[9];
    uint8_t _rx_buf[7];

    // UTF-16 State machine for fragmented packets
    bool _utf16_pending;
    uint8_t _utf16_low_byte;

    // --- Low Level Protocol ---
    
    // Command Register Addresses
    static constexpr uint16_t REG_MODE        = 0x0000;
    static constexpr uint16_t REG_SETTING_CODE= 0x0003;
    static constexpr uint16_t REG_OUTPUT_CFG  = 0x000D;

    // Protocol Constants
    static constexpr uint8_t HEADER           = 0x7E;
    static constexpr uint8_t CMD_WRITE        = 0x08;
    static constexpr uint8_t TAIL_HI          = 0xAB;
    static constexpr uint8_t TAIL_LO          = 0xCD;

    /**
     * @brief Send a write command to a register
     */
    bool writeRegister(uint16_t address, uint8_t value);

    /**
     * @brief Wait for a valid ACK response from the scanner
     */
    bool waitForAck(uint16_t timeout_ms = 200);

    /**
     * @brief Flush RX buffer
     */
    void flushInput();

    /**
     * @brief Helper to convert UTF-16 codepoint to UTF-8 and append to string
     */
    void appendCodepointToUtf8(uint16_t cp, String &out);
};

#endif // GM65_SCANNER_H