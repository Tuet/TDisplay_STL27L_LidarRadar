/*
 * @file STL27L.h
 * @brief Arduino-style driver for the STL-27L 2D LiDAR sensor.
 *
 * The driver receives data through an ESP32 hardware UART and controls
 * the sensor's motor through an ESP32 LEDC PWM output. Data reception is
 * polling-based: the application must call Lidar.update() as often as
 * possible. Each 47-byte packet is CRC-8 validated (polynomial 0x4D) and
 * decoded into STL27LPoint records that are stored in a fixed-size array
 * of STL27L_POINT_COUNT angular bins.
 *
 * Default configuration
 * ---------------------
 *   - UART number: STL27L_UART_NUMBER      (default 2)
 *   - Baud rate:   STL27L_BAUD_RATE        (default 921600)
 *   - PWM:         STL27L_MOTOR_PWM_FREQUENCY (default 1 kHz)
 *                  STL27L_MOTOR_DUTY_PERCENT  (default 62 %)
 *
 * Pin hook-up (STL-27L -> ESP32)
 * ------------------------------
 *   Pin 1 (Tx)  -> any free GPIO that the chosen UART can receive on
 *                  (e.g. GPIO 27, used in src/main.ino)
 *   Pin 2 (PWM) -> any free GPIO that supports LEDC PWM
 *                  (e.g. GPIO 26, used in src/main.ino; optional, pass -1
 *                  to begin() if not wired)
 *   Pin 3 (GND) -> GND
 *   Pin 4 (P5V) -> 5V
 *
 * See README.md for the full pin table and build instructions.
 *
 * License
 * -------
 * This is free and unencumbered software released into the public domain.
 * Anyone is free to copy, modify, publish, use, compile, sell, or
 * distribute this software, either in source code form or as a compiled
 * binary, for any purpose, commercial or non-commercial, and by any
 * means.
 *
 * For more information, see <https://unlicense.org>
 */

#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>

/*
 * Number of angular bins for one complete revolution.
 *
 * This can be changed in platformio.ini:
 *
 *   -D STL27L_POINT_COUNT=720
 */
#ifndef STL27L_POINT_COUNT
#define STL27L_POINT_COUNT 720
#endif

/*
 * STL-27L UART configuration according to the development manual.
 */
#ifndef STL27L_BAUD_RATE
#define STL27L_BAUD_RATE 921600UL
#endif

/*
 * Motor PWM configuration.
 *
 * The manual specifies:
 *   - Frequency: 20 to 50 kHz
 *   - Recommended frequency: 30 kHz
 *   - External-control activation: 45% < duty cycle < 55%
 * 
 * Manual seems wrong about frequency and duty, the following worked for me:
 *   - Frequency: 1 kHz
 *   - Duty cycle: 62% (for a little more then 10Hz speed)
 *   - Alternative: Not connect the PWM PIN at all works well too
 */
#ifndef STL27L_MOTOR_PWM_FREQUENCY
//#define STL27L_MOTOR_PWM_FREQUENCY 30000UL
#define STL27L_MOTOR_PWM_FREQUENCY 1000UL
#endif

#ifndef STL27L_MOTOR_DUTY_PERCENT
#define STL27L_MOTOR_DUTY_PERCENT 62
#endif

/*
 * UART2 is used by default.
 *
 * UART0 is normally used for the USB serial monitor.
 * UART2 is available on the classic ESP32 used by the original T-Display.
 */
#ifndef STL27L_UART_NUMBER
#define STL27L_UART_NUMBER 2
#endif

/*
 * Number of measurement points contained in one STL-27L packet.
 */
static constexpr size_t STL27L_POINTS_PER_PACKET = 12;

/*
 * Complete STL-27L packet length:
 *
 *   Header        1 byte
 *   Version/len   1 byte
 *   Speed         2 bytes
 *   Start angle   2 bytes
 *   Points       36 bytes
 *   End angle     2 bytes
 *   Timestamp     2 bytes
 *   CRC           1 byte
 *                --------
 *                47 bytes
 */
static constexpr size_t STL27L_PACKET_SIZE = 47;

/**
 * One angular point maintained by the STL-27L driver.
 */
struct STL27LPoint
{
    /**
     * Measured distance in millimetres.
     */
    uint16_t distance;

    /**
     * Reflection intensity reported by the sensor.
     */
    uint8_t intensity;

    /**
     * Point lifetime.
     *
     * A newly received point is assigned initialTtl. Once per detected
     * revolution, every non-zero TTL value is decremented by one.
     */
    uint8_t ttl;
};

/**
 * Arduino-style driver for the STL-27L LiDAR.
 *
 * The driver receives data through an ESP32 hardware UART and controls the
 * motor through an ESP32 LEDC PWM output.
 *
 * Data reception is polling-based. The application must call update() as
 * often as possible.
 */
class STL27LClass
{
public:
    STL27LClass();

    /**
     * Start the LiDAR interface.
     *
     * @param dataPin  ESP32 GPIO connected to the LiDAR TX output.
     * @param motorPin ESP32 GPIO connected to the LiDAR PWM input.
     *
     * @return true if UART and PWM setup succeeded.
     */
    bool begin(int dataPin, int motorPin);

    /**
     * Stop UART reception and stop the motor.
     *
     * The motor output is detached from LEDC and driven HIGH.
     */
    void end();

    /**
     * Read and process all currently available UART bytes.
     *
     * Call this function frequently from loop().
     */
    void update();

    /**
     * Check whether a new revolution has been completed.
     *
     * Reading the flag clears it.
     *
     * @return true once for every detected revolution.
     */
    bool available();

    /**
     * Return the number of angular bins.
     */
    constexpr size_t size() const
    {
        return STL27L_POINT_COUNT;
    }

    /**
     * Return a read-only point by index.
     *
     * @param index Angular point index in the range 0 to size() - 1.
     */
    const STL27LPoint &point(size_t index) const;

    /**
     * Return the approximate angle represented by an index.
     *
     * @return Angle in degrees in the range 0 to less than 360.
     */
    float angleForIndex(size_t index) const;

    /**
     * Return the most recently reported angular speed.
     *
     * @return Angular speed in degrees per second.
     */
    uint16_t speed() const
    {
        return _speed;
    }

    /**
     * Return the most recently received LiDAR timestamp.
     */
    uint16_t timestamp() const
    {
        return _timestamp;
    }

    /**
     * Return the number of packets rejected because of CRC errors.
     */
    uint32_t crcErrorCount() const
    {
        return _crcErrorCount;
    }

    /**
     * Return the number of packets rejected because of rx errors.
     */
    uint32_t rxErrorCount() const
    {
        return _rxErrorCount;
    }

    /**
     * Return the number of UART hardware/ring-buffer overflow events
     * reported by the ESP32 driver (UART_FIFO_OVF_ERROR or
     * UART_BUFFER_FULL_ERROR).
     *
     * A non-zero value means that incoming LiDAR bytes were dropped before
     * they could be processed, which is a common cause of CRC errors.
     */
    uint32_t uartOverflowCount() const
    {
        return _uartOverflowCount;
    }

    /**
     * Return the number of valid packets received.
     */
    uint32_t validPacketCount() const
    {
        return _validPacketCount;
    }

    /**
     * Return whether the driver is currently active.
     */
    bool isRunning() const
    {
        return _running;
    }

    /**
     * Set the motor PWM duty cycle.
     *
     * @param percent Duty cycle from 0 to 100 percent.
     *
     * Values outside the sensor's recommended activation range may stop
     * external motor control.
     */
    bool setMotorDuty(uint8_t percent);

    /**
     * Clear all stored measurements and TTL values.
     */
    void clear();

    /**
     * TTL value assigned to a freshly received point.
     *
     * The default of 20 means a point is treated as fresh for roughly
     * 20 detected revolutions before its TTL reaches zero.
     *
     * Changing this value at runtime only affects points that arrive
     * after the change; already-stored points keep the TTL they were
     * originally assigned.
     */
    uint8_t initialTtl = 10;

private:
    static constexpr uint8_t PACKET_HEADER = 0x54;
    static constexpr uint8_t PACKET_VERSION_LENGTH = 0x2C;
    static constexpr uint8_t PWM_RESOLUTION_BITS = 8;

    HardwareSerial _serial;

    STL27LPoint _points[STL27L_POINT_COUNT];

    uint8_t _packetBuffer[STL27L_PACKET_SIZE];
    size_t _packetPosition;

    int _dataPin;
    int _motorPin;

    bool _running;
    bool _newRevolution;
    bool _havePreviousAngle;

    uint16_t _previousAngle;
    uint16_t _speed;
    uint16_t _timestamp;

    uint32_t _crcErrorCount;
    uint32_t _rxErrorCount;
    uint32_t _validPacketCount;

    /**
     * Incremented by the UART error callback whenever the ESP32 driver
     * reports a FIFO or ring-buffer overflow.
     *
     * Marked volatile because it is written from the UART event task and
     * read from the main loop.
     */
    volatile uint32_t _uartOverflowCount;

    /**
     * Process a chunk of newly received UART bytes and maintain packet
     * synchronisation.
     *
     * Processing works in bulk rather than byte-by-byte:
     *   - std::memchr() locates the next packet header.
     *   - std::memcpy() copies the remaining packet body in one call.
     *
     * This drastically reduces the per-byte CPU cost compared to calling a
     * subroutine for every single byte, which matters at 921600 baud.
     */
    void processChunk(const uint8_t *data, size_t length);

    /**
     * Validate and decode one complete packet.
     */
    void processPacket();

    /**
     * Insert one measurement into the nearest angular bin.
     */
    void storePoint(uint16_t angleHundredths,
                    uint16_t distance,
                    uint8_t intensity);

    /**
     * Detect the transition from the end to the start of a revolution.
     */
    void checkRevolution(uint16_t angleHundredths);

    /**
     * Age all currently stored points by one revolution.
     */
    void agePoints();

    /**
     * Calculate the STL-27L CRC-8 using a precomputed 256-entry lookup
     * table.
     *
     * This replaces the bitwise implementation (8 shift/branch operations
     * per byte) with a single table lookup per byte, which meaningfully
     * reduces CPU time spent per received packet at 921600 baud.
     */
    static uint8_t calculateCRC(const uint8_t *data, size_t length);

    /**
     * STL-27L CRC-8 lookup table.
     *
     * Generated for polynomial 0x4D, as specified by the development
     * manual and previously implemented bit-by-bit.
     */
    static const uint8_t CRC_TABLE[256];

    /**
     * Read a little-endian 16-bit integer.
     */
    static uint16_t readUInt16LE(const uint8_t *data);
};

/*
 * Default global LiDAR instance.
 *
 * Including STL27L.h makes the following style possible:
 *
 *   Lidar.begin(dataPin, motorPin);
 */
extern STL27LClass Lidar;