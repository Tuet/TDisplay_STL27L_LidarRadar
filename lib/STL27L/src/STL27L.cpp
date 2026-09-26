/*
 * @file STL27L.cpp
 * @brief Implementation of the STL27L Arduino-style LiDAR driver.
 *
 * Responsibilities
 * ----------------
 *   - UART RX via the ESP32 HardwareSerial driver and overflow callback.
 *   - Packet synchronisation, bulk chunk processing (std::memchr/memcpy).
 *   - CRC-8 validation using a precomputed 256-entry lookup table
 *     (polynomial 0x4D).
 *   - Angular bin assignment and per-revolution TTL aging of stored
 *     measurement points.
 *   - LEDC motor PWM control (start / stop / duty update).
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

#include "STL27L.h"

#include <cstring>


/*
 * Default global LiDAR object.
 */
STL27LClass Lidar;

STL27LClass::STL27LClass()
    : _serial(STL27L_UART_NUMBER),
      _packetPosition(0),
      _dataPin(-1),
      _motorPin(-1),
      _running(false),
      _newRevolution(false),
      _havePreviousAngle(false),
      _previousAngle(0),
      _speed(0),
      _timestamp(0),
      _crcErrorCount(0),
      _rxErrorCount(0),
      _validPacketCount(0),
      _uartOverflowCount(0)
{
    clear();
}

bool STL27LClass::begin(int dataPin, int motorPin)
{
    /*
     * Stop a previous configuration before assigning new pins.
     */
    if (_running)
    {
        end();
    }

    if (dataPin < 0 || dataPin == motorPin)
    {
        return false;
    }

    _dataPin = dataPin;
    _motorPin = motorPin;

    _packetPosition = 0;
    _newRevolution = false;
    _havePreviousAngle = false;
    _previousAngle = 0;
    _speed = 0;
    _timestamp = 0;
    _crcErrorCount = 0;
    _rxErrorCount = 0;
    _validPacketCount = 0;
    _uartOverflowCount = 0;

    clear();

    /*
     * The STL-27L generates a relatively high data rate at 921600 baud.
     * A larger UART RX buffer gives the application additional time while
     * the display is being redrawn.
     *
     * setRxBufferSize() must be called before begin().
     */
    if (_serial.setRxBufferSize(32768) == 0)
    {
        return false;
    }

    /*
     * Report real UART hardware/ring-buffer overflows. This is the
     * authoritative signal that incoming bytes were actually dropped,
     * as opposed to _rxErrorCount, which only tracks how often update()
     * self-limits its own processing time.
     *
     * The callback runs on the UART event task, so only a simple
     * increment of a volatile counter is performed here.
     */
    _serial.onReceiveError(
        [this](hardwareSerial_error_t error)
        {
            if (error == UART_FIFO_OVF_ERROR ||
                error == UART_BUFFER_FULL_ERROR)
            {
                _uartOverflowCount = _uartOverflowCount + 1U;
            }
        }
    );

    /*
     * The LiDAR interface is receive-only:
     *
     *   RX pin = dataPin
     *   TX pin = -1
     */
    _serial.begin(
        STL27L_BAUD_RATE,
        SERIAL_8N1,
        _dataPin,
        -1
    );

    _running = true;

    /*
     * Configure the ESP32 LEDC peripheral for motor PWM.
     */
    if(_motorPin >= 0) {
      if (!ledcAttach(
              _motorPin,
              STL27L_MOTOR_PWM_FREQUENCY,
              PWM_RESOLUTION_BITS))
      {
          _serial.end();
          return false;
      }

      if (!setMotorDuty(STL27L_MOTOR_DUTY_PERCENT))
      {
          end();
          return false;
      }
   }

    return true;
}

void STL27LClass::end()
{
    if (_running)
    {
        /*
         * Stop processing UART data and release the UART driver.
         */
        _serial.end();

        /*
         * Remove the PWM peripheral from the pin before forcing the output
         * HIGH as requested.
         */
        ledcDetach(_motorPin);

        pinMode(_motorPin, OUTPUT);
        digitalWrite(_motorPin, HIGH);
    }

    _running = false;
    _newRevolution = false;
    _havePreviousAngle = false;
    _packetPosition = 0;
}

void STL27LClass::update()
{
    if (!_running)
    {
        return;
    }

    /*
     * Do not wait for the UART buffer to become empty. The LiDAR sends
     * continuously, so the buffer might never become completely empty.
     *
     * The byte and time limits ensure that update() regularly returns
     * control to the application.
     *
     * Bytes are read in bulk with a single HardwareSerial::read(buffer,
     * size) call per chunk instead of one HardwareSerial::read() call per
     * byte. Each call acquires an internal UART mutex and goes through the
     * IDF ring buffer API, so batching drastically cuts the per-byte
     * overhead at 921600 baud. The chunk is then handed to processChunk(),
     * which resynchronises and copies whole packets with memchr()/memcpy()
     * rather than invoking a per-byte state machine.
     */
    constexpr size_t CHUNK_SIZE = 512;
    constexpr size_t MAX_BYTES_PER_UPDATE = 4096;
    constexpr uint32_t MAX_TIME_US = 15000;

    static uint8_t chunk[CHUNK_SIZE];

    const uint32_t startTime = micros();
    size_t processedBytes = 0;

    for (;;)
    {
        const int availableNow = _serial.available();

        if (availableNow <= 0)
        {
            break;
        }

        size_t toRead = static_cast<size_t>(availableNow);

        if (toRead > CHUNK_SIZE)
        {
            toRead = CHUNK_SIZE;
        }

        const size_t readCount = _serial.read(chunk, toRead);

        if (readCount == 0)
        {
            ++_rxErrorCount;
            break;
        }

        processChunk(chunk, readCount);
        processedBytes += readCount;

        if (processedBytes >= MAX_BYTES_PER_UPDATE)
        {
            ++_rxErrorCount;
            break;
        }

        if ((micros() - startTime) >= MAX_TIME_US)
        {
            break;
        }
    }
}

bool STL27LClass::available()
{
    const bool result = _newRevolution;
    _newRevolution = false;
    return result;
}

const STL27LPoint &STL27LClass::point(size_t index) const
{
    /*
     * Return point zero for an invalid index. This avoids returning an
     * invalid reference while keeping the API convenient for Arduino code.
     */
    if (index >= STL27L_POINT_COUNT)
    {
        index = 0;
    }

    return _points[index];
}

float STL27LClass::angleForIndex(size_t index) const
{
    if (index >= STL27L_POINT_COUNT)
    {
        index %= STL27L_POINT_COUNT;
    }

    return (static_cast<float>(index) * 360.0f) /
           static_cast<float>(STL27L_POINT_COUNT);
}

bool STL27LClass::setMotorDuty(uint8_t percent)
{
    if (!_running || _motorPin < 0)
    {
        return false;
    }

    if (percent > 100)
    {
        percent = 100;
    }

    constexpr uint32_t maximumDuty =
        (1UL << PWM_RESOLUTION_BITS) - 1UL;

    const uint32_t duty =
        (maximumDuty * static_cast<uint32_t>(percent) + 50UL) / 100UL;

    return ledcWrite(_motorPin, duty);
}

void STL27LClass::clear()
{
    std::memset(_points, 0, sizeof(_points));
}

void STL27LClass::processChunk(const uint8_t *data, size_t length)
{
    size_t offset = 0;

    while (offset < length)
    {
        /*
         * Wait for the packet header while the parser is idle.
         *
         * memchr() scans the remaining chunk in one call instead of
         * comparing bytes one at a time through a subroutine call.
         */
        if (_packetPosition == 0)
        {
            const void *headerPtr = std::memchr(
                data + offset,
                PACKET_HEADER,
                length - offset
            );

            if (headerPtr == nullptr)
            {
                break;
            }

            offset = static_cast<size_t>(
                static_cast<const uint8_t *>(headerPtr) - data
            );

            _packetBuffer[0] = PACKET_HEADER;
            _packetPosition = 1;
            ++offset;

            continue;
        }

        /*
         * Copy as many bytes as are available, up to the remainder of the
         * packet, in a single memcpy() rather than one byte per call.
         */
        const size_t needed = STL27L_PACKET_SIZE - _packetPosition;
        const size_t remaining = length - offset;
        const size_t toCopy = (remaining < needed) ? remaining : needed;

        std::memcpy(
            &_packetBuffer[_packetPosition],
            data + offset,
            toCopy
        );

        _packetPosition += toCopy;
        offset += toCopy;

        if (_packetPosition < STL27L_PACKET_SIZE)
        {
            continue;
        }

        /*
         * The version/length byte is checked here, once per complete
         * packet, instead of immediately after every second received
         * byte. If it does not match, look for another header further
         * inside the buffer instead of discarding just one byte at a
         * time.
         */
        if (_packetBuffer[1] != PACKET_VERSION_LENGTH)
        {
            const void *nextHeader = std::memchr(
                &_packetBuffer[1],
                PACKET_HEADER,
                STL27L_PACKET_SIZE - 1
            );

            if (nextHeader != nullptr)
            {
                const size_t index = static_cast<size_t>(
                    static_cast<const uint8_t *>(nextHeader) - _packetBuffer
                );

                const size_t keep = STL27L_PACKET_SIZE - index;

                std::memmove(_packetBuffer, nextHeader, keep);
                _packetPosition = keep;
            }
            else
            {
                _packetPosition = 0;
            }

            continue;
        }

        processPacket();
        _packetPosition = 0;
    }
}

void STL27LClass::processPacket()
{
    /*
     * The final byte is the CRC. It is calculated over the preceding
     * 46 bytes.
     */
    const uint8_t expectedCRC =
        _packetBuffer[STL27L_PACKET_SIZE - 1];

    const uint8_t calculatedCRC =
        calculateCRC(_packetBuffer, STL27L_PACKET_SIZE - 1);

    if (calculatedCRC != expectedCRC)
    {
        ++_crcErrorCount;
        return;
    }

    ++_validPacketCount;

    /*
     * Packet layout:
     *
     *   0       Header
     *   1       Version and length
     *   2..3    Speed
     *   4..5    Start angle
     *   6..41   Twelve measurement points
     *   42..43  End angle
     *   44..45  Timestamp
     *   46      CRC
     */
    _speed = readUInt16LE(&_packetBuffer[2]);

    const uint16_t startAngle =
        readUInt16LE(&_packetBuffer[4]);

    const uint16_t endAngle =
        readUInt16LE(&_packetBuffer[42]);

    _timestamp = readUInt16LE(&_packetBuffer[44]);

    /*
     * Angles are stored in hundredths of a degree.
     *
     * If the packet crosses zero degrees, temporarily add 360 degrees to
     * the end angle so interpolation continues in the correct direction.
     */
    uint32_t adjustedEndAngle = endAngle;

    if (adjustedEndAngle < startAngle)
    {
        adjustedEndAngle += 36000UL;
    }

    /*
     * Fixed-point (Q24.8) angle interpolation.
     *
     * The original implementation performed a floating-point
     * multiply-add for every one of the twelve points in the packet.
     * On the ESP32 (which has hardware single-precision float support but
     * still costs noticeably more than plain 32-bit integer arithmetic per
     * operation, especially inside a tight per-point loop), avoiding
     * floats here measurably reduces the CPU time spent per packet.
     *
     * An 8-bit fractional part (1/256 of a hundredth of a degree) is far
     * more precision than the final rounded-to-integer angle needs, while
     * keeping (adjustedEndAngle - startAngle) << 8 (at most roughly 72000)
     * safely inside the uint32_t range.
     */
    const uint32_t angleStepQ8 =
        ((adjustedEndAngle - startAngle) << 8U) /
        (STL27L_POINTS_PER_PACKET - 1);

    uint32_t interpolatedAngleQ8 =
        (static_cast<uint32_t>(startAngle) << 8U) + (1UL << 7U);

    for (size_t i = 0; i < STL27L_POINTS_PER_PACKET; ++i)
    {
        const size_t pointOffset = 6 + i * 3;

        const uint16_t distance =
            readUInt16LE(&_packetBuffer[pointOffset]);

        const uint8_t intensity =
            _packetBuffer[pointOffset + 2];

        uint32_t interpolatedAngle = interpolatedAngleQ8 >> 8U;
        interpolatedAngle %= 36000UL;

        storePoint(
            static_cast<uint16_t>(interpolatedAngle),
            distance,
            intensity
        );

        interpolatedAngleQ8 += angleStepQ8;
    }
}

void STL27LClass::storePoint(uint16_t angleHundredths,
                            uint16_t distance,
                            uint8_t intensity)
{
    /*
     * Detect and process the revolution boundary before writing the new
     * point. This way, all older data is aged first and the newly received
     * point receives a fresh TTL from initialTtl.
     */
    checkRevolution(angleHundredths);

    /*
     * Convert the angle to the nearest angular array index:
     *
     *   index = round(angle / 360 degrees * point count)
     */
    uint32_t index =
        (
            static_cast<uint32_t>(angleHundredths) *
            static_cast<uint32_t>(STL27L_POINT_COUNT) +
            18000UL
        ) /
        36000UL;

    index %= STL27L_POINT_COUNT;

    STL27LPoint &destination = _points[index];

    destination.distance = distance;
    destination.intensity = intensity;
    destination.ttl = initialTtl;
}

void STL27LClass::checkRevolution(uint16_t angleHundredths)
{
    if (_havePreviousAngle)
    {
        /*
         * Use wide threshold regions to avoid interpreting small angle
         * jitter as a revolution.
         *
         * A transition from above 300 degrees to below 60 degrees is
         * considered a completed revolution.
         */
        const bool crossedZero =
            _previousAngle > 30000 &&
            angleHundredths < 6000;

        if (crossedZero)
        {
            agePoints();
            _newRevolution = true;
        }
    }

    _previousAngle = angleHundredths;
    _havePreviousAngle = true;
}

void STL27LClass::agePoints()
{
    for (size_t i = 0; i < STL27L_POINT_COUNT; ++i)
    {
        if (_points[i].ttl > 0)
        {
            --_points[i].ttl;
        }
    }
}

/*
 * STL-27L CRC-8 lookup table (polynomial 0x4D, initial value 0x00).
 *
 * Precomputed from the previously used bitwise implementation. Using a
 * table trades 256 bytes of flash for replacing 8 shift/branch operations
 * per byte with a single array lookup, which reduces the CPU time spent
 * validating each 47-byte packet at 921600 baud.
 */
const uint8_t STL27LClass::CRC_TABLE[256] = {
    0x00U, 0x4DU, 0x9AU, 0xD7U, 0x79U, 0x34U, 0xE3U, 0xAEU,
    0xF2U, 0xBFU, 0x68U, 0x25U, 0x8BU, 0xC6U, 0x11U, 0x5CU,
    0xA9U, 0xE4U, 0x33U, 0x7EU, 0xD0U, 0x9DU, 0x4AU, 0x07U,
    0x5BU, 0x16U, 0xC1U, 0x8CU, 0x22U, 0x6FU, 0xB8U, 0xF5U,
    0x1FU, 0x52U, 0x85U, 0xC8U, 0x66U, 0x2BU, 0xFCU, 0xB1U,
    0xEDU, 0xA0U, 0x77U, 0x3AU, 0x94U, 0xD9U, 0x0EU, 0x43U,
    0xB6U, 0xFBU, 0x2CU, 0x61U, 0xCFU, 0x82U, 0x55U, 0x18U,
    0x44U, 0x09U, 0xDEU, 0x93U, 0x3DU, 0x70U, 0xA7U, 0xEAU,
    0x3EU, 0x73U, 0xA4U, 0xE9U, 0x47U, 0x0AU, 0xDDU, 0x90U,
    0xCCU, 0x81U, 0x56U, 0x1BU, 0xB5U, 0xF8U, 0x2FU, 0x62U,
    0x97U, 0xDAU, 0x0DU, 0x40U, 0xEEU, 0xA3U, 0x74U, 0x39U,
    0x65U, 0x28U, 0xFFU, 0xB2U, 0x1CU, 0x51U, 0x86U, 0xCBU,
    0x21U, 0x6CU, 0xBBU, 0xF6U, 0x58U, 0x15U, 0xC2U, 0x8FU,
    0xD3U, 0x9EU, 0x49U, 0x04U, 0xAAU, 0xE7U, 0x30U, 0x7DU,
    0x88U, 0xC5U, 0x12U, 0x5FU, 0xF1U, 0xBCU, 0x6BU, 0x26U,
    0x7AU, 0x37U, 0xE0U, 0xADU, 0x03U, 0x4EU, 0x99U, 0xD4U,
    0x7CU, 0x31U, 0xE6U, 0xABU, 0x05U, 0x48U, 0x9FU, 0xD2U,
    0x8EU, 0xC3U, 0x14U, 0x59U, 0xF7U, 0xBAU, 0x6DU, 0x20U,
    0xD5U, 0x98U, 0x4FU, 0x02U, 0xACU, 0xE1U, 0x36U, 0x7BU,
    0x27U, 0x6AU, 0xBDU, 0xF0U, 0x5EU, 0x13U, 0xC4U, 0x89U,
    0x63U, 0x2EU, 0xF9U, 0xB4U, 0x1AU, 0x57U, 0x80U, 0xCDU,
    0x91U, 0xDCU, 0x0BU, 0x46U, 0xE8U, 0xA5U, 0x72U, 0x3FU,
    0xCAU, 0x87U, 0x50U, 0x1DU, 0xB3U, 0xFEU, 0x29U, 0x64U,
    0x38U, 0x75U, 0xA2U, 0xEFU, 0x41U, 0x0CU, 0xDBU, 0x96U,
    0x42U, 0x0FU, 0xD8U, 0x95U, 0x3BU, 0x76U, 0xA1U, 0xECU,
    0xB0U, 0xFDU, 0x2AU, 0x67U, 0xC9U, 0x84U, 0x53U, 0x1EU,
    0xEBU, 0xA6U, 0x71U, 0x3CU, 0x92U, 0xDFU, 0x08U, 0x45U,
    0x19U, 0x54U, 0x83U, 0xCEU, 0x60U, 0x2DU, 0xFAU, 0xB7U,
    0x5DU, 0x10U, 0xC7U, 0x8AU, 0x24U, 0x69U, 0xBEU, 0xF3U,
    0xAFU, 0xE2U, 0x35U, 0x78U, 0xD6U, 0x9BU, 0x4CU, 0x01U,
    0xF4U, 0xB9U, 0x6EU, 0x23U, 0x8DU, 0xC0U, 0x17U, 0x5AU,
    0x06U, 0x4BU, 0x9CU, 0xD1U, 0x7FU, 0x32U, 0xE5U, 0xA8U,
};

uint8_t STL27LClass::calculateCRC(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;

    for (size_t i = 0; i < length; ++i)
    {
        crc = CRC_TABLE[crc ^ data[i]];
    }

    return crc;
}

uint16_t STL27LClass::readUInt16LE(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8U);
}