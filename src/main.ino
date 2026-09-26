/*
 * T-Display ESP32 + STL-27L LiDAR Radar
 * --------------------------------------
 * Live 2D radar visualisation driven by an STL-27L (LDROBOT D800 family)
 * 2D LiDAR sensor and rendered on a TTGO T-Display (ESP32, ST7789 135x240).
 *
 * The sketch polls the sensor over UART2 at 921600 baud, decodes each
 * 47-byte packet into 720 angular bins via the STL27L driver library, and
 * once per revolution draws a top-down radar (grid, range rings, sweep,
 * status line) into an off-screen TFT_eSprite framebuffer that is then
 * blitted to the display in a single SPI transaction.
 *
 * Pin hook-up (STL-27L -> T-Display ESP32)
 * ----------------------------------------
 *   Pin 1 (Tx)  -> GPIO 27  (LIDAR_DATA_PIN, required)
 *   Pin 2 (PWM) -> GPIO 26  (LIDAR_PWM_PIN, optional; pass -1 if unused,
 *                            or simply leave the sensor's PWM wire open)
 *   Pin 3 (GND) -> GND      (common ground mandatory)
 *   Pin 4 (P5V) -> 5V       (sensor power)
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

#include <Arduino.h>
#include <TFT_eSPI.h>
#include <STL27L.h>

#include <cmath>

/*
 * Suggested free GPIO pins for the classic ESP32 T-Display.
 *
 * LiDAR TX  -> ESP32 GPIO 27
 * LiDAR PWM -> ESP32 GPIO 26 (optional, set to -1 if not used)
 *
 * The grounds of the LiDAR and ESP32 must be connected.
 */
static constexpr int LIDAR_DATA_PIN = 27;
static constexpr int LIDAR_PWM_PIN = 26;

/*
 * Radar display geometry.
 *
 * The status area occupies the bottom part of the 240 x 135 display.
 */
static constexpr int16_t SCREEN_WIDTH = 240;
static constexpr int16_t SCREEN_HEIGHT = 135;

static constexpr int16_t RADAR_CENTER_X = SCREEN_WIDTH / 2;
static constexpr int16_t RADAR_CENTER_Y = 62;
static constexpr int16_t RADAR_RADIUS = 58;

static constexpr int16_t STATUS_Y = 123;

/*
 * Automatic range limits.
 *
 * These values prevent a single very short or very long measurement from
 * making the display unusable.
 */
static constexpr float MINIMUM_RANGE_MM = 100.0f;
static constexpr float MAXIMUM_RANGE_MM = 15000.0f;

/*
 * Additional margin around the most distant visible point.
 */
static constexpr float RANGE_MARGIN = 1.10f;

/*
 * Smoothing factors:
 *
 * The scale expands relatively quickly when a more distant object appears.
 * It contracts slowly when distant objects disappear, reducing flicker.
 */
static constexpr float SCALE_GROW_FACTOR = 0.20f;
static constexpr float SCALE_SHRINK_FACTOR = 0.05f;

static constexpr float DEGREES_TO_RADIANS =
    3.14159265358979323846f / 180.0f;

TFT_eSPI tft = TFT_eSPI();

/*
 * Off-screen RAM framebuffer.
 *
 * TFT_eSprite inherits from TFT_eSPI and overrides the low-level drawing
 * primitives (drawPixel, drawLine, drawFastHLine, drawFastVLine, fillRect,
 * fillSprite, drawChar) so that every draw call writes into this RAM buffer
 * instead of going out over SPI to the display. The high-level helpers
 * (fillScreen, drawCircle, drawString, ...) ultimately call one of those
 * overridden primitives, so they transparently target RAM as well.
 *
 * After every frame has been assembled, sprite.pushSprite(0, 0) blits the
 * whole buffer to the display in a single SPI transaction, which removes
 * the per-pixel tearing that would otherwise be visible while the radar
 * is being redrawn.
 */
TFT_eSprite sprite = TFT_eSprite(&tft);

/*
 * Current outer radar radius in millimetres.
 */
float displayedRangeMM = 3000.0f;

/**
 * Limit a floating-point value to a selected range.
 */
float clampFloat(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

/**
 * Determine the largest currently valid LiDAR distance.
 */
float findMaximumValidDistance()
{
    uint16_t maximumDistance = 0;

    for (size_t i = 0; i < Lidar.size(); ++i)
    {
        const STL27LPoint &point = Lidar.point(i);

        if (point.ttl == 0 || point.distance == 0)
        {
            continue;
        }

        if (point.distance > maximumDistance)
        {
            maximumDistance = point.distance;
        }
    }

    return static_cast<float>(maximumDistance);
}

/**
 * Slowly adapt the displayed range to the measured point cloud.
 */
void updateAutomaticScale()
{
    const float maximumDistance = findMaximumValidDistance();

    if (maximumDistance <= 0.0f)
    {
        return;
    }

    const float targetRange = clampFloat(
        maximumDistance * RANGE_MARGIN,
        MINIMUM_RANGE_MM,
        MAXIMUM_RANGE_MM
    );

    const float smoothing =
        targetRange > displayedRangeMM
            ? SCALE_GROW_FACTOR
            : SCALE_SHRINK_FACTOR;

    displayedRangeMM +=
        (targetRange - displayedRangeMM) * smoothing;

    displayedRangeMM = clampFloat(
        displayedRangeMM,
        MINIMUM_RANGE_MM,
        MAXIMUM_RANGE_MM
    );
}

/**
 * Draw the static radar grid.
 *
 * The caller passes either the TFT directly (writes go straight to the
 * display) or a TFT_eSprite (writes are stored in RAM and pushed to the
 * display in one shot). Both classes share the same drawing API because
 * TFT_eSprite inherits from TFT_eSPI and overrides the low-level
 * primitives that every higher-level helper ends up calling.
 */
void drawRadarGrid(TFT_eSPI &canvas)
{
    //canvas.fillScreen(TFT_BLACK);
    canvas.fillRect(0,0,SCREEN_WIDTH,SCREEN_HEIGHT,TFT_BLACK);

    const uint16_t darkGreen = canvas.color565(0, 55, 0);
    const uint16_t mediumGreen = canvas.color565(0, 90, 0);

    /*
     * Range circles.
     */
    canvas.drawCircle(
        RADAR_CENTER_X,
        RADAR_CENTER_Y,
        RADAR_RADIUS,
        mediumGreen
    );

    canvas.drawCircle(
        RADAR_CENTER_X,
        RADAR_CENTER_Y,
        (RADAR_RADIUS * 2) / 3,
        darkGreen
    );

    canvas.drawCircle(
        RADAR_CENTER_X,
        RADAR_CENTER_Y,
        RADAR_RADIUS / 3,
        darkGreen
    );

    /*
     * Main axes.
     */
    canvas.drawFastHLine(
        RADAR_CENTER_X - RADAR_RADIUS,
        RADAR_CENTER_Y,
        RADAR_RADIUS * 2 + 1,
        darkGreen
    );

    canvas.drawFastVLine(
        RADAR_CENTER_X,
        RADAR_CENTER_Y - RADAR_RADIUS,
        RADAR_RADIUS * 2 + 1,
        darkGreen
    );

    /*
     * Diagonal 45-degree axes.
     */
    constexpr float diagonalFactor = 0.70710678f;

    const int16_t diagonal =
        static_cast<int16_t>(
            static_cast<float>(RADAR_RADIUS) *
            diagonalFactor
        );

    canvas.drawLine(
        RADAR_CENTER_X - diagonal,
        RADAR_CENTER_Y - diagonal,
        RADAR_CENTER_X + diagonal,
        RADAR_CENTER_Y + diagonal,
        darkGreen
    );

    canvas.drawLine(
        RADAR_CENTER_X + diagonal,
        RADAR_CENTER_Y - diagonal,
        RADAR_CENTER_X - diagonal,
        RADAR_CENTER_Y + diagonal,
        darkGreen
    );
}

/**
 * Convert intensity and TTL into an RGB565 display color.
 *
 * Color assignment:
 *
 *   Red channel   = LiDAR intensity
 *   Green channel = TTL
 *   Blue channel  = zero
 *
 * Fresh and intense points therefore appear yellow. Older points gradually
 * lose green and move toward red.
 */
uint16_t colorForPoint(const STL27LPoint &point)
{
    const uint8_t red = point.intensity;
    const uint8_t green = point.ttl;
    const uint8_t blue = 255;

    return tft.color565(red, green, blue);
}

/**
 * Draw every valid angular LiDAR bin as one radar pixel.
 */
void drawRadarPoints(TFT_eSPI &canvas)
{
    /*
     * This loop performs up to STL27L_POINT_COUNT individual pixel writes.
     * When targeting the RAM framebuffer (the production case) these
     * writes are pure memory stores and finish almost instantly, so the
     * UART-drain interval below is more than sufficient; it is kept
     * untouched for safety in case the function is ever called against
     * the live display instead of the sprite.
     */
    constexpr size_t UPDATE_INTERVAL_POINTS = 64;

    for (size_t i = 0; i < Lidar.size(); ++i)
    {
        if ((i % UPDATE_INTERVAL_POINTS) == 0)
        {
            Lidar.update();
        }

        const STL27LPoint &point = Lidar.point(i);

        /*
         * Ignore expired and invalid measurements.
         */
        if (point.ttl == 0 || point.distance == 0)
        {
            continue;
        }

        float normalizedDistance =
            static_cast<float>(point.distance) /
            displayedRangeMM;

        /*
         * Keep measurements outside the scale at the outer radar ring.
         */
        normalizedDistance = clampFloat(
            normalizedDistance,
            0.0f,
            1.0f
        );

        const float pixelRadius =
            normalizedDistance *
            static_cast<float>(RADAR_RADIUS);

        const float angleDegrees =
            Lidar.angleForIndex(i);

        const float angleRadians =
            angleDegrees * DEGREES_TO_RADIANS;

        /*
         * The STL-27L coordinate system defines:
         *
         *   0 degrees = sensor front
         *   Increasing angles = clockwise
         *
         * On the display, zero degrees is drawn upward. Therefore:
         *
         *   x = centerX + sin(angle) * radius
         *   y = centerY - cos(angle) * radius
         */
        const int16_t x =
            RADAR_CENTER_X +
            static_cast<int16_t>(
                std::sin(angleRadians) * pixelRadius
            );

        const int16_t y =
            RADAR_CENTER_Y -
            static_cast<int16_t>(
                std::cos(angleRadians) * pixelRadius
            );

        canvas.drawPixel(x, y, colorForPoint(point));
    }
}

/**
 * Draw status information below the radar.
 */
void drawStatus(TFT_eSPI &canvas)
{
    canvas.fillRect(
        0,
        STATUS_Y,
        SCREEN_WIDTH,
        SCREEN_HEIGHT - STATUS_Y,
        TFT_BLACK
    );

    canvas.setTextDatum(TL_DATUM);
    canvas.setTextFont(1);
    canvas.setTextColor(TFT_LIGHTGREY, TFT_BLACK);

    const float rangeMeters = displayedRangeMM / 1000.0f;
    const float rotationFrequency =
        static_cast<float>(Lidar.speed()) / 360.0f;

    char status[64];

    snprintf(
        status,
        sizeof(status),
        "Range %.1fm  Speed %.1fHz  CRC %lu  OVF %lu",
        rangeMeters,
        rotationFrequency,
        static_cast<unsigned long>(Lidar.crcErrorCount()),
        static_cast<unsigned long>(Lidar.uartOverflowCount())
    );

    canvas.drawString(status, 2, STATUS_Y + 1);
}

/**
 * Draw a complete radar frame.
 *
 * Every drawing step runs against the off-screen RAM framebuffer
 * (`sprite`) instead of the display directly, so the user only ever sees
 * a fully assembled frame: the sprite is blitted to the display in one
 * `pushSprite(0, 0)` call at the end. This removes the per-pixel
 * tearing that the previous direct-to-display implementation produced
 * while the grid, the 720 radar points, and the status text were being
 * pushed over SPI one at a time.
 *
 * The drawing operations themselves are now pure memory writes, which
 * is much faster than going out over SPI, so the LiDAR UART has even
 * more headroom than before. Lidar.update() is still called once per
 * 64 points inside drawRadarPoints() to keep the parser in sync.
 */
void drawRadar()
{
    updateAutomaticScale();

    drawRadarGrid(sprite);
    drawRadarPoints(sprite);
    drawStatus(sprite);

    /*
     * Push the assembled frame to the display in a single SPI burst.
     */
    sprite.pushSprite(0, 0);
}

void setup()
{
    Serial.begin(921600);
    delay(500);

    Serial.println();
    Serial.println("Starting STL-27L radar display.");

    /*
     * Switch on the T-Display backlight.
     */
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);

    tft.init();
    tft.setRotation(1);
    tft.fillScreen(TFT_BLACK);

    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextFont(2);

    sprite.setColorDepth(16);            // 16 Bit Farbtiefe (RGB565)
    sprite.createSprite(tft.width(), tft.height());

    tft.drawString(
        "Starting LiDAR...",
        tft.width() / 2,
        tft.height() / 2
    );

    if (!Lidar.begin(LIDAR_DATA_PIN, LIDAR_PWM_PIN))
    {
        Serial.println("ERROR: LiDAR initialization failed.");

        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_RED, TFT_BLACK);

        tft.drawString(
            "LiDAR init failed",
            tft.width() / 2,
            tft.height() / 2
        );

        return;
    }

    Serial.printf(
        "LiDAR started on RX GPIO %d and PWM GPIO %d.\n",
        LIDAR_DATA_PIN,
        LIDAR_PWM_PIN
    );

    Serial.printf(
        "Angular point count: %u\n",
        static_cast<unsigned int>(Lidar.size())
    );

    Serial.printf(
        "UART baud rate: %lu baud\n",
        static_cast<unsigned long>(STL27L_BAUD_RATE)
    );

}

void loop()
{
    /*
     * update() must be called continuously because the LiDAR transmits at
     * 921600 baud.
     */
    Lidar.update();

    /*
     * available() returns true once per revolution and clears the flag.
     */
    if (Lidar.available())
    {
        drawRadar();

        /*
         * Serial.printf() can block for a long time if the USB/UART0 TX
         * buffer is full, e.g. when no serial monitor is attached to drain
         * it. That would stall loop() and therefore Lidar.update(),
         * risking exactly the kind of UART overflow this message reports
         * on. Only print when the TX buffer has enough room to accept the
         * whole line without blocking.
         */
        if (Serial.availableForWrite() >= 128)
        {
            Serial.printf(
                "Revolution complete: speed=%.2f Hz, packets=%lu, CRC errors=%lu, RX errors=%lu, UART overflows=%lu\n",
                static_cast<float>(Lidar.speed()) / 360.0f,
                static_cast<unsigned long>(Lidar.validPacketCount()),
                static_cast<unsigned long>(Lidar.crcErrorCount()),
                static_cast<unsigned long>(Lidar.rxErrorCount()),
                static_cast<unsigned long>(Lidar.uartOverflowCount())
            );
        }
    }
}