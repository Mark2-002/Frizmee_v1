#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#define LED_COUNT 8

#define SPI_FREQ 4000000  // 4MHz
#define WS2812_0_CODE 0x8 // Binary: 1000 (T0H=0.25us, T0L=0.75us at 4MHz)
#define WS2812_1_CODE 0xE // Binary: 1100 (T1H=0.75us, T1L=0.25us at 4MHz)

struct rgb_color
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

static struct rgb_color leds[LED_COUNT];
static uint8_t spi_buffer[LED_COUNT * 24 + 50]; // Extra bytes for reset

// SPI device and configuration
const struct device *spi_dev;
struct spi_config spi_cfg = {
    .frequency = SPI_FREQ,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,
    .slave = 0,
    .cs = NULL,
};

void set_pixel_color(int pixel, uint8_t red, uint8_t green, uint8_t blue)
{
    if (pixel < LED_COUNT && pixel >= 0)
    {
        leds[pixel].red = red;
        leds[pixel].green = green;
        leds[pixel].blue = blue;
    }
}

void clear_all_leds(void)
{
    for (int i = 0; i < LED_COUNT; i++)
    {
        leds[i].red = 0;
        leds[i].green = 0;
        leds[i].blue = 0;
    }
}

void send_to_strip(void)
{
    int buf_idx = 0;

    // Add reset pulse at beginning (low for >50us)//20
    for (int i = 0; i < 50; i++)
    {
        spi_buffer[buf_idx++] = 0x00;
    }

    // Convert LED data to SPI format`
    for (int led = 0; led < LED_COUNT; led++)
    {
        // WS2812B expects GRB order, not RGB
        uint8_t colors[3] = {leds[led].green, leds[led].red, leds[led].blue};

        for (int color_byte = 0; color_byte < 3; color_byte++)
        {
            uint8_t byte = colors[color_byte];

            // Convert each bit to SPI timing
            for (int bit = 7; bit >= 0; bit--)
            {
                if (byte & (1 << bit))
                {
                    spi_buffer[buf_idx++] = WS2812_1_CODE; // High bit
                }
                else
                {
                    spi_buffer[buf_idx++] = WS2812_0_CODE; // Low bit
                }
            }
        }
    }

    // Add reset pulse at end
    for (int i = 0; i < 30; i++)
    {
        spi_buffer[buf_idx++] = 0x00;
    }

    // Send via SPI
    struct spi_buf tx_buf = {
        .buf = spi_buffer,
        .len = buf_idx};
    struct spi_buf_set tx_bufs = {
        .buffers = &tx_buf,
        .count = 1};

    int ret = spi_write(spi_dev, &spi_cfg, &tx_bufs);
    if (ret < 0)
    {
        printf("SPI write failed: %d", ret);
    }

    // Wait for reset (>300us for WS2812B)
    k_usleep(500);
}

int8_t ws2812_init(void)
{
    // // Initialize GPIO for boost converter
    // if (!device_is_ready(led.port)) {
    //     return -1;
    // }
    // gpio_pin_configure(led.port, led.pin, GPIO_OUTPUT_ACTIVE | led.dt_flags);
    // gpio_pin_set(led.port, led.pin, 1); // Turn on LED to indicate startup

    // Initialize SPI device
    spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi00));
    if (!device_is_ready(spi_dev))
    {
        printk("SPI device not ready!");
        return -1;
    }
    else
    {
        printk("spiDev is ready");
    }

    clear_all_leds();
    send_to_strip();
    k_sleep(K_MSEC(100));
    return 0;
}

void ws2812_deinit(void)
{
    clear_all_leds();
    send_to_strip();
    // gpio_pin_set(led.port, led.pin, 0); // Turn off boost converter
}

int8_t ws2812_set_color(int pixel, uint8_t red, uint8_t green, uint8_t blue)
{
    if (pixel < 0 || pixel >= LED_COUNT)
    {
        return -1;
    }
    set_pixel_color(pixel, red, green, blue);
    send_to_strip();
    return 0;
}

int8_t ws2812_set_colors(uint8_t red, uint8_t green, uint8_t blue)
{
    for (int i = 0; i < LED_COUNT; i++)
    {
        set_pixel_color(i, red, green, blue);
    }
    send_to_strip();
    return 0;
}