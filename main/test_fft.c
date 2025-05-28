/* FFT Example

   This example runs a few FFTs and measure the timing.

  Author: Robin Scheibler, 2017
   This code is released under MIT license. See the README for more details.§
*/
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#include "soc/timer_group_struct.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"

#include "fft.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "format_wav.h"
#include "u8g2_esp32_hal.h"
#include "freertos/queue.h"

#define PIN_SDA             GPIO_NUM_5
#define PIN_SCL             GPIO_NUM_6
#define CONFIG_EXAMPLE_SAMPLE_RATE 44100
#define CONFIG_EXAMPLE_BIT_SAMPLE 32
#define CONFIG_EXAMPLE_I2S_DATA_GPIO 9
#define CONFIG_EXAMPLE_I2S_CLK_GPIO 11
#define CONFIG_SPI_MOSI_GPIO GPIO_NUM_37
#define CONFIG_SPI_MISO_GPIO GPIO_NUM_35
#define CONFIG_SPI_SCLK_GPIO GPIO_NUM_36
#define CONFIG_SPI_CS_GPIO GPIO_NUM_38
#define CONFIG_REC_TIME 10
#define useSD
//#define CONFIG_EXAMPLE_REC_TIME 1

//Display variable
u8g2_t u8g2;

static const char *TAG = "FFT";
#define BUFF_SIZE 1024
#define SPI_DMA_CHAN        SPI_DMA_CH_AUTO
#define NUM_CHANNELS        (1) // For mono recording only!
#define SD_MOUNT_POINT      "/sdcard"
#define SAMPLE_SIZE         (CONFIG_EXAMPLE_BIT_SAMPLE * 1024)
#define BYTE_RATE           (CONFIG_EXAMPLE_SAMPLE_RATE * (CONFIG_EXAMPLE_BIT_SAMPLE / 8)) * NUM_CHANNELS
#define LED_PIN GPIO_NUM_48
// When testing SD and SPI modes, keep in mind that once the card has been
// initialized in SPI mode, it can not be reinitialized in SD mode without
// toggling power to the card.
sdmmc_host_t host = SDSPI_HOST_DEFAULT();
sdmmc_card_t *card;
i2s_chan_handle_t rx_handle = NULL;
static i2s_chan_handle_t rx_chan; // I2S rx channel handler
static int16_t i2s_readraw_buff[SAMPLE_SIZE];
QueueHandle_t sample_queue, frequency_queue;

int32_t raw_samples[BUFF_SIZE] = {0};
size_t bytes_read;
const int WAVE_HEADER_SIZE = 44;

/* Can run 'make menuconfig' to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define REP 100
#define MIN_LOG_N 6
#define MAX_LOG_N 12

#define GPIO_OUTPUT 48

double start, end;

static void fft_task(void *args)
{
    int32_t fft_samples[BUFF_SIZE] = {0};
    fft_config_t *fft_analysis = (fft_config_t*)args;
    while (1)
    {
        // Receive data from the queue
        if (xQueueReceive(sample_queue, (void *)fft_samples, portMAX_DELAY) == pdTRUE)
        {
            // Fill array with some dummy data
            for (int k = 0 ; k < fft_analysis->size ; k++){
              fft_analysis->input[k] = fft_samples[k];
               ESP_LOGI(TAG, "%d-th smp : %f", k, fft_samples[k]);
            }
            // Execute transformation
            fft_execute(fft_analysis);
            // Now do something with the output
            //ESP_LOGI(TAG,"DC component : %f", fft_analysis->output[0]);  // DC is at [0]
            u8g2_ClearBuffer(&u8g2);
            int scale = fft_analysis->size/2/u8g2.height;
            int real0 = sqrt(pow (fft_analysis->output[2], 2) + pow(fft_analysis->output[2+1], 2));
            for (int k = 1 ; k < fft_analysis->size / 2 ; k+=scale){
              ESP_LOGD(TAG, "%d-th freq : %f+j%f", k, fft_analysis->output[2*k], fft_analysis->output[2*k+1]);
              float real1 = sqrt(pow (fft_analysis->output[2*(k+scale)], 2) + pow(fft_analysis->output[2*(k+scale)+1], 2));
              ESP_LOGI(TAG, "%d-th freq : %f", k, real0);
              
              u8g2_DrawLine(&u8g2, k/scale,     u8g2.height - ((int)real0 + u8g2.height/2), 
                                   k/scale + 1, u8g2.height - ((int)real1 + u8g2.height/2));
              real0 = real1;
            }
            u8g2_SendBuffer(&u8g2);
              //printf("Middle component : %f\n", fft_analysis->output[1]);  // N/2 is real and stored at [1]
        }
        vTaskDelay(pdMS_TO_TICKS(80));

        // printf("executing compute fft \n");
    }

    fft_destroy(fft_analysis);
    vTaskDelete(NULL);
}

void app_fft_init(){
      // Create fft plan and let it allocate arrays

}

void record_wav(uint32_t rec_time)
{

    // Use POSIX and C standard library functions to work with files.
    int flash_wr_size = 0;

    char file_path[100] = SD_MOUNT_POINT"/record.wav";

    printf("********** %s ************\n", file_path);

    ESP_LOGI(TAG, "Opening file");

    uint32_t flash_rec_time = BYTE_RATE * rec_time;
    const wav_header_t wav_header =
        WAV_HEADER_PCM_DEFAULT(flash_rec_time, 32, CONFIG_EXAMPLE_SAMPLE_RATE, 1);

    // First check if file exists before creating a new file.
    struct stat st;
    if (stat(file_path, &st) == 0)
    {
        printf("inside unlink if \n");
        // Delete it if it exists
        unlink(file_path);
    }

    // printf("outside unlink if \n");

    // Create new WAV file
    FILE *f = fopen(file_path, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    // Write the header to the WAV file
    fwrite(&wav_header, sizeof(wav_header), 1, f);

    i2s_channel_enable(rx_chan);
    gpio_set_level(LED_PIN, 1);

    ESP_LOGI(TAG, "Starting recording for %d seconds!", CONFIG_REC_TIME);

    // Start recording
    while (flash_wr_size < flash_rec_time)
    {
        size_t bytes_read = 0;
        // Read the RAW samples from the microphone
        i2s_channel_read(rx_chan, raw_samples, sizeof(int32_t) * BUFF_SIZE, &bytes_read, portMAX_DELAY);
        int samples_read = bytes_read / 4;
        if (samples_read == BUFF_SIZE)
        {
            // Send data to the queue
            if (xQueueSend(sample_queue, (void *)raw_samples, portMAX_DELAY) != pdTRUE)
            {
                printf("Failed to send data to queue\n");
            }
        }
        fwrite(raw_samples, bytes_read, 1, f);
        flash_wr_size += bytes_read;
    }

    gpio_set_level(LED_PIN, 0);
    ESP_LOGI(TAG, "Recording done!");
    fclose(f);
    ESP_LOGI(TAG, "File written on SDCard");

}

static void record_wave_task(void *args)
{
  ESP_LOGI(TAG, "Recording!");
    while (1)
    {

        record_wav(CONFIG_REC_TIME);
#ifdef useSD
        ESP_ERROR_CHECK(i2s_channel_disable(rx_chan));
#endif
         //ESP_ERROR_CHECK(i2s_del_channel(rx_chan));

        vTaskDelay(pdMS_TO_TICKS(5000));
    }
    gpio_set_level(LED_PIN, 0);
    vTaskDelete(NULL);
}

void mount_sdcard(void)
{
    esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 8 * 1024};
    ESP_LOGI(TAG, "Initializing SD card");
    host.max_freq_khz = 1000;
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = CONFIG_SPI_MOSI_GPIO,
        .miso_io_num = CONFIG_SPI_MISO_GPIO,
        .sclk_io_num = CONFIG_SPI_SCLK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_SPI_CS_GPIO;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);
}

static void i2s_init_std_simplex(void)
{
    i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(CONFIG_EXAMPLE_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, // some codecs may require mclk signal, this example doesn't need it
            .bclk = CONFIG_EXAMPLE_I2S_CLK_GPIO,
            .ws = GPIO_NUM_10,
            .dout = I2S_GPIO_UNUSED,
            .din = CONFIG_EXAMPLE_I2S_DATA_GPIO,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    rx_std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_chan, &rx_std_cfg));
}

void app_main()
{
   // initialize the u8g2 hal
	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda = PIN_SDA;
	u8g2_esp32_hal.scl = PIN_SCL;
	u8g2_esp32_hal_init(u8g2_esp32_hal);

	// initialize the u8g2 library
	u8g2_Setup_ssd1306_i2c_128x64_noname_f(
		&u8g2,
		U8G2_R0,
		u8g2_esp32_i2c_byte_cb,
		u8g2_esp32_gpio_and_delay_cb);
	
	// set the display address
	u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);
	
	// initialize the display
	u8g2_InitDisplay(&u8g2);
	
	// wake up the display
	u8g2_SetPowerSave(&u8g2, 0);
  
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    // Mount the SDCard for recording the audio file
    sample_queue = xQueueCreate(10, BUFF_SIZE * sizeof(int32_t));
    frequency_queue = xQueueCreate(2048, sizeof(float));

    if (sample_queue == NULL)
    {
        printf("Failed to create sample queue\n");
        return;
    }

    if (frequency_queue == NULL)
    {
        printf("Failed to create frequency queue\n");
        return;
    }
    printf("PDM microphone recording example start\n--------------------------------------\n");
    #ifdef useSD
    // Mount the SDCard for recording the audio file
    mount_sdcard();
    #endif //useSD
    fft_config_t *fft_analysis = fft_init(BUFF_SIZE, FFT_REAL, FFT_FORWARD, NULL, NULL);

    // Acquire a I2S PDM channel for the PDM digital microphone
    i2s_init_std_simplex();
    i2s_channel_enable(rx_chan);
    gpio_set_level(LED_PIN, 1);
  //clock_init();
    xTaskCreate(record_wave_task, "i2s_example_read_task", 32384, NULL, 5, NULL);
    xTaskCreate(fft_task, "fft_task", 8096, fft_analysis, 5, NULL);

  while (1)
  {
    //fft_test_task();
    //rfft_test_task();
    //fft8_test_task();
    //fft4_test_task();
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}
