//DAC and BT stack includes
#include <stdio.h>
#include "DAC.c"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include <string.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "freertos/semphr.h"
#include "driver/gptimer.h"
#include "NimbleBT.c"
#include <stdatomic.h>
#include "I2C.c"
#include "driver/adc.h"
#include "esp_adc_cal.h"
//gatt server includes
#include <assert.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "bleprph.h"
#include "services/ans/ble_svc_ans.h"


spi_host_device_t host_id = SPI2_HOST;  //host channel for SPI bus

//handles to identify red green and blue SPI objects
spi_device_handle_t handleRed;
spi_device_handle_t handleGreen;
spi_device_handle_t handleBlue;

//GPIO pin that drives the On/Off signal
#define CONFIG_GPIO_OUTPUT_48 48
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<CONFIG_GPIO_OUTPUT_48)
#define PROPORTIONAL_CONSTANT 0
#define INTEGRAL_CONSTANT 20.04 
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

/*** Maximum number of characteristics with the notify flag ***/
#define MAX_NOTIFY 6

uint16_t redPacket;
uint16_t greenPacket;
uint16_t bluePacket;
double currentReferenceRed;
double currentReferenceGreen;
double currentReferenceBlue;
double currentErrorRed = 0;
double currentErrorGreen = 0;
double currentErrorBlue = 0;
double proportionalOutputRed = 0;
double proportionalOutputGreen = 0;
double proportionalOutputBlue = 0;
double integralOutputRed = 0;
double integralOutputGreen = 0;
double integralOutputBlue = 0;
double actualCurrentRed;
double actualCurrentGreen;
double actualCurrentBlue;
uint16_t outRed;
uint16_t outGreen;
uint16_t outBlue;

int
gatt_svr_write(struct os_mbuf *, uint16_t , uint16_t ,
               void *, uint16_t *);

int
gatt_svc_access(uint16_t , uint16_t ,
                struct ble_gatt_access_ctxt *, void *);

void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *, void *);

int
gatt_svr_init(void);

void reverse(uint8_t*);

uint16_t feedbackValue = 945;



//declares the service UUID to be used for the grow light system with UUID: 59462f12-9543-9999-12c8-58b459a2712d
const ble_uuid128_t growlightServiceUUID =
BLE_UUID128_INIT(0x2d, 0x71, 0xa2, 0x59, 0xb4, 0x58, 0xc8, 0x12,
                    0x99, 0x99, 0x43, 0x95, 0x12, 0x2f, 0x46, 0x59);

/* Red Color characteristic that can be subscribed to with UUID: a5ca1f9c-92cb-4f8f-ab7d-3368562cadf4 */
uint8_t redColorValue[2];
uint16_t redColorValueHandle;
const ble_uuid128_t redColorUUID =
BLE_UUID128_INIT(0xf4, 0xad, 0x2c, 0x56, 0x68, 0x33, 0x7d, 0xab,
                    0x8f, 0x4f, 0xcb, 0x92, 0x9c, 0x1f, 0xca, 0xa5);

/* Green Color characteristic that can be subscribed to with UUID: 3cc449a6-50a9-40cb-9eea-901d31ea8ae1*/
uint8_t greenColorValue[2];
uint16_t greenColorValueHandle;
const ble_uuid128_t greenColorUUID =
BLE_UUID128_INIT(0xe1, 0x8a, 0xea, 0x31, 0x1d, 0x90, 0xea, 0x9e,
                    0xcb, 0x40, 0xa9, 0x50, 0xa6, 0x49, 0xc4, 0x3c);

/* Blue Color characteristic that can be subscribed to with UUID: e669b089-cc6e-4fda-8893-316925591df6*/
uint8_t blueColorValue[2];
uint16_t blueColorValueHandle;
const ble_uuid128_t blueColorUUID =
BLE_UUID128_INIT(0xf6, 0x1d, 0x59, 0x25, 0x69, 0x31, 0x93, 0x88,
                    0xda, 0x4f, 0x6e, 0xcc, 0x89, 0xb0, 0x69, 0xe6);

/* on/Off characteristic that can be subscribed to with UUID: 51be37e8-53c8-40dd-89f5-787170826c07*/
uint8_t onOffValue;
uint16_t onOffValueHandle;
static const ble_uuid128_t onOffValueUUID =
BLE_UUID128_INIT(0x07, 0x6c, 0x82, 0x70, 0x71, 0x78, 0xf5, 0x89,
                    0xdd, 0x40, 0xc8, 0x53, 0xe8, 0x37, 0xbe, 0x51);

/* timer Info characteristic that can be subscribed to with UUID: 2bcc2532-4628-4746-920e-867d76a10845*/
uint8_t timerInfoValue[2];
uint16_t timerInfoValueHandle;
const ble_uuid128_t timerInfoUUID =
BLE_UUID128_INIT(0x45, 0x08, 0xa1, 0x76, 0x7d, 0x86, 0x0e, 0x92,
                    0x46, 0x47, 0x28, 0x46, 0x32, 0x25, 0xcc, 0x2b);

/* lumens Display characteristic that can be subscribed to with UUID: 5432ae84-6a51-4b93-84f0-e69056d654ef */
uint8_t lumensDisplayValue[2];
uint16_t lumensDisplayValueHandle;
const ble_uuid128_t lumensDisplayUUID =
BLE_UUID128_INIT(0xef, 0x54, 0xd6, 0x56, 0x90, 0xe6, 0xf0, 0x84,
                    0x93, 0x4b, 0x51, 0x6a, 0x84, 0xae, 0x32, 0x54);

            
// A custom descriptor
uint8_t gatt_svr_dsc_val;
const ble_uuid128_t gatt_svr_dsc_uuid =
BLE_UUID128_INIT(0x01, 0x01, 0x01, 0x01, 0x12, 0x12, 0x12, 0x12,
                    0x23, 0x23, 0x23, 0x23, 0x34, 0x34, 0x34, 0x34);


const struct ble_gatt_svc_def gatt_svr_svcs[] = 
{ 
    {
        /*** Service ***/
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &growlightServiceUUID.u,
        .characteristics = (struct ble_gatt_chr_def[])
        { 
            {
                /*** these characteristics can be subscribed to by writing 0x00 and 0x01 to the CCCD ***/
                .uuid = &redColorUUID.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &redColorValueHandle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { 
                    {
                    .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                    .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else   
                    .att_flags = BLE_ATT_F_READ,
#endif
                    .access_cb = gatt_svc_access,
                    }, 
                    {
                    0, /* No more descriptors in this characteristic */
                    }
                },
            }, 
            
            {
                .uuid = &greenColorUUID.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &greenColorValueHandle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { 
                    {
                    .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                    .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else
                    .att_flags = BLE_ATT_F_READ,
#endif
                    .access_cb = gatt_svc_access,
                    }, 
                    {
                    0, /* No more descriptors in this characteristic */
                    }
                },
            },

            {
                .uuid = &blueColorUUID.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &blueColorValueHandle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { 
                    {
                    .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                    .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else
                    .att_flags = BLE_ATT_F_READ,
#endif
                    .access_cb = gatt_svc_access,
                    }, 
                    {
                    0, /* No more descriptors in this characteristic */
                    }
                },
            },

            {
                .uuid = &onOffValueUUID.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &onOffValueHandle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { 
                    {
                    .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                    .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else
                    .att_flags = BLE_ATT_F_READ,
#endif
                    .access_cb = gatt_svc_access,
                    }, 
                    {
                    0, /* No more descriptors in this characteristic */
                    }

                },
            },

            {
                .uuid = &timerInfoUUID.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &timerInfoValueHandle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { 
                    {
                    .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                    .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else
                    .att_flags = BLE_ATT_F_READ,
#endif
                    .access_cb = gatt_svc_access,
                    }, 
                    {
                    0, /* No more descriptors in this characteristic */
                    }
                },
            },

            {
                .uuid = &lumensDisplayUUID.u,
                .access_cb = gatt_svc_access,
#if CONFIG_EXAMPLE_ENCRYPTION
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                BLE_GATT_CHR_F_READ_ENC | BLE_GATT_CHR_F_WRITE_ENC |
                BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#else
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY | BLE_GATT_CHR_F_INDICATE,
#endif
                .val_handle = &lumensDisplayValueHandle,
                .descriptors = (struct ble_gatt_dsc_def[])
                { 
                    {
                    .uuid = &gatt_svr_dsc_uuid.u,
#if CONFIG_EXAMPLE_ENCRYPTION
                    .att_flags = BLE_ATT_F_READ | BLE_ATT_F_READ_ENC,
#else
                    .att_flags = BLE_ATT_F_READ,
#endif
                    .access_cb = gatt_svc_access,
                    }, 
                    {
                    0, /* No more descriptors in this characteristic */
                    }
                },    
            },

            {
                0, //no more characteristics in this service
            },
        }
    
    },

    {
            0, /* No more services. */
    },
};


static void check_efuse(void)
{
    //Check if TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }
    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}


static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}


void controlTask(void * pvParameters)
{

    static esp_adc_cal_characteristics_t *adc_chars;
    static const adc_channel_t channelRed = ADC_CHANNEL_7; 
    static const adc_channel_t channelGreen = ADC_CHANNEL_6;
    static const adc_channel_t channelBlue = ADC_CHANNEL_5;  
    static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
    static const adc_atten_t atten = ADC_ATTEN_DB_11;
    static const adc_unit_t unit = ADC_UNIT_2;

    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_2) {
        //adc2_config_width(width);
        adc2_config_channel_atten(channelRed, atten);
        adc2_config_channel_atten(channelGreen, atten);
        adc2_config_channel_atten(channelBlue, atten);
    } else {
        adc1_config_channel_atten((adc1_channel_t)channelRed, atten);
        adc1_config_channel_atten((adc1_channel_t)channelGreen, atten);
        adc1_config_channel_atten((adc1_channel_t)channelBlue, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    uint16_t adcRed;
    uint16_t adcGreen;
    uint16_t adcBlue;
    for(;;)    //infinite loop for code
    { 
    
        integralOutputRed = 0;
        integralOutputGreen = 0;
        integralOutputBlue = 0;
        //Multisampling
        adcRed = 0;
        adcGreen = 0;
        adcBlue = 0;
        
        for(int i = 0; i < NO_OF_SAMPLES; i++)
        {    
            if (unit == ADC_UNIT_2) {
                int rawRed = 0;
                int rawGreen = 0;
                int rawBlue = 0;
                adcRed += adc2_get_raw((adc2_channel_t)channelRed, width, &rawRed);
                adcRed += rawRed;

                adcGreen += adc2_get_raw((adc2_channel_t)channelGreen, width, &rawGreen);
                adcGreen += rawGreen;

                adcBlue += adc2_get_raw((adc2_channel_t)channelBlue, width, &rawBlue);
                adcBlue += rawBlue;
            } else {
                adc1_get_raw((adc1_channel_t)channelRed);
                adc1_get_raw((adc1_channel_t)channelGreen);
                adc1_get_raw((adc1_channel_t)channelGreen);
            }
        }
        adcRed /= NO_OF_SAMPLES;
        adcGreen /= NO_OF_SAMPLES;
        adcBlue /= NO_OF_SAMPLES;

        currentReferenceRed = (-0.0000732 * ((((redColorValue[0]) << 8 | redColorValue[1]) * -1) + 4095)) + 0.3;
        currentReferenceGreen = (-0.0000732 * ((((greenColorValue[0]) << 8 | greenColorValue[1]) * -1) + 4095)) + 0.3;
        currentReferenceBlue = (-0.0000732 * ((((blueColorValue[0]) << 8 | blueColorValue[1]) * -1) + 4095)) + 0.3;
        
        actualCurrentRed = (3.1 / 40950) * adcRed;
        actualCurrentGreen = (3.1 / 40950) * adcGreen;
        actualCurrentBlue = (3.1 / 40950) * adcBlue;

        currentErrorRed = currentReferenceRed - actualCurrentRed;
        currentErrorGreen = currentReferenceGreen - actualCurrentGreen;
        currentErrorBlue = currentReferenceBlue - actualCurrentBlue;

        proportionalOutputRed = PROPORTIONAL_CONSTANT * currentErrorRed;
        integralOutputRed += currentErrorRed / INTEGRAL_CONSTANT;

        proportionalOutputGreen = PROPORTIONAL_CONSTANT * currentErrorGreen;
        integralOutputGreen += currentErrorGreen / INTEGRAL_CONSTANT;

        proportionalOutputBlue = PROPORTIONAL_CONSTANT * currentErrorBlue;
        integralOutputBlue += currentErrorBlue / INTEGRAL_CONSTANT;

        outRed = ((proportionalOutputRed + integralOutputRed) - 0.3) / -0.00000732;
        if(outRed > 4095)
        {
            outRed = 4095;
        }
        if(outRed < 0)
        {
            outRed = 0;
        }

        outGreen = ((proportionalOutputGreen + integralOutputGreen) - 0.3) / -0.00000732;
        if(outGreen > 4095)
        {
            outGreen = 4095;
        }
        if(outGreen < 0)
        {
            outGreen = 0;
        }

        outBlue = ((proportionalOutputBlue + integralOutputBlue) - 0.3) / -0.00000732;
        if(outBlue > 4095)
        {
            outBlue = 4095;
        }
        if(outBlue < 0)
        {
            outBlue = 0;
        }
        //generate packets to send to the DAC including the 12-bit color value and 4 bit command. Flips the value as max digital value correlates with min current and vice versa
        redPacket = (outRed) | 0x3000;
        greenPacket = (outGreen) | 0x3000;
        bluePacket = (outBlue) | 0x3000; 
        //redPacket = (((redColorValue[0] << 8 | 0x3000) | redColorValue[1]) * -1) + 4095;
        //greenPacket = (((greenColorValue[0] << 8 | 0x3000) | greenColorValue[1]) * -1) + 4095;
        //bluePacket = (((blueColorValue[0] << 8 | 0x3000) | blueColorValue[1]) * -1) + 4095;
        //swaps the order of the red data to fit with ESP32 SPI data formatting which needs the MSB to be first so it can be shifted in
        uint16_t toLoadRed = SPI_SWAP_DATA_TX(redPacket, 16);
        //swaps the order of the green data to fit with ESP32 SPI data formatting which needs the MSB to be first so it can be shifted in
        uint16_t toLoadGreen = SPI_SWAP_DATA_TX(greenPacket, 16);
        //swaps the order of the blue data to fit with ESP32 SPI data formatting which needs the MSB to be first so it can be shifted in
        uint16_t toLoadBlue = SPI_SWAP_DATA_TX(bluePacket, 16);
        
        //begin SPI transactions and transmission
        spi_transaction_t transRed = createTransaction(&toLoadRed); //creates a SPI transaction for red and makes sure theres no errors
        esp_err_t ret = transmitToDAC(&transRed, handleRed);    //transmits value to the red DAC
        ESP_ERROR_CHECK(ret);

        
        
        spi_transaction_t transGreen = createTransaction(&toLoadGreen); //creates a SPI transaction for green and makes sure theres no errors
        esp_err_t ret2 = transmitToDAC(&transGreen, handleGreen);   //transmits value to the green DAC
        ESP_ERROR_CHECK(ret2);

        
        
        spi_transaction_t transBlue = createTransaction(&toLoadBlue);   //creates a SPI transaction for blue and makes sure theres no errors
        esp_err_t ret3 = transmitToDAC(&transBlue, handleBlue);     //transmits value to the blue DAC
        ESP_ERROR_CHECK(ret3);  
    }   
}

void colorTask(void * pvParameters)
{
    int rc;

    /* Initialize NVS — it is used to store PHY calibration data */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(tag, "Failed to init nimble %d ", ret);
        return;
    }
    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = bleprph_on_reset;
    ble_hs_cfg.sync_cb = bleprph_on_sync;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sm_io_cap = CONFIG_EXAMPLE_IO_TYPE;
#ifdef CONFIG_EXAMPLE_BONDING
    ble_hs_cfg.sm_bonding = 1;
    /* Enable the appropriate bit masks to make sure the keys
     * that are needed are exchanged
     */
    ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC;
    ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ENC;
#endif
#ifdef CONFIG_EXAMPLE_MITM
    ble_hs_cfg.sm_mitm = 1;
#endif
#ifdef CONFIG_EXAMPLE_USE_SC
    ble_hs_cfg.sm_sc = 1;
#else
    ble_hs_cfg.sm_sc = 0;
#endif
#ifdef CONFIG_EXAMPLE_RESOLVE_PEER_ADDR
    /* Stores the IRK */
    ble_hs_cfg.sm_our_key_dist |= BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist |= BLE_SM_PAIR_KEY_DIST_ID;
#endif

    rc = gatt_svr_init();
    assert(rc == 0);

    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set("Grow-Light-Panel");
    assert(rc == 0);

    /* XXX Need to have template for store */
    ble_store_config_init();


    nimble_port_freertos_init(bleprph_host_task);
    rc = scli_init();
        if (rc != ESP_OK) {
        ESP_LOGE(tag, "scli_init() failed");
    }
    

    i2c_master_dev_handle_t dev_handle;
    i2c_master_bus_handle_t i2c_bus_handle; 

    for(;;)
    {    
        

        
    }
}


void LEDStateTask(void * pvParameters)
{
    //before the for loop of the task, the initialization of the GPIO is done setting the pin to 48 aand th direction to output
    gpio_config_t outputConfig = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL,
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&outputConfig); //sets the configuration data for gpio 0 
    uint16_t timerPacket;
    for(;;)
    {
        
        timerPacket = (timerInfoValue[0] << 8) | timerInfoValue[1]; //generates a timer packet from the 8 bit array
        if(timerPacket != 0)
        {
            //we only execute this logic when the timer info has a value other than zero indicating that the user has set a timer
            gpio_set_level(CONFIG_GPIO_OUTPUT_48, 1);   //turns the LEDs on
            vTaskDelay((1000 * timerPacket) / portTICK_PERIOD_MS);  //delays by the number of seconds given in the timer info set by the user
            gpio_set_level(CONFIG_GPIO_OUTPUT_48, 0);   //turns off the LEDs after the timer is done
            timerInfoValue[0] = 0;
            timerInfoValue[1] = 0;  //importantly resets the value of timer info back to zero as we dont want the timer to repeat until a new value has been written to the timer info by the user
        }
        else 
        {
            gpio_set_level(CONFIG_GPIO_OUTPUT_48, onOffValue);    //sets the level according to the system mode variable which indicates the on or off state
        }
    }
}


void app_main(void)
{
    
    spi_bus_config_t bus = configureSPIBus();   //configure the SPI bus
    esp_err_t busError = initializeSPIBus(host_id, &bus);    //initialize the bus on the host device
    ESP_ERROR_CHECK(busError);  //check for any errors when initializing the SPI bus

    spi_device_interface_config_t redDAC = configureDAC(14);    //configure the red LED channel DAC on GPIO 14
    spi_device_interface_config_t greenDAC = configureDAC(21);  //configure green LED channel DAC on GPIO 21
    spi_device_interface_config_t blueDAC = configureDAC(47);   //configure blue LED channel DAC on GPIO 47
    
    addDAC(host_id, &redDAC, &handleRed);   //add a device on the SPI bus for the red DAC
    addDAC(host_id, &greenDAC, &handleGreen);   //add a device on the SPI bus for the green DAC
    addDAC(host_id, &blueDAC, &handleBlue); //add a device on the SPI bus for the blue DAC
    
    xTaskCreate(colorTask, "Color Control", 8091, NULL, tskIDLE_PRIORITY, NULL);    //create a task on the freeRTOS task scheduler for the SPI function
    xTaskCreate(LEDStateTask, "State Control", 8091, NULL, tskIDLE_PRIORITY, NULL); //create a task on the freeRTOS task scheduler for the FET State Control
    xTaskCreate(controlTask,  "Control", 8091, NULL, tskIDLE_PRIORITY, NULL);    //create a task on the freeRTOS task scheduler for the SPI function
}


int
gatt_svr_write(struct os_mbuf *om, uint16_t min_len, uint16_t max_len,
               void *dst, uint16_t *len)
{
    uint16_t om_len;
    int rc;

    om_len = OS_MBUF_PKTLEN(om);
    if (om_len < min_len || om_len > max_len) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(om, dst, max_len, len);
    //reverse((uint8_t*)dst);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    return 0;
}

/**
 * Access callback whenever a characteristic/descriptor is read or written to.
 * Here reads and writes need to be handled.
 * ctxt->op tells weather the operation is read or write and
 * weather it is on a characteristic or descriptor,
 * ctxt->dsc->uuid tells which characteristic/descriptor is accessed.
 * attr_handle give the value handle of the attribute being accessed.
 * Accordingly do:
 *     Append the value to ctxt->om if the operation is READ
 *     Write ctxt->om to the value if the operation is WRITE
 **/

int
gatt_svc_access(uint16_t conn_handle, uint16_t attr_handle,
                struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    const ble_uuid_t *uuid;
    int rc;

    switch (ctxt->op) {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            MODLOG_DFLT(INFO, "Characteristic read; conn_handle=%d attr_handle=%d\n",
                        conn_handle, attr_handle);
        } else {
            MODLOG_DFLT(INFO, "Characteristic read by NimBLE stack; attr_handle=%d\n",
                        attr_handle);
        }
        uuid = ctxt->chr->uuid;
        if (attr_handle == redColorValueHandle) {
            rc = os_mbuf_append(ctxt->om,
                                &redColorValue,
                                sizeof(redColorValue));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == greenColorValueHandle) {
            rc = os_mbuf_append(ctxt->om,
                                &greenColorValue,
                                sizeof(greenColorValue));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == blueColorValueHandle) {
            rc = os_mbuf_append(ctxt->om,
                                &blueColorValue,
                                sizeof(blueColorValue));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == onOffValueHandle) {
            rc = os_mbuf_append(ctxt->om,
                                &onOffValue,
                                sizeof(onOffValue));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == timerInfoValueHandle) {
            rc = os_mbuf_append(ctxt->om,
                                &timerInfoValue,
                                sizeof(timerInfoValue));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }
        if (attr_handle == lumensDisplayValueHandle) {
            rc = os_mbuf_append(ctxt->om,
                                &lumensDisplayValue,
                                sizeof(lumensDisplayValue));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        goto unknown;
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            MODLOG_DFLT(INFO, "Characteristic write; conn_handle=%d attr_handle=%d",
                        conn_handle, attr_handle);
        } else {
            MODLOG_DFLT(INFO, "Characteristic write by NimBLE stack; attr_handle=%d",
                        attr_handle);
        }
        uuid = ctxt->chr->uuid;
        if (attr_handle == redColorValueHandle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(redColorValue),
                                sizeof(redColorValue),
                                &redColorValue, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }
        if (attr_handle == greenColorValueHandle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(greenColorValue),
                                sizeof(greenColorValue),
                                &greenColorValue, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }
        if (attr_handle == blueColorValueHandle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(blueColorValue),
                                sizeof(blueColorValue),
                                &blueColorValue, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }    
        if (attr_handle == onOffValueHandle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(onOffValue),
                                sizeof(onOffValue),
                                &onOffValue, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }
        if (attr_handle == timerInfoValueHandle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(timerInfoValue),
                                sizeof(timerInfoValue),
                                &timerInfoValue, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }
        if (attr_handle == lumensDisplayValueHandle) {
            rc = gatt_svr_write(ctxt->om,
                                sizeof(lumensDisplayValue),
                                sizeof(lumensDisplayValue),
                                &lumensDisplayValue, NULL);
            ble_gatts_chr_updated(attr_handle);
            MODLOG_DFLT(INFO, "Notification/Indication scheduled for "
                        "all subscribed peers.\n");
            return rc;
        }

        goto unknown;


    case BLE_GATT_ACCESS_OP_READ_DSC:
        if (conn_handle != BLE_HS_CONN_HANDLE_NONE) {
            MODLOG_DFLT(INFO, "Descriptor read; conn_handle=%d attr_handle=%d\n",
                        conn_handle, attr_handle);
        } else {
            MODLOG_DFLT(INFO, "Descriptor read by NimBLE stack; attr_handle=%d\n",
                        attr_handle);
        }
        uuid = ctxt->dsc->uuid;
        if (ble_uuid_cmp(uuid, &gatt_svr_dsc_uuid.u) == 0) {
            rc = os_mbuf_append(ctxt->om,
                                &gatt_svr_dsc_val,
                                sizeof(gatt_svr_dsc_val));
            return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
        }

        goto unknown;

    case BLE_GATT_ACCESS_OP_WRITE_DSC:
        goto unknown;

    default:
        goto unknown;
    }

unknown:
    /* Unknown characteristic/descriptor;
     * The NimBLE host should not have called this function;
     */
    assert(0);
    return BLE_ATT_ERR_UNLIKELY;
}

void
gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int
gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_ans_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    /* Setting a value for the read-only descriptor */
    gatt_svr_dsc_val = 0x99;

    return 0;
}

/*
void reverse(uint8_t *b) {
   uint8_t byte;
   byte = b[0];
   b[0] = b[1];
   b[1] = byte;
}
*/


