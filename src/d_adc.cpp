// #include "d_esp_wrv.h"
#include "d_adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
// #include "sdkconfig.h"
#include "pins_arduino.h"

bool state = 0;

spi_transaction_t trans = {
    .flags = SPI_TRANS_USE_RXDATA,
    .cmd = 0,
    .addr = 0,
    .length = 24,
    .rxlength = 0};

void isr_adc2432(void *arg)
{
    uint32_t sign;
    int32_t temp;
    struct d_adc_24_32 *s = (struct d_adc_24_32 *)arg;
    static bool prevint = 0;
    // static spi_transaction_t transptr[64];
    if (!gpio_get_level((gpio_num_t)s->pins.DRDY_) && !(s->memovfl))
    {
        prevint = 0;
        gpio_set_intr_type((gpio_num_t)s->pins.CS, GPIO_INTR_DISABLE);
        gpio_set_direction((gpio_num_t)s->pins.CS, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)s->pins.CS, 0);
        spi_device_polling_transmit(*(s->spi), &trans);
        gpio_set_level((gpio_num_t)s->pins.CS, 1);
        gpio_set_direction((gpio_num_t)s->pins.CS, GPIO_MODE_INPUT);
        gpio_set_intr_type((gpio_num_t)s->pins.CS, GPIO_INTR_POSEDGE);
        temp = SPI_SWAP_DATA_RX(*((int32_t *)trans.rx_data), 24);
        sign = temp & 0x00800000;
        s->mem[s->cnt] = sign ? (temp | 0xFF000000) : temp;
        if (s->muxindex)
            s->muxindex[s->cnt] = s->seqind_;
        s->cnt = (s->cnt == s->memlen - 1) ? 0 : s->cnt + 1;
        s->memovfl = (!s->cnt) ? 1 : s->memovfl;
        //
    }
    if (gpio_get_level((gpio_num_t)s->pins.DRDY_) & !prevint)
    {
        // gpio_set_level((gpio_num_t)C_O, state);
        prevint = 1;
        s->seqind_++;
        // state = !state;
        // gpio_set_level((gpio_num_t)C_O, 0);
    }
}

void IRAM_ATTR isr_adc2432_mux(void *arg)
{
    struct d_adc_24_32 *s = (struct d_adc_24_32 *)arg;
    // gpio_set_level((gpio_num_t)C_O, state);
    // state = !state;
    s->seqind_ = 0;
}

void dadc2432_init(struct d_adc_24_32 *p, uint8_t DRDY_, uint8_t CS, uint8_t bno) // Constructor for Single ADC
{
    p->slope = 1;
    p->bias = 1;
    p->cnt = 0;
    p->memovfl = 0;

    p->pins.DRDY_ = DRDY_;
    p->pins.CS = CS;

    static gpio_config_t confIO = {
        //   .pin_bit_mask = ((unsigned long long)1<< (p->pins.DRDY_)),     /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
        .mode = GPIO_MODE_INPUT,               /*!< GPIO mode: set input/output mode                     */
        .pull_up_en = GPIO_PULLUP_DISABLE,     /*!< GPIO pull-up                                         */
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /*!< GPIO pull-down                                       */
        .intr_type = GPIO_INTR_DISABLE         /*!< GPIO interrupt type - previously set                 */
    };
    confIO.pin_bit_mask = ((unsigned long long)1 << (p->pins.DRDY_));
    gpio_config(&confIO);

    static gpio_config_t confIO1 = {
        //   .pin_bit_mask = ((unsigned long long)1<< (p->pins.CS)),     /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
        .mode = GPIO_MODE_INPUT,               /*!< GPIO mode: set input/output mode                     */
        .pull_up_en = GPIO_PULLUP_DISABLE,     /*!< GPIO pull-up                                         */
        .pull_down_en = GPIO_PULLDOWN_DISABLE, /*!< GPIO pull-down                                       */
        .intr_type = GPIO_INTR_DISABLE         /*!< GPIO interrupt type - previously set                 */
    };
    confIO1.pin_bit_mask = ((unsigned long long)1 << (p->pins.CS));
    gpio_config(&confIO1);

    p->brd.reserved = 0B10000000;
    p->brd.brdtype = 0B10011011;
    p->brd.brdno = bno;
}

void dadc2432_whoami(struct d_adc_24_32 *p, struct boardconf *b)
{
    uint8_t temp = 0x80;
    uart_flush(INTCOM);
    uart_write_bytes(INTCOM, &(p->brd.reserved), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdtype), 1);
    uart_write_bytes(INTCOM, &(temp), 1);
    // vTaskDelay(5 / portTICK_PERIOD_MS);
    uint8_t tempa[3] = {0, 0, 0};

    if (uart_read_bytes(INTCOM, tempa, 3, 10 / portTICK_PERIOD_MS) >= 0)
    {
        b->reserved = tempa[0];
        b->brdtype = tempa[1];
        b->brdno = tempa[2];
        return;
    }
    b->reserved = tempa[0];
    b->brdtype = tempa[1];
    b->brdno = tempa[2];
}

void dadc2432_chgbno(struct d_adc_24_32 *p, uint8_t bno)
{
    uint8_t temp = 0x60;
    uart_flush(INTCOM);
    uart_write_bytes(INTCOM, &(p->brd.reserved), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdtype), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdno), 1);
    uart_write_bytes(INTCOM, &(temp), 1);
    uart_write_bytes(INTCOM, &(bno), 1);
    vTaskDelay(5 / portTICK_PERIOD_MS);
    uart_flush(INTCOM);
    dadc2432_whoami(p, &(p->brd));
}

void dadc2432_trycon(struct d_adc_24_32 *p)
{
    uart_flush(INTCOM);
    uart_write_bytes(INTCOM, &(p->brd.reserved), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdtype), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdno), 1);
    uint8_t temp = 0x70;
    uart_write_bytes(INTCOM, &(temp), 1);
    // vTaskDelay(5 / portTICK_PERIOD_MS);
    uint8_t tempa[4];
    p->con_ = 0;
    if (uart_read_bytes(INTCOM, tempa, 4, 10 / portTICK_PERIOD_MS) == 4)
    {
        if ((tempa[0] == p->brd.reserved) || (tempa[1] == p->brd.brdtype) || (tempa[2] == p->brd.brdno) || (tempa[3] == 0x61))
            p->con_ = 1;
        return;
    }
}

void dadc2432_attachSPI(struct d_adc_24_32 *p, spi_device_handle_t *spi) // Attach SPI to Single ADC
{
    p->spi = spi;
}

void dadc2432_SetMux(struct d_adc_24_32 *p, uint8_t m) // Set Analog signal channel
{
    uart_flush(INTCOM);
    uart_write_bytes(INTCOM, &(p->brd.reserved), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdtype), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdno), 1);
    uint8_t temp = (m > 15) ? 15 : m;
    uart_write_bytes(INTCOM, &(temp), 1);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void dadc2432_initcontacq(struct d_adc_24_32 *p, uint32_t memlen, int32_t *mem, uint8_t *muxindex) // Initiate Contnous acquisition mode
{
    p->acqstate = 1;
    p->mem = mem;
    p->muxindex = muxindex;
    p->memlen = memlen - 1;

    gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    gpio_set_intr_type((gpio_num_t)p->pins.DRDY_, GPIO_INTR_ANYEDGE);
    ESP_ERROR_CHECK(gpio_isr_handler_add((gpio_num_t)p->pins.DRDY_, isr_adc2432, (void *)p));
    gpio_intr_enable((gpio_num_t)p->pins.DRDY_);

    // ESP_ERROR_CHECK(spi_device_acquire_bus(*(p->spi), 1));
    // ESP_ERROR_CHECK(spi_device_queue_trans(*(p->spi),&trans, 1));
}

int dadc2432_muxseq(struct d_adc_24_32 *p, float swtime, uint8_t len, uint8_t *seq, bool sync)
{
    uint8_t temp;
    uint8_t swtime_t;
    if (swtime < 4) // 5 msec
    {
        p->swtime = (uint32_t)(swtime * 1000.0);
        swtime_t = swtime; // round(swtime/1.3/2);
        p->sync_ = sync;
        uart_write_bytes(INTCOM, &(p->brd.reserved), 1);
        uart_write_bytes(INTCOM, &(p->brd.brdtype), 1);
        uart_write_bytes(INTCOM, &(p->brd.brdno), 1);
        temp = 0x30 | p->sync_;
        uart_write_bytes(INTCOM, &temp, 1);
        uart_write_bytes(INTCOM, &(swtime_t), 1);

        if (len == 0)
        {
            return -1;
        }
        uart_write_bytes(INTCOM, &(p->brd.reserved), 1);
        uart_write_bytes(INTCOM, &(p->brd.brdtype), 1);
        uart_write_bytes(INTCOM, &(p->brd.brdno), 1);
        p->seqlen_ = (len > 16) ? 16 : len;
        temp = 0x10 | (p->seqlen_ - 1);
        uart_write_bytes(INTCOM, &temp, 1);
        p->seq = seq;
        for (uint8_t i = 0; i < p->seqlen_; i++)
            p->seq[i] = (p->seq[i] > 15) ? 15 : p->seq[i];

        for (uint8_t i = 0; i < p->seqlen_; i++)
            uart_write_bytes(INTCOM, &(p->seq[i]), 1);

        // setup interrupt
        gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
        gpio_set_intr_type((gpio_num_t)p->pins.CS, GPIO_INTR_POSEDGE);
        gpio_isr_handler_add((gpio_num_t)p->pins.CS, isr_adc2432_mux, (void *)p);
        gpio_intr_enable((gpio_num_t)p->pins.CS);
        return 0;
    }
    return -1;
}

void dadc2432_termcontacq(struct d_adc_24_32 *p) // Terminate continous acquisitionuint32_t changMCLK(uint32_t);
{
    gpio_isr_handler_remove((gpio_num_t)p->pins.DRDY_);
}

uint8_t dadc2432_getmuxstate(struct d_adc_24_32 *p)
{
    char temprec[4];

    uart_flush(INTCOM);
    uart_write_bytes(INTCOM, &(p->brd.reserved), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdtype), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdno), 1);
    char temp = 0x20;
    uart_write_bytes(INTCOM, &temp, 1);
    // vTaskDelay(5/portTICK_PERIOD_MS);

    if (uart_read_bytes(INTCOM, temprec, 4, 10 / portTICK_PERIOD_MS) == 4)
    {
        p->muxstate = 16;
        // ESP_LOGI("UART","%x %x %x %x",temprec[0],temprec[1],temprec[2],temprec[3]);
        if ((temprec[0] == p->brd.reserved) && (temprec[1] == p->brd.brdtype) && (temprec[2] == p->brd.brdno))
            p->muxstate = temprec[3] & 0x0F;
        return (p->muxstate);
    }
    return 16;
}

bool dadc2432_powerdown(struct d_adc_24_32 *p)
{
    uint8_t tempa[4];
    uint8_t tempstat = 0;

    uart_flush(INTCOM);
    uart_write_bytes(INTCOM, &(p->brd.reserved), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdtype), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdno), 1);
    char temp = 0x40;
    uart_write_bytes(INTCOM, &temp, 1);

    uart_write_bytes(INTCOM, &(p->brd.reserved), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdtype), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdno), 1);
    temp = 0x50;
    uart_write_bytes(INTCOM, &temp, 1);

    if (uart_read_bytes(INTCOM, tempa, 4, 10 / portTICK_PERIOD_MS) == 4)
    {
        if ((tempa[0] == p->brd.reserved) || (tempa[1] == p->brd.brdtype) || (tempa[2] == p->brd.brdno))
            tempstat = tempa[3] & 0x0F;
        return (tempstat);
    }
    return 0;
}

bool dadc2432_powerup(struct d_adc_24_32 *p)
{
    uint8_t tempa[4];
    uint8_t tempstat = 0;

    // char *temp;
    // temp = (char*)&(p->brd);
    uart_flush(INTCOM);
    uart_write_bytes(INTCOM, &(p->brd.reserved), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdtype), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdno), 1);
    char temp = 0x41;
    uart_write_bytes(INTCOM, &temp, 1);

    uart_write_bytes(INTCOM, &(p->brd.reserved), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdtype), 1);
    uart_write_bytes(INTCOM, &(p->brd.brdno), 1);
    temp = 0x50;
    uart_write_bytes(INTCOM, &temp, 1);

    if (uart_read_bytes(INTCOM, tempa, 4, 10 / portTICK_PERIOD_MS) == 4)
    {
        if ((tempa[0] == p->brd.reserved) || (tempa[1] == p->brd.brdtype) || (tempa[2] == p->brd.brdno))
            tempstat = tempa[3] & 0x0F;
        return (tempstat);
    }
    return 0;
}

void dadc2432_applycalcorr(struct d_adc_24_32 *p, float s, float c) // Apply correction factors for ADC calibration
{
    p->slope = s;
    p->bias = c;
}

float dadc2432_getADCval(struct d_adc_24_32 *p) // Get single ADC value
{
    int32_t data = 0;
    p->acqstate = 0;

    gpio_set_level((gpio_num_t)p->pins.CS, 1);

    unsigned int t = 200;
    while (gpio_get_level((gpio_num_t)p->pins.DRDY_) == 1)
    {
        if (!(t--))
            return -1;
    };
    while (gpio_get_level((gpio_num_t)p->pins.DRDY_) == 0)
    {
        if (!(t--))
            return -2;
    };
    while (gpio_get_level((gpio_num_t)p->pins.DRDY_) == 1)
    {
        if (!(t--))
            return -3;
    };

    gpio_set_level((gpio_num_t)p->pins.CS, 0);
    // data = ((uint32_t)p->spi->transfer32(0))>>8;//this->spi->transfer32(0);
    ESP_ERROR_CHECK(spi_device_transmit(*(p->spi), &trans));
    data = SPI_SWAP_DATA_RX(*((int32_t *)trans.rx_data), 24);
    // ESP_LOGI("DATA", "%ld %x %x %x %x", data, trans.rx_data[0],trans.rx_data[1],trans.rx_data[2],trans.rx_data[3]);
    gpio_set_level((gpio_num_t)p->pins.CS, 1);
    // pinMode(this->pins.CS, INPUT);
    uint32_t sign = data & 0x00800000;
    data = sign ? (data | 0xFF000000) : data;
    return (data * 119.20928955078125e-9 * 5 + 2.5);
}

bool dadc2432_getacqstate(struct d_adc_24_32 *p) // Present acquisition state - Continous or single value
{

    return p->acqstate;
}
