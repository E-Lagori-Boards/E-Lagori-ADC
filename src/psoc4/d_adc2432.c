/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/

#include "d_adc2432.h"

**<
 * @brief Initializes a single ADC device.
 *
 * This function initializes a single ADC device with the given board number.
 *
 * @param p Pointer to the ADC device structure.
 * @param b Board number of the ADC device.
 */
void dadc2432_init(struct d_adc_24_32 *p, uint8_t b){ // Constructor for Single ADC
    p->brd.reserved = 0x80;
    p->brd.brdtype = 0x9B;
    p->brd.brdno = b;
    p->slope = 1;
    p->bias = 0;
    p->memovfl = 0;
    p->muxstate = 0;
    
    PWM_MCLK_Start();
    CS_Write(1);
}

/**<
 * @brief Changes the board number of an ADC device.
 *
 * This function changes the board number of an ADC device.
 *
 * @param p Pointer to the ADC device structure.
 * @param bno New board number of the ADC device.
 */
void dadc2432_chgbno(struct d_adc_24_32 *p, uint8_t bno){
    uint8_t temp = 0x60;
    UART_AMOSI_PutChar(p->brd.reserved);
    UART_AMOSI_PutChar(p->brd.brdtype);
    UART_AMOSI_PutChar(p->brd.brdno);
    UART_AMOSI_PutChar(temp);
    UART_AMOSI_PutChar(bno);
    CyDelay(5);
}

/**<
 * @brief Sets the analog signal channel.
 *
 * This function sets the analog signal channel of an ADC device.
 *
 * @param p Pointer to the ADC device structure.
 * @param m Channel number (0-15).
 */
void dadc2432_SetMux(struct d_adc_24_32 *p,uint8_t m){ // Set Analog signal channel
    UART_AMOSI_PutChar(p->brd.reserved);
    UART_AMOSI_PutChar(p->brd.brdtype);
    UART_AMOSI_PutChar(p->brd.brdno);
    uint8_t temp = (m > 15) ? 15 : m;
    UART_AMOSI_PutChar(temp);
    CyDelay(5);
}
// uint8_t dadc2432_seqind(struct d_adc_24_32 *p,){return this->seqind_;};
//uint32 adc_val[MEMLEN];
//uint8 Memovfl;
//CY_ISR( DRDY_ISR_Handler ){
//
//    uint8 buf[4] = {0,0,0,0};
//    uint32 rec_data;
//    
//    static uint32 cnt = 0;
//    spiRead(buf, 4, (uint8*)&rec_data);
//    adc_val[cnt % MEMLEN] = rec_data;
//    cnt++;
//    Memovfl = (cnt % MEMLEN)?Memovfl:1;
//    DRDY_ClearInterrupt();
//}

/**<
 * @brief Initializes continuous acquisition mode.
 *
 * This function initializes continuous acquisition mode for an ADC device.
 *
 * @param p Pointer to the ADC device structure.
 * @param memlen Length of the memory buffer.
 * @param mem Pointer to the memory buffer.
 * @param muxindex Pointer to the mux index array.
 */
void dadc2432_initcontacq(struct d_adc_24_32 *p, uint32_t memlen,int32_t *mem, uint8_t *muxindex){// Initiate Contnous acquisition mode - Interrupt handler should be assigned externally
    p->acqstate = 1;
    p->mem = mem;
    p->muxindex = muxindex;
    p->memlen = memlen-1;
    DRDY_ISR_Enable();
    DRDY_ISR_Start();
}

/**<
 * @brief Configures the mux sequence.
 *
 * This function configures the mux sequence for an ADC device.
 *
 * @param p Pointer to the ADC device structure.
 * @param swtime Switching time (in seconds).
 * @param len Length of the sequence.
 * @param seq Pointer to the sequence array.
 * @param sync Synchronization mode (0 or 1).
 *
 * @return 0 on success, -1 on error.
 */
int dadc2432_muxseq(struct d_adc_24_32 *p, float swtime, uint8_t len, uint8_t *seq, uint8_t sync){
    uint8_t temp;
    uint8_t swtime_t;
    if (swtime < 4) // 5 msec
    {
        p->swtime = (uint32_t)(swtime*1000.0);
        swtime_t = swtime;//round(swtime/1.3/2);
        p->sync_ = sync;
        UART_AMOSI_PutChar(p->brd.reserved);
        UART_AMOSI_PutChar(p->brd.brdtype);
        UART_AMOSI_PutChar(p->brd.brdno);
        temp = 0x30 | p->sync_;
        UART_AMOSI_PutChar(temp);
        UART_AMOSI_PutChar(swtime_t);

        if (len == 0){
        return -1;
        }
        UART_AMOSI_PutChar(p->brd.reserved);
        UART_AMOSI_PutChar(p->brd.brdtype);
        UART_AMOSI_PutChar(p->brd.brdno);
        p->seqlen_ = (len > 16)?16:len;
        temp = 0x10|(p->seqlen_-1);
        UART_AMOSI_PutChar(temp);
        p->seq = seq;
        for (uint8_t i = 0; i<p->seqlen_; i++)
            p->seq[i] = (p->seq[i] > 15)?15:p->seq[i];

        for (uint8_t i = 0; i<p->seqlen_; i++)
            UART_AMOSI_PutChar(p->seq[i]);

//        //setup interrupt
//        gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
//        gpio_set_intr_type(p->pins.CS, GPIO_INTR_POSEDGE);
//        gpio_isr_handler_add(p->pins.CS, isr_adc2432_mux, (void*) p);
//        gpio_intr_enable(p->pins.CS);
        return 0;
    }
    return -1;
}

/**<
 * @brief Terminates continuous acquisition mode.
 *
 * This function terminates continuous acquisition mode for an ADC device.
 *
 * @param p Pointer to the ADC device structure.
 */
void dadc2432_termcontacq(struct d_adc_24_32 *p){ // Terminate continous acquisitionuint32_t changMCLK(uint32_t);
    DRDY_ISR_Stop();
}

/**<
 * @brief Powers down an ADC device.
 *
 * This function powers down an ADC device.
 *
 * @param p Pointer to the ADC device structure.
 *
 * @return 0 on success.
 */
uint8_t dadc2432_powerdown(struct d_adc_24_32 *p){
    uint8_t tempa[3];
	uint8_t tempstat = 0;
	
    UART_AMOSI_PutChar(p->brd.reserved);
    UART_AMOSI_PutChar(p->brd.brdtype);
    UART_AMOSI_PutChar(p->brd.brdno);
    char temp = 0x40;
    UART_AMOSI_PutChar(temp);
    
    UART_AMOSI_PutChar(p->brd.reserved);
    UART_AMOSI_PutChar(p->brd.brdtype);
    UART_AMOSI_PutChar(p->brd.brdno);
    temp = 0x50;
    UART_AMOSI_PutChar(temp);
    
    return 0;
}

/**<
 * @brief Powers up an ADC device.
 *
 * This function powers up an ADC device.
 *
 * @param p Pointer to the ADC device structure.
 *
 * @return 0 on success.
 */
uint8_t dadc2432_powerup(struct d_adc_24_32 *p){
    uint8_t tempa[3];
	uint8_t tempstat = 0;

    UART_AMOSI_PutChar(p->brd.reserved);
    UART_AMOSI_PutChar(p->brd.brdtype);
    UART_AMOSI_PutChar(p->brd.brdno);
    char temp = 0x41;
     UART_AMOSI_PutChar(temp);
    
    UART_AMOSI_PutChar(p->brd.reserved);
    UART_AMOSI_PutChar(p->brd.brdtype);
    UART_AMOSI_PutChar(p->brd.brdno);
    temp = 0x50;
    UART_AMOSI_PutChar(temp);

    return 0;
}

/**<
 * @brief Applies calibration correction factors.
 *
 * This function applies calibration correction factors to an ADC device.
 *
 * @param p Pointer to the ADC device structure.
 * @param s Slope correction factor.
 * @param c Bias correction factor.
 */
void dadc2432_applycalcorr(struct d_adc_24_32 *p, float s, float c){ // Apply correction factors for ADC calibration 
    p->slope = s;
    p->bias = c;
}

/**<
 * @brief Reads data from the SPI bus.
 *
 * This function reads data from the SPI bus.
 *
 * @param buf Pointer to the buffer to store the read data.
 * @param size Number of bytes to read.
 * @param rec_data Pointer to the received data.
 */
void spiRead(uint8 *buf, uint16 size, uint8 *rec_data){
    uint8 i;
//    SPI_Start();
    CS_Write(0);
    SPIM_SpiUartPutArray(buf, size);
    while(SPIM_SpiUartGetRxBufferSize() != (uint32)size){}
    SPIM_SpiUartClearTxBuffer();
    i=0;
    while( SPIM_SpiUartGetRxBufferSize() != 0 ){ 
        rec_data[i] = SPIM_SpiUartReadRxData();
        i++;
    }
    CS_Write(1);
}

/**<
 * @brief Gets a single ADC value.
 *
 * This function gets a single ADC value from an ADC device.
 *
 * @param p Pointer to the ADC device structure.
 *
 * @return The ADC value.
 */
float dadc2432_getADCval(struct d_adc_24_32 *p){
     uint32_t data = 0,adj_data = 0;
     int32_t *d;
     uint8 buf[4] = {0, 0, 0, 0};
     uint8 *point, *adj_point;
     point = (uint8*)&data;
    p->acqstate = 0;

    CS_Write(1);
    
    unsigned int t = 2000;
    while(DRDY_Read() == 1){if (!(t--)) return -1;};
    
    CS_Write(0);
    spiRead(buf, 3, (uint8*)&data);
    
    point = (uint8*)&data;
    adj_point = (uint8*)&adj_data;
    adj_point[3] = point[3];
    adj_point[0] = point[2];
    adj_point[1] = point[1];
    adj_point[2] = point[0];

    uint32_t sign = adj_data & 0x00800000;
    adj_data = sign?(adj_data | 0xFF000000):adj_data;
    d = (int32_t*)&adj_data;
    return (*d*119.20928955078125e-9+0.5);
    //return (*d*119.20928955078125e-9 *5 +2.5);
}; // Get single ADC value

/**<
 * @brief Gets the acquisition state of an ADC device.
 *
 * This function gets the acquisition state of an ADC device.
 *
 * @param p Pointer to the ADC device structure.
 *
 * @return 0 if single value acquisition, 1 if continuous acquisition.
 */
uint8_t dadc2432_getacqstate(struct d_adc_24_32 *p){; // Present acquisition state - Continous or single value
    return p->acqstate;
}

/* [] END OF FILE */
