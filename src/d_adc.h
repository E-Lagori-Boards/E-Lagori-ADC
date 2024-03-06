#include "driver/spi_master.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "pins_arduino.h"
    


struct d_adc_24_32{
    bool acqstate; //0 - Single acq, 1 - Continous acq
	struct boardconf brd;
	uint8_t muxstate;
    spi_device_handle_t *spi; // SPI communication 
    uint32_t memlen; // Memory length for continous acquisition
    int32_t *mem; // Memory for continous acquisition
    uint8_t *muxindex;
    float slope, bias; // Slope and Bias for ADC calibration 
    bool con_,sync_;;
    uint8_t seqlen_, *seq;
    uint32_t swtime,cnt;
    uint8_t seqind_;
    struct {
        uint8_t DRDY_, CS; 
    } pins; // pins attached to the system on E-Lagori boards
    bool memovfl; //Continous acquisiont memory overflow
};

extern "C" void dadc2432_init(struct d_adc_24_32 *p,uint8_t DRDY_, uint8_t CS, uint8_t bno); // Constructor for Single ADC
extern "C" void dadc2432_whoami(struct d_adc_24_32 *p, struct boardconf *);
extern "C" void dadc2432_chgbno(struct d_adc_24_32 *p, uint8_t);
extern "C" void dadc2432_trycon(struct d_adc_24_32 *p);
// bool dadc2432_con(struct d_adc_24_32 *p){return this->con_;};
extern "C" void dadc2432_attachSPI(struct d_adc_24_32 *p,spi_device_handle_t *); // Attach SPI to Single ADC
extern "C" void dadc2432_SetMux(struct d_adc_24_32 *p,uint8_t m); // Set Analog signal channel
// uint8_t dadc2432_seqind(struct d_adc_24_32 *p,){return this->seqind_;};
extern "C" void dadc2432_initcontacq(struct d_adc_24_32 *p, uint32_t memlen,int32_t *mem, uint8_t *muxindex); // Initiate Contnous acquisition mode
extern "C" int dadc2432_muxseq(struct d_adc_24_32 *p, float, uint8_t, uint8_t *, bool);
extern "C" void dadc2432_termcontacq(struct d_adc_24_32 *p); // Terminate continous acquisitionuint32_t changMCLK(uint32_t);
extern "C" uint8_t dadc2432_getmuxstate(struct d_adc_24_32 *p);
extern "C" bool dadc2432_powerdown(struct d_adc_24_32 *p);
extern "C" bool dadc2432_powerup(struct d_adc_24_32 *p);
extern "C" void dadc2432_applycalcorr(struct d_adc_24_32 *p, float, float); // Apply correction factors for ADC calibration        
extern "C" float dadc2432_getADCval(struct d_adc_24_32 *p); // Get single ADC value
extern "C" bool dadc2432_getacqstate(struct d_adc_24_32 *p); // Present acquisition state - Continous or single value
