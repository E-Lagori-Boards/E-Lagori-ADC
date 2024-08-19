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
#ifndef D_ADC2432_H
#define D_ADC2432_H

//#include <project.h>
#include "D_PSC4\Boardconf.h"

    
extern long intrdet;

struct d_adc_24_32{
    uint8_t acqstate; //0 - Single acq, 1 - Continous acq
	struct boardconf brd;
	uint8_t muxstate;
    uint32_t memlen; // Memory length for continous acquisition
    int32_t *mem; // Memory for continous acquisition
    uint8_t *muxindex;
    float slope, bias; // Slope and Bias for ADC calibration 
    uint8_t con_,sync_;;
    uint8_t seqlen_, *seq;
    uint32_t swtime,cnt;
    uint8_t seqind_;
    struct {
        uint8_t DRDY_, CS; 
    } pins; // pins attached to the system on E-Lagori boards
    uint8_t memovfl; //Continous acquisiont memory overflow
};

void dadc2432_init(struct d_adc_24_32 *p, uint8_t b); // Constructor for Single ADC
void dadc2432_chgbno(struct d_adc_24_32 *p, uint8_t);
void dadc2432_SetMux(struct d_adc_24_32 *p,uint8_t m); // Set Analog signal channel
// uint8_t dadc2432_seqind(struct d_adc_24_32 *p,){return this->seqind_;};
void dadc2432_initcontacq(struct d_adc_24_32 *p, uint32_t memlen,int32_t *mem, uint8_t *muxindex); // Initiate Contnous acquisition mode
int dadc2432_muxseq(struct d_adc_24_32 *p, float, uint8_t, uint8_t *, uint8_t);
void dadc2432_termcontacq(struct d_adc_24_32 *p); // Terminate continous acquisitionuint32_t changMCLK(uint32_t);
uint8_t dadc2432_getmuxstate(struct d_adc_24_32 *p);
uint8_t dadc2432_powerdown(struct d_adc_24_32 *p);
uint8_t dadc2432_powerup(struct d_adc_24_32 *p);
void dadc2432_applycalcorr(struct d_adc_24_32 *p, float, float); // Apply correction factors for ADC calibration        
float dadc2432_getADCval(struct d_adc_24_32 *p); // Get single ADC value
uint8_t dadc2432_getacqstate(struct d_adc_24_32 *p); // Present acquisition state - Continous or single value

#endif
/* [] END OF FILE */
