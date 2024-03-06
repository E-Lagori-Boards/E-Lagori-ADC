#include <d_esp_wrv.h>
#include <d_adc.h>
#define MEMLEN 1024

struct d_esp_wvr_7_00 proc;
struct boardconf b;
struct d_adc_24_32 adc;

uint8_t i = 0;

int32_t mem[MEMLEN];
uint8_t muxseq[MEMLEN] = { 0 };
uint8_t seq[] = { I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15, I16 };


void setup() {
  Serial.begin(115200);
  Serial.println("Initial Board");
  // extern "C"
  dewvr_init(&proc, 0x81, 6);
  dadc2432_init(&adc, C_G, C_P, 0x82);
  // put your setup code here, to run once:
}

void loop() {
  Serial.print("Board Information :");

  dewvr_whoami(&proc, &b);

  Serial.print(b.reserved);
  Serial.print(" ");
  Serial.print(b.brdtype);
  Serial.print(" ");
  Serial.print(b.brdno);
  Serial.print(" \n");

  dadc2432_whoami(&adc, &b);

  Serial.print("ADC Information :");
  Serial.print(b.reserved);
  Serial.print(" ");
  Serial.print(b.brdtype);
  Serial.print(" ");
  Serial.print(b.brdno);
  Serial.print(" \n");

  dewvr_trycon(&proc);
  if (proc.con_) {
    Serial.println("Connection to Processor Succesfull");
  } else {
    Serial.println("Connection to Processor Not Succesfull");
  }

  dadc2432_trycon(&adc);
  if (adc.con_) {
    Serial.println("Connection to ADC Succesfull");
  } else {
    Serial.println("Connection to ADC Not Succesfull");
  }

  // dewvr_spiBegin(&proc,400000);
  Serial.print("The Processor SPI Address is :");
  Serial.println((long)proc.spi);

  dadc2432_attachSPI(&adc, &proc.spi);
  Serial.print("The ADC SPI Address is :");
  Serial.println((long)*adc.spi);

  dadc2432_SetMux(&adc, I1);
  Serial.print("The ADC MUXSTATE:");
  Serial.println(dadc2432_getmuxstate(&adc));


  Serial.println("---------------------------------");

  delay(2000);
}
