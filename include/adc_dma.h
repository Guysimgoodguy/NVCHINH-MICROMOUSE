#ifndef ADC_DMA_H
#define ADC_DMA_H

#include <Arduino.h>

extern volatile uint16_t adcValue;

void adc_dma_init(void);

#endif