#ifndef _BSP_SPI_H_
#define _BSP_SPI_H_

#include "struct_typedef.h"

void SPI1_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);

void SPI1_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);


#endif
