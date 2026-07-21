

#ifndef CAN_DMA_BUFFER_H
#define CAN_DMA_BUFFER_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

#define CAN_DMA_RING_SIZE       16U

#define CAN_DMA_MAX_PAYLOAD     64U

#define CAN_DMA_MDMA_TIMEOUT_MS  2U

typedef struct __attribute__((aligned(32)))
{
    uint32_t         identifier;     /* 4 byte */
    uint32_t         timestamp_ms;   /* 4 byte */
    uint8_t          id_type;        /* 1 byte */
    uint8_t          data_length;    /* 1 byte */
    volatile uint8_t ready;          /* 1 byte */
    volatile uint8_t dma_active;     /* 1 byte */
    uint8_t          _padding[20];   /* 32'ye tamamlayan boşluk (12 + 20 = 32) */


    /* SADECE MDMA buraya yazar. CPU sadece okur. */
    uint8_t          data[CAN_DMA_MAX_PAYLOAD];
} can_dma_frame_t;



void can_dma_buffer_init(void);

void can_dma_buffer_push_from_isr(FDCAN_HandleTypeDef *hfdcan);


uint8_t can_dma_buffer_pop(can_dma_frame_t *out_frame);


uint8_t can_dma_buffer_is_empty(void);


uint32_t can_dma_buffer_get_overrun_count(void);


void can_dma_buffer_reset_stats(void);

#endif /* CAN_DMA_BUFFER_H */
