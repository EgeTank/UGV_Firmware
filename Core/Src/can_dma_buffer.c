#include "can_dma_buffer.h"
#include <string.h>


extern MDMA_HandleTypeDef hmdma_mdma_channel0_sw_0;

/* ============================================================
   DERLEME-ZAMANI KONTROL
   ============================================================ */
#if (CAN_DMA_RING_SIZE & (CAN_DMA_RING_SIZE - 1U)) != 0U
    #error "CAN_DMA_RING_SIZE must be a power of 2 (8, 16, 32...)"
#endif

#define RING_MASK  (CAN_DMA_RING_SIZE - 1U)


#define FDCAN_FIFO_ELEMENT_SIZE_WORDS   18U    /* 2 header + 16 data = 72 byte */
#define FDCAN_FIFO_ELEMENT_SIZE_BYTES   72U
#define FDCAN_FIFO_HEADER_SIZE_BYTES     8U    /* R0 + R1 */
#define FDCAN_FIFO_DATA_OFFSET_BYTES     8U    /* Payload, header'dan sonra */

/* R0 bitleri */
#define FDCAN_R0_STDID_SHIFT    18U
#define FDCAN_R0_STDID_MASK     0x1FFCU0000U   /* bits [28:18] */
#define FDCAN_R0_XTD_BIT        (1U << 30)

/* R1 bitleri */
#define FDCAN_R1_DLC_SHIFT      16U
#define FDCAN_R1_DLC_MASK       0x000F0000U    /* bits [19:16] */


static can_dma_frame_t __attribute__((aligned(32))) ring[CAN_DMA_RING_SIZE];

static volatile uint32_t wr_idx      = 0U;
static volatile uint32_t rd_idx      = 0U;
static volatile uint32_t overrun_cnt = 0U;




static uint8_t dlc_to_bytes(uint8_t dlc)
{
    static const uint8_t dlc_table[16] =
        { 0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64 };
    return (dlc < 16U) ? dlc_table[dlc] : 64U;
}


void can_dma_buffer_init(void)
{
    wr_idx      = 0U;
    rd_idx      = 0U;
    overrun_cnt = 0U;
    memset(ring, 0, sizeof(ring));
    for (uint32_t i = 0 ; i < CAN_DMA_RING_SIZE; i++)
    {
    	ring[i].dma_active = 0U;
    }
}


void can_dma_buffer_push_from_isr(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t next_wr = (wr_idx + 1U) & RING_MASK;

    /* 1. OVERRUN BLOĞU */
    if (next_wr == rd_idx)
    {
        overrun_cnt++;
        /* DÜZELTME EKLENDİ (8 bit sağa kaydır) */
        uint32_t get_idx = ((hfdcan->Instance->RXF0S >> 8U) & 0x3FU);
        hfdcan->Instance->RXF0A = get_idx;
        return;
    }

    uint32_t slot = wr_idx;
    // Bu slot zaten aktif MDMA mı bekliyor? (güvenlik)

    /* 2. DMA ACTIVE GÜVENLİK BLOĞU */
    if (ring[slot].dma_active)
    {
        // Çok nadir olur, olursa FIFO'yu acknowledge et ve çık
        /* DÜZELTME EKLENDİ (8 bit sağa kaydır) */
        uint32_t get_idx = ((hfdcan->Instance->RXF0S >> 8U) & 0x3FU);
        hfdcan->Instance->RXF0A = get_idx;
        return;
    }

    /* 3. NORMAL AKIŞ BLOĞU */
    /* DÜZELTME EKLENDİ (8 bit sağa kaydır) */
    uint32_t get_idx   = ((hfdcan->Instance->RXF0S >> 8U) & 0x3FU);
    uint32_t elem_addr = hfdcan->msgRam.RxFIFO0SA
                         + (get_idx * FDCAN_FIFO_ELEMENT_SIZE_WORDS * 4U);

    volatile uint32_t *mram = (volatile uint32_t *)elem_addr;
    uint32_t r0 = mram[0];
    uint32_t r1 = mram[1];

    ring[slot].identifier  = (r0 >> FDCAN_R0_STDID_SHIFT) & 0x7FFU;
    ring[slot].id_type     = (r0 & FDCAN_R0_XTD_BIT) ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;

    uint8_t dlc_code = (uint8_t)((r1 & FDCAN_R1_DLC_MASK) >> FDCAN_R1_DLC_SHIFT);
    ring[slot].data_length = dlc_to_bytes(dlc_code);
    ring[slot].timestamp_ms = HAL_GetTick();
    ring[slot].ready = 0U;
    ring[slot].dma_active = 1U;   // ← ARTIK AKTİF

    uint32_t mram_payload_addr = elem_addr + FDCAN_FIFO_DATA_OFFSET_BYTES;

    //Tüm data alanını temizler (İyileştirilmiş olan bölge)
    SCB_CleanDCache_by_Addr((uint32_t *)ring[slot].data , CAN_DMA_MAX_PAYLOAD);

    if (HAL_MDMA_Start_IT(&hmdma_mdma_channel0_sw_0,
                          mram_payload_addr,
                          (uint32_t)ring[slot].data,
                          ring[slot].data_length,
                          1U) == HAL_OK)
    {
        wr_idx = next_wr;   // Başarılı, ring indeksini ilerlet
    }
    else
    {
        // MDMA başlatılamadı – fallback memcpy
        memcpy(ring[slot].data, (const void *)mram_payload_addr, ring[slot].data_length);
        ring[slot].ready = 1U;
        ring[slot].dma_active = 0U;
        wr_idx = next_wr;
    }

    // FIFO'yu acknowledge et
    hfdcan->Instance->RXF0A = get_idx;
}

/*
   HAL_MDMA_XferCpltCallback()

   MDMA tamamlandı → ring[slot].data artık geçerli.
   D-Cache invalidate: CPU eski (stale) cache okumadan önce temizle.
    */
void HAL_MDMA_XferCpltCallback(MDMA_HandleTypeDef *hmdma)
{
    if (hmdma != &hmdma_mdma_channel0_sw_0) return;

    // Hangi slot aktif ve ready değil? (Ring küçük, arama yap)
    for (uint32_t i = 0; i < CAN_DMA_RING_SIZE; i++)
    {
        if (ring[i].dma_active && !ring[i].ready)
        {
            SCB_InvalidateDCache_by_Addr((uint32_t *)ring[i].data , CAN_DMA_MAX_PAYLOAD);
            ring[i].ready = 1U;
            ring[i].dma_active = 0U;
            break;
        }
    }
}

/*
   HAL_MDMA_XferErrorCallback()

   MDMA hata: MRAM'dan CPU ile kopyala (fallback).
   */

void HAL_MDMA_XferErrorCallback(MDMA_HandleTypeDef *hmdma)
{
    if (hmdma != &hmdma_mdma_channel0_sw_0) return;

    for (uint32_t i = 0; i < CAN_DMA_RING_SIZE; i++)
    {
        if (ring[i].dma_active && !ring[i].ready)
        {
            // Hata durumunda boş frame gönder – fallback yapamayız, sadece ready yap
            ring[i].ready = 1U;
            ring[i].dma_active = 0U;
            break;
        }
    }
}

/*
   can_dma_buffer_pop()
   DEĞİŞMEDİ — can_comm.c ile arayüz aynı.
    */
uint8_t can_dma_buffer_pop(can_dma_frame_t *out_frame)
{
    if (out_frame == NULL) return 0U;
    if (rd_idx == wr_idx) return 0U;
    if (!ring[rd_idx].ready) return 0U;

    uint32_t slot = rd_idx;

    // 1. ÖNCE VERİYİ GÜVENLE KOPYALA
    // (rd_idx henüz ilerlemediği için ISR bu slotun "dolu" olduğunu bilir ve buraya dokunmaz)
    out_frame->identifier   = ring[slot].identifier;
    out_frame->id_type      = ring[slot].id_type;
    out_frame->data_length  = ring[slot].data_length;
    out_frame->timestamp_ms = ring[slot].timestamp_ms;
    memcpy(out_frame->data, ring[slot].data, ring[slot].data_length);

    // 2. KOPYALAMA BİTTİKTEN SONRA İNDEKSİ İLERLET
    // (Artık ISR'a "bu slotu kullanabilirsin" izni veriyoruz)
    __disable_irq();
    rd_idx = (rd_idx + 1U) & RING_MASK;
    __enable_irq();

    return 1U;
}

/*
   Yardımcı sorgular — DEĞİŞMEDİ
    */
uint8_t can_dma_buffer_is_empty(void)
{
    return (rd_idx == wr_idx) ? 1U : 0U;
}

uint32_t can_dma_buffer_get_overrun_count(void)
{
    return overrun_cnt;
}

void can_dma_buffer_reset_stats(void)
{
    overrun_cnt = 0U;
}
