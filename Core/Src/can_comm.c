#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"

/* YENİ: FreeRTOS ve Mutex Kütüphaneleri eklendi */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
/* ---------------------------------------------- */

#include "can_comm.h"
#include "protocol.h"
#include "system_config.h"
#include "motor_control.h"
#include "failsafe.h"
#include "state_machine.h"
#include "odrive_can.h"
#include "can_dma_buffer.h"
#include "heartbeat.h"
#include "bms_emus_can.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern volatile uint32_t last_command_rx_time;
extern heartbeat_t hb;
extern BMS_Data_t bms_data;
extern osMutexId_t canTxMutexHandle;

static uint32_t can_tx_error_count    = 0;
/* ControlTask içerisinden (freertos.c) erişilebilmesi için static kaldırıldı */
uint32_t can_rx_error_count           = 0;
uint32_t can_protocol_error_count     = 0;

uint32_t get_fdcan_dlc(uint8_t len)
{
    if (len <= 8)  return FDCAN_DLC_BYTES_8;
    if (len <= 12) return FDCAN_DLC_BYTES_12;
    if (len <= 16) return FDCAN_DLC_BYTES_16;
    if (len <= 20) return FDCAN_DLC_BYTES_20;
    if (len <= 24) return FDCAN_DLC_BYTES_24;
    if (len <= 32) return FDCAN_DLC_BYTES_32;
    if (len <= 48) return FDCAN_DLC_BYTES_48;
    if (len <= 64) return FDCAN_DLC_BYTES_64;
    return FDCAN_DLC_BYTES_64;
}

int can_send_Nbyte(uint16_t std_id, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier          = std_id;
    TxHeader.IdType              = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    TxHeader.DataLength          = get_fdcan_dlc(len);
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_ON;
    TxHeader.FDFormat             = FDCAN_FD_CAN;
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0;

    int ret = 0;

    /* FDCAN Donanımını Race Condition'dan koruyan Mutex */
    if (osMutexAcquire(canTxMutexHandle, pdMS_TO_TICKS(5)) == osOK)
    {
        HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(
                &hfdcan1, &TxHeader, (uint8_t *)data);

        if (status != HAL_OK)
        {
            can_tx_error_count++;
        }
        else
        {
            ret = 1;
        }
        osMutexRelease(canTxMutexHandle);
    }
    else
    {
        /* Mutex (hat/yol) 5ms içinde alınamazsa mesajı yollama (çakışmayı önle), hata sayacını artır */
        can_tx_error_count++;
    }

    return ret;
}

int can_send_report(const report_t *rpt)
{
    uint8_t buffer[CANFD_RPT_DATA_LEN];
    proto_pack_report(rpt, buffer);
    return can_send_Nbyte(CAN_ID_DRIVE_RPT, buffer, CANFD_RPT_DATA_LEN);
}

int can_send_bms_report(const bms_report_t *bms)
{
    uint8_t buffer[CANFD_BMS_RPT_DATA_LEN];
    proto_pack_bms_report(bms, buffer);
    return can_send_Nbyte(CAN_ID_BMS_RPT, buffer, CANFD_BMS_RPT_DATA_LEN);
}

void can_init(void)
{
    HAL_FDCAN_Start(&hfdcan1);

    /* Filtre 0: ROS tarafı — Heartbeat (0x0FF) + Drive Command (0x100) */
    FDCAN_FilterTypeDef sFilterRos;
    sFilterRos.IdType       = FDCAN_STANDARD_ID;
    sFilterRos.FilterIndex  = 0;
    sFilterRos.FilterType   = FDCAN_FILTER_RANGE;
    sFilterRos.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterRos.FilterID1    = HEARTBEAT_CAN_ID;   /* 0x0FF */
    sFilterRos.FilterID2    = CAN_ID_DRIVE_CMD;
    /* 0x100 */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterRos);

    /* Filtre 1: MCU tarafı — ODrive mesajları (0x001 - 0x01F) */
    FDCAN_FilterTypeDef sFilterOdrive;
    sFilterOdrive.IdType       = FDCAN_STANDARD_ID;
    sFilterOdrive.FilterIndex  = 1;
    sFilterOdrive.FilterType   = FDCAN_FILTER_RANGE;
    sFilterOdrive.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterOdrive.FilterID1    = 0x001U;
    sFilterOdrive.FilterID2    = 0x01FU;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterOdrive);

    /* Filtre 2: MCU tarafı — BMS mesajları (0x300 - 0x36D) */
    FDCAN_FilterTypeDef sFilterBms;
    sFilterBms.IdType       = FDCAN_STANDARD_ID;
    sFilterBms.FilterIndex  = 2;
    sFilterBms.FilterType   = FDCAN_FILTER_RANGE;
    sFilterBms.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterBms.FilterID1    = BMS_BASE_ID;              /* 0x300 */
    sFilterBms.FilterID2    = BMS_BASE_ID + 109U;
    /* 0x36D */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterBms);

    HAL_FDCAN_ActivateNotification(
        &hfdcan1,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
        0);
    can_dma_buffer_init();
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                                uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
    {
        can_dma_buffer_push_from_isr(hfdcan);
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL) != 0U)
    {
        can_rx_error_count++;
    }
}

uint32_t can_get_tx_error_count(void)      { return can_tx_error_count;      }
uint32_t can_get_rx_error_count(void)      { return can_rx_error_count;      }
uint32_t can_get_protocol_error_count(void){ return can_protocol_error_count; }
