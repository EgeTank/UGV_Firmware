#ifndef BMS_EMUS_CAN_H
#define BMS_EMUS_CAN_H

#include <stdint.h>

#define BMS_BASE_ID             0x300U
#define BMS_OFFSET_CURRENT      5U
#define BMS_OFFSET_VOLT_START   20U
#define BMS_OFFSET_VOLT_END     29U
#define BMS_OFFSET_TEMP_START   100U
#define BMS_OFFSET_TEMP_END     109U
#define BMS_MAX_CELLS           14U

typedef struct {
    float   current_A;
    float   cell_voltages[BMS_MAX_CELLS];
    float   cell_temps[BMS_MAX_CELLS];
    uint8_t cells_received;
} BMS_Data_t;

void  BMS_Init(BMS_Data_t *bms);
void  BMS_ParseCanMessage(uint32_t can_id, const uint8_t *data, uint8_t dlc, BMS_Data_t *bms);
float BMS_GetAvgCellVoltage(const BMS_Data_t *bms);
float BMS_GetMinCellVoltage(const BMS_Data_t *bms);
float BMS_GetMaxCellVoltage(const BMS_Data_t *bms);
float BMS_GetMaxCellTemp(const BMS_Data_t *bms);
float BMS_GetMinCellTemp(const BMS_Data_t *bms);

#endif /* BMS_EMUS_CAN_H */
