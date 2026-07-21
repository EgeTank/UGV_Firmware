#include "bms_emus_can.h"
#include <string.h>

void BMS_Init(BMS_Data_t *bms)
{
    memset(bms, 0, sizeof(BMS_Data_t));
    for (uint8_t i = 0; i < BMS_MAX_CELLS; i++)
    {
        bms->cell_voltages[i] = 3.6f;
        bms->cell_temps[i]    = 25.0f;
    }
    bms->current_A      = 0.0f;
    bms->cells_received = 0;
}

void BMS_ParseCanMessage(uint32_t can_id, const uint8_t *data, uint8_t dlc, BMS_Data_t *bms)
{
    if (can_id < BMS_BASE_ID) return;
    uint32_t offset = can_id - BMS_BASE_ID;

    if (offset == BMS_OFFSET_CURRENT && dlc >= 2)
    {
        int16_t raw_current = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
        bms->current_A = raw_current * 0.1f;
    }
    else if (offset >= BMS_OFFSET_VOLT_START && offset <= BMS_OFFSET_VOLT_END)
    {
        uint8_t group = (uint8_t)(offset - BMS_OFFSET_VOLT_START);
        for (uint8_t i = 0; i < dlc; i++)
        {
            uint8_t cell_index = (group * 8u) + i;
            if (cell_index < BMS_MAX_CELLS)
            {
                bms->cell_voltages[cell_index] = (data[i] * 0.01f) + 2.00f;
                if (cell_index >= bms->cells_received)
                    bms->cells_received = cell_index + 1u;
            }
        }
    }
    else if (offset >= BMS_OFFSET_TEMP_START && offset <= BMS_OFFSET_TEMP_END)
    {
        uint8_t group = (uint8_t)(offset - BMS_OFFSET_TEMP_START);
        for (uint8_t i = 0; i < dlc; i++)
        {
            uint8_t cell_index = (group * 8u) + i;
            if (cell_index < BMS_MAX_CELLS)
                bms->cell_temps[cell_index] = (float)data[i] - 100.0f;
        }
    }
}

float BMS_GetAvgCellVoltage(const BMS_Data_t *bms)
{
    if (bms->cells_received == 0) return 3.6f;
    float sum = 0.0f;
    for (uint8_t i = 0; i < bms->cells_received; i++)
        sum += bms->cell_voltages[i];
    return sum / (float)bms->cells_received;
}

float BMS_GetMinCellVoltage(const BMS_Data_t *bms)
{
    if (bms->cells_received == 0) return 3.6f;
    float min_v = bms->cell_voltages[0];
    for (uint8_t i = 1; i < bms->cells_received; i++)
        if (bms->cell_voltages[i] < min_v) min_v = bms->cell_voltages[i];
    return min_v;
}

float BMS_GetMaxCellVoltage(const BMS_Data_t *bms)
{
    if (bms->cells_received == 0) return 3.6f;
    float max_v = bms->cell_voltages[0];
    for (uint8_t i = 1; i < bms->cells_received; i++)
        if (bms->cell_voltages[i] > max_v) max_v = bms->cell_voltages[i];
    return max_v;
}

float BMS_GetMaxCellTemp(const BMS_Data_t *bms)
{
    float max_temp = -100.0f;
    for (uint8_t i = 0; i < BMS_MAX_CELLS; i++)
        if (bms->cell_temps[i] > max_temp) max_temp = bms->cell_temps[i];
    return max_temp;
}

float BMS_GetMinCellTemp(const BMS_Data_t *bms)
{
    float min_temp = 100.0f;
    for (uint8_t i = 0; i < BMS_MAX_CELLS; i++)
        if (bms->cell_temps[i] < min_temp) min_temp = bms->cell_temps[i];
    return min_temp;
}
