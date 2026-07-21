#include "protocol.h"
#include <string.h>

static void proto_write_u32_le(uint8_t *out, uint32_t v)
{
    out[0] = (uint8_t)((v >>  0) & 0xFFu);
    out[1] = (uint8_t)((v >>  8) & 0xFFu);
    out[2] = (uint8_t)((v >> 16) & 0xFFu);
    out[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static uint32_t proto_read_u32_le(const uint8_t *in)
{
    return ((uint32_t)in[0] <<  0) | ((uint32_t)in[1] <<  8) |
           ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
}

static void proto_write_f32_le(uint8_t *out, float v)
{
    uint32_t u = 0;
    memcpy(&u, &v, sizeof(u));
    proto_write_u32_le(out, u);
}

static float proto_read_f32_le(const uint8_t *in)
{
    uint32_t u = proto_read_u32_le(in);
    float v = 0.0f;
    memcpy(&v, &u, sizeof(v));
    return v;
}

uint8_t proto_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? ((crc << 1) ^ 0x07) : (crc << 1);
    }
    return crc;
}

void proto_pack_command(const command_t *cmd, uint8_t out[CANFD_CMD_DATA_LEN])
{
    uint16_t idx = 0;
    proto_write_f32_le(&out[idx], cmd->hizlan);    idx += 4;
    proto_write_f32_le(&out[idx], cmd->don);        idx += 4;
    proto_write_f32_le(&out[idx], cmd->taret_pan);  idx += 4;
    proto_write_f32_le(&out[idx], cmd->taret_tilt); idx += 4;
    out[idx++] = (uint8_t)cmd->surus_modu;
    out[idx++] = (uint8_t)cmd->frenle;
    out[idx++] = (uint8_t)cmd->kontrol;
    out[CANFD_CMD_DATA_LEN - 1] = proto_crc8(out, CANFD_CMD_DATA_LEN - 1);
}

int proto_unpack_command(const uint8_t in[CANFD_CMD_DATA_LEN], command_t *cmd)
{
    if (proto_crc8(in, CANFD_CMD_DATA_LEN - 1) != in[CANFD_CMD_DATA_LEN - 1])
        return 0;
    uint16_t idx = 0;
    cmd->hizlan     = proto_read_f32_le(&in[idx]); idx += 4;
    cmd->don        = proto_read_f32_le(&in[idx]); idx += 4;
    cmd->taret_pan  = proto_read_f32_le(&in[idx]); idx += 4;
    cmd->taret_tilt = proto_read_f32_le(&in[idx]); idx += 4;
    cmd->surus_modu = (int8_t)in[idx++];
    cmd->frenle     = (int8_t)in[idx++];
    cmd->kontrol    = (int8_t)in[idx++];
    return 1;
}

void proto_pack_report(const report_t *rpt, uint8_t out[CANFD_RPT_DATA_LEN])
{
    uint16_t idx = 0;
    proto_write_f32_le(&out[idx], rpt->mevcut_hiz);  idx += 4;
    proto_write_f32_le(&out[idx], rpt->mevcut_yon);  idx += 4;
    proto_write_f32_le(&out[idx], rpt->mevcut_egim); idx += 4;
    proto_write_f32_le(&out[idx], rpt->mevcut_pan);  idx += 4;
    proto_write_f32_le(&out[idx], rpt->mevcut_tilt); idx += 4;
    out[idx++] = (uint8_t)rpt->hata;
    out[idx++] = (uint8_t)rpt->kontrol;
    out[idx++] = 0x00;
    out[CANFD_RPT_DATA_LEN - 1] = proto_crc8(out, CANFD_RPT_DATA_LEN - 1);
}

int proto_unpack_report(const uint8_t in[CANFD_RPT_DATA_LEN], report_t *rpt)
{
    if (proto_crc8(in, CANFD_RPT_DATA_LEN - 1) != in[CANFD_RPT_DATA_LEN - 1])
        return 0;
    uint16_t idx = 0;
    rpt->mevcut_hiz  = proto_read_f32_le(&in[idx]); idx += 4;
    rpt->mevcut_yon  = proto_read_f32_le(&in[idx]); idx += 4;
    rpt->mevcut_egim = proto_read_f32_le(&in[idx]); idx += 4;
    rpt->mevcut_pan  = proto_read_f32_le(&in[idx]); idx += 4;
    rpt->mevcut_tilt = proto_read_f32_le(&in[idx]); idx += 4;
    rpt->hata    = (int8_t)in[idx++];
    rpt->kontrol = (int8_t)in[idx++];
    return 1;
}

void proto_pack_bms_report(const bms_report_t *bms,
                            uint8_t out[CANFD_BMS_RPT_DATA_LEN])
{
    uint16_t idx = 0;
    proto_write_f32_le(&out[idx], bms->avg_voltage); idx += 4;
    proto_write_f32_le(&out[idx], bms->min_voltage); idx += 4;
    proto_write_f32_le(&out[idx], bms->max_voltage); idx += 4;
    proto_write_f32_le(&out[idx], bms->current_a);   idx += 4;
    out[idx++] = (uint8_t)bms->max_temp;
    out[idx++] = (uint8_t)bms->min_temp;
    out[idx++] = bms->cells;
    out[idx++] = bms->fault;
    out[idx++] = 0x00;
    out[idx++] = 0x00;
    out[idx++] = 0x00;
    out[CANFD_BMS_RPT_DATA_LEN - 1] = proto_crc8(out, CANFD_BMS_RPT_DATA_LEN - 1);
}
