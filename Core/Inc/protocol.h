// protocol.h
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include <string.h>

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#define PROTO_STATIC_ASSERT(cond, msg) _Static_assert((cond), msg)
#else
#define PROTO_STATIC_ASSERT(cond, msg)
#endif

#define CANFD_CMD_DATA_LEN      20
#define CANFD_RPT_DATA_LEN      24
#define CANFD_BMS_RPT_DATA_LEN  24
#define CANFD_STATUS_LEN         1

typedef struct {
    float  hizlan;
    float  don;
    float  taret_pan;
    float  taret_tilt;
    int8_t surus_modu;
    int8_t frenle;
    int8_t kontrol;
} command_t;

typedef struct {
    float  mevcut_hiz;
    float  mevcut_yon;
    float  mevcut_egim;
    float  mevcut_pan;
    float  mevcut_tilt;
    int8_t hata;
    int8_t kontrol;
} report_t;


typedef struct {
    float   avg_voltage;
    float   min_voltage;
    float   max_voltage;
    float   current_a;
    int8_t  max_temp;
    int8_t  min_temp;
    uint8_t cells;
    uint8_t fault;
} bms_report_t;

typedef struct {
    uint8_t safety_state;
} status_t;

void    proto_pack_command(const command_t *cmd, uint8_t out[CANFD_CMD_DATA_LEN]);
void    proto_pack_report(const report_t *rpt, uint8_t out[CANFD_RPT_DATA_LEN]);
void    proto_pack_bms_report(const bms_report_t *bms, uint8_t out[CANFD_BMS_RPT_DATA_LEN]);
int     proto_unpack_command(const uint8_t in[CANFD_CMD_DATA_LEN], command_t *cmd);
int     proto_unpack_report(const uint8_t in[CANFD_RPT_DATA_LEN], report_t *rpt);
uint8_t proto_crc8(const uint8_t *data, uint16_t len);


PROTO_STATIC_ASSERT(sizeof(int8_t)  == 1, "int8 must be 1 byte");
PROTO_STATIC_ASSERT(sizeof(uint8_t) == 1, "uint8 must be 1 byte");
PROTO_STATIC_ASSERT(CANFD_CMD_DATA_LEN     == (4u*4u + 3u*1u + 1u),      "cmd len mismatch");
PROTO_STATIC_ASSERT(CANFD_RPT_DATA_LEN     == (5u*4u + 2u + 1u + 1u),    "rpt len mismatch");
PROTO_STATIC_ASSERT(CANFD_BMS_RPT_DATA_LEN == (4u*4u + 4u*1u + 3u + 1u), "bms len mismatch");

#endif
