#include "odrive_can.h"
#include "can_comm.h"
#include "unit_conversion.h"
#include "stm32h7xx_hal.h"
#include <string.h>

/* ============================================================
   ANLIK DURUM CACHE
   ============================================================ */
static odrive_state_t odrive_state;

/* ============================================================
   LITTLE-ENDIAN YARDIMCI FONKSİYONLAR
   (protocol.c ile aynı mantık — ODrive de little-endian kullanır)
   ============================================================ */
static float read_f32_le(const uint8_t *in)
{
    uint32_t u = ((uint32_t)in[0] <<  0) |
                 ((uint32_t)in[1] <<  8) |
                 ((uint32_t)in[2] << 16) |
                 ((uint32_t)in[3] << 24);
    float v = 0.0f;
    memcpy(&v, &u, sizeof(v));
    return v;
}

static uint32_t read_u32_le(const uint8_t *in)
{
    return ((uint32_t)in[0] <<  0) |
           ((uint32_t)in[1] <<  8) |
           ((uint32_t)in[2] << 16) |
           ((uint32_t)in[3] << 24);
}

/**
 * static int16_t read_i16_le(const uint8_t *in)
{
    return (int16_t)(((uint16_t)in[0]) | ((uint16_t)in[1] << 8));
}
*/ //ilerde unpack için gerekbilir diye ekledim.
static void write_f32_le(uint8_t *out, float v)
{
    uint32_t u = 0;
    memcpy(&u, &v, sizeof(u));
    out[0] = (uint8_t)((u >>  0) & 0xFFu);
    out[1] = (uint8_t)((u >>  8) & 0xFFu);
    out[2] = (uint8_t)((u >> 16) & 0xFFu);
    out[3] = (uint8_t)((u >> 24) & 0xFFu);
}

static void write_u32_le(uint8_t *out, uint32_t v)
{
    out[0] = (uint8_t)((v >>  0) & 0xFFu);
    out[1] = (uint8_t)((v >>  8) & 0xFFu);
    out[2] = (uint8_t)((v >> 16) & 0xFFu);
    out[3] = (uint8_t)((v >> 24) & 0xFFu);
}

static void write_i16_le(uint8_t *out, int16_t v)
{
    out[0] = (uint8_t)((v >> 0) & 0xFF);
    out[1] = (uint8_t)((v >> 8) & 0xFF);
}

/* ============================================================
   UNPACK FONKSİYONLARI — ODrive → STM32
   Her biri ilgili mesajı byte dizisinden struct'a çevirir.
   ============================================================ */

/* Heartbeat (0x001) — 8 byte
   [0-3]: axis_error (u32)
   [4]  : axis_state (u8)
   [5]  : procedure_result (u8)
   [6]  : trajectory_done_bit (u8) */
static void unpack_heartbeat(const uint8_t *data, odrive_heartbeat_t *out)
{
    out->axis_error       = read_u32_le(&data[0]);
    out->axis_state       = data[4];
    out->procedure_result = data[5];
    out->trajectory_done  = data[6] & 0x01u;
}

/* Encoder Estimates (0x009) — 8 byte
   [0-3]: pos_estimate (f32, turns)
   [4-7]: vel_estimate (f32, turns/s) */
static void unpack_encoder(const uint8_t *data, odrive_encoder_t *out)
{
    out->pos_estimate = read_f32_le(&data[0]);
    out->vel_estimate = read_f32_le(&data[4]);
}

/* Get_Iq (0x014) — 8 byte
   [0-3]: iq_setpoint (f32, A)
   [4-7]: iq_measured (f32, A) */
static void unpack_iq(const uint8_t *data, odrive_iq_t *out)
{
    out->iq_setpoint = read_f32_le(&data[0]);
    out->iq_measured = read_f32_le(&data[4]);
}

/* Get_Bus_Voltage_Current (0x017) — 8 byte
   [0-3]: bus_voltage (f32, V)
   [4-7]: bus_current (f32, A) */
static void unpack_bus_vi(const uint8_t *data, odrive_bus_vi_t *out)
{
    out->bus_voltage = read_f32_le(&data[0]);
    out->bus_current = read_f32_le(&data[4]);
}

/* ============================================================
   PACK FONKSİYONLARI — STM32 → ODrive
   Her biri struct'ı byte dizisine çevirir.
   ============================================================ */

/* Set_Input_Vel (0x00D) — 8 byte
   [0-3]: input_vel      (f32, turns/s)
   [4-7]: input_torque_ff (f32, Nm) */
static void pack_set_vel(const odrive_set_vel_t *cmd, uint8_t *out)
{
    write_f32_le(&out[0], cmd->input_vel);
    write_f32_le(&out[4], cmd->input_torque_ff);
}

/* Set_Input_Torque (0x00E) — 4 byte
   [0-3]: input_torque (f32, Nm) */
static void pack_set_torque(const odrive_set_torque_t *cmd, uint8_t *out)
{
    write_f32_le(&out[0], cmd->input_torque);
}

/* Set_Input_Pos (0x00C) — 8 byte
   [0-3]: input_pos  (f32, turns)
   [4-5]: vel_ff     (i16, 0.001 turns/s)
   [6-7]: torque_ff  (i16, 0.001 Nm) */
static void pack_set_pos(const odrive_set_pos_t *cmd, uint8_t *out)
{
    write_f32_le(&out[0], cmd->input_pos);
    write_i16_le(&out[4], cmd->vel_ff);
    write_i16_le(&out[6], cmd->torque_ff);
}

/* Set_Controller_Mode (0x00BU) — 8 byte
   [0-3]: control_mode (u32)
   [4-7]: input_mode   (u32) */
static void pack_ctrl_mode(const odrive_ctrl_mode_t *cmd, uint8_t *out)
{
    write_u32_le(&out[0], cmd->control_mode);
    write_u32_le(&out[4], cmd->input_mode);
}

/* ============================================================
   INIT
   ============================================================ */
void odrive_init(void)
{
    memset(&odrive_state, 0, sizeof(odrive_state));
    odrive_state.is_connected = 0;
}

/* ============================================================
   GELEN MESAJ İŞLEYİCİ
   can_process() içinden çağrılır:
     odrive_process_rx(rxHeader.Identifier, rxData, rxHeader.DataLength);
   ============================================================ */
void odrive_process_rx(uint16_t can_id, const uint8_t *data, uint8_t len)
{
    /* Bu mesaj ODrive'a ait mi? */
    if (!ODRIVE_IS_MY_MSG(can_id)) return;

    uint8_t cmd_id = ODRIVE_GET_CMD_ID(can_id);

    switch (cmd_id)
    {
        case ODRIVE_CMD_HEARTBEAT:
            if (len >= 7)
            {
                unpack_heartbeat(data, &odrive_state.heartbeat);
                odrive_state.is_connected        = 1;
                odrive_state.last_heartbeat_tick = HAL_GetTick();
            }
            break;

        case ODRIVE_CMD_GET_ENCODER:
            if (len >= 8)
            {
                unpack_encoder(data, &odrive_state.encoder);
            }
            break;

        case ODRIVE_CMD_GET_IQ:
            if (len >= 8)
            {
                unpack_iq(data, &odrive_state.iq);
            }
            break;

        case ODRIVE_CMD_GET_BUS_VI:
            if (len >= 8)
            {
                unpack_bus_vi(data, &odrive_state.bus_vi);
            }
            break;

        default:
            /* Tanımlanmamış ODrive mesajı — yoksay */
            break;
    }
}

/* ============================================================
   KOMUT GÖNDERME FONKSİYONLARI
   Hepsi can_send_Nbyte() kullanır — mevcut CAN katmanına dokunmaz.
   ============================================================ */

/* Hız komutu — ana sürüş modu
   vel_turns_per_s : odrive_mps_to_turns_per_s() ile dönüştürülmüş değer
   torque_ff_nm    : eğim için tork feedforward (0.0f = yok) */
int odrive_send_velocity(float vel_turns_per_s, float torque_ff_nm)
{
    odrive_set_vel_t cmd;
    cmd.input_vel       = vel_turns_per_s;
    cmd.input_torque_ff = torque_ff_nm;

    uint8_t buf[8];
    pack_set_vel(&cmd, buf);

    return can_send_Nbyte(
        ODRIVE_CAN_ID(ODRIVE_NODE_ID, ODRIVE_CMD_SET_INPUT_VEL),
        buf, 8);
}

/* Tork komutu */
int odrive_send_torque(float torque_nm)
{
    odrive_set_torque_t cmd;
    cmd.input_torque = torque_nm;

    uint8_t buf[4];
    pack_set_torque(&cmd, buf);

    return can_send_Nbyte(
        ODRIVE_CAN_ID(ODRIVE_NODE_ID, ODRIVE_CMD_SET_INPUT_TORQUE),
        buf, 4);
}

/* Pozisyon komutu */
int odrive_send_position(float pos_turns, float vel_ff, float torque_ff)
{
    odrive_set_pos_t cmd;
    cmd.input_pos  = pos_turns;
    cmd.vel_ff     = (int16_t)(vel_ff    * 1000.0f);
    cmd.torque_ff  = (int16_t)(torque_ff * 1000.0f);

    uint8_t buf[8];
    pack_set_pos(&cmd, buf);

    return can_send_Nbyte(
        ODRIVE_CAN_ID(ODRIVE_NODE_ID, ODRIVE_CMD_SET_INPUT_POS),
        buf, 8);
}

/* Kontrol modu değiştirme
   Örnek: odrive_set_control_mode(ODRIVE_CTRL_MODE_VELOCITY,
                                   ODRIVE_INPUT_MODE_VEL_RAMP); */
int odrive_set_control_mode(uint32_t ctrl_mode, uint32_t input_mode)
{
    odrive_ctrl_mode_t cmd;
    cmd.control_mode = ctrl_mode;
    cmd.input_mode   = input_mode;

    uint8_t buf[8];
    pack_ctrl_mode(&cmd, buf);

    return can_send_Nbyte(
        ODRIVE_CAN_ID(ODRIVE_NODE_ID, ODRIVE_CMD_SET_CTRL_MODE),
        buf, 8);
}

float odrive_mps_to_turns_per_s(float speed_mps)
{
    const float WHEEL_CIRC_M = WHEEL_DIAMETER_M * 3.14159265f;
    float wheel_turns_s = speed_mps / WHEEL_CIRC_M;
    return wheel_turns_s * GEAR_REDUCTION_RATIO;
}

float odrive_turns_per_s_to_mps(float turns_per_s)
{
    const float WHEEL_CIRC_M = WHEEL_DIAMETER_M * 3.14159265f;
    float wheel_turns_s = turns_per_s / GEAR_REDUCTION_RATIO;
    return wheel_turns_s * WHEEL_CIRC_M;
}

/* ============================================================
   DURUM SORGULAMA
   ============================================================ */
const odrive_state_t* odrive_get_state(void)
{
    return &odrive_state;
}

uint8_t odrive_is_connected(void)
{
    /* 1000ms içinde heartbeat gelmemişse bağlantı kopmuş say */
    if (!odrive_state.is_connected) return 0;
    return ((HAL_GetTick() - odrive_state.last_heartbeat_tick) < 1000U) ? 1u : 0u;
}

uint8_t odrive_has_error(void)
{
    return (odrive_state.heartbeat.axis_error != 0u) ? 1u : 0u;
}
