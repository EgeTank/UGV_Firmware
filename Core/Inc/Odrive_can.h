#ifndef ODRIVE_CAN_H
#define ODRIVE_CAN_H

#include <stdint.h>
#include <string.h>


#define ODRIVE_NODE_ID          0U

/* -------------------------------------------------------
   ODrive Komut ID'leri (Cmd_ID) — ODrive Pro dokümanından
   ------------------------------------------------------- */
#define ODRIVE_CMD_HEARTBEAT        0x001U  /* Axis durumu ve hata kodu         */
#define ODRIVE_CMD_GET_ENCODER      0x009U  /* Pozisyon + Hız (turns, turns/s)  */
#define ODRIVE_CMD_SET_INPUT_POS    0x00CU  /* Pozisyon komutu                  */
#define ODRIVE_CMD_SET_INPUT_VEL    0x00DU  /* Hız komutu (turns/s + tork FF)   */
#define ODRIVE_CMD_SET_INPUT_TORQUE 0x00EU  /* Tork komutu (Nm)                 */
#define ODRIVE_CMD_GET_IQ           0x014U  /* Motor akım ölçümü (A)            */
#define ODRIVE_CMD_GET_BUS_VI       0x017U  /* Bus voltaj ve akım               */
#define ODRIVE_CMD_SET_CTRL_MODE    0x00BU  /* Kontrol modu seçimi              */

/* -------------------------------------------------------
   Kontrol Modları (Set_Controller_Mode için)
   ------------------------------------------------------- */
#define ODRIVE_CTRL_MODE_VOLTAGE    0U
#define ODRIVE_CTRL_MODE_TORQUE     1U
#define ODRIVE_CTRL_MODE_VELOCITY   2U
#define ODRIVE_CTRL_MODE_POSITION   3U

/* -------------------------------------------------------
   Input Modları
   ------------------------------------------------------- */
#define ODRIVE_INPUT_MODE_PASSTHROUGH   1U
#define ODRIVE_INPUT_MODE_VEL_RAMP      2U
#define ODRIVE_INPUT_MODE_TRAP_TRAJ     5U

/* -------------------------------------------------------
   Axis State'leri (Heartbeat'ten gelir)
   ------------------------------------------------------- */
#define ODRIVE_AXIS_STATE_IDLE          1U
#define ODRIVE_AXIS_STATE_CLOSED_LOOP   8U

/* -------------------------------------------------------
   CAN ID Hesaplama Makrosu
   ------------------------------------------------------- */
#define ODRIVE_CAN_ID(node_id, cmd_id)  (((node_id) << 5) | (cmd_id))

/* -------------------------------------------------------
   ODrive mesajı mı? Verilen CAN ID, bu node'a ait mi?
   ------------------------------------------------------- */
#define ODRIVE_IS_MY_MSG(can_id) \
    (((can_id) >> 5) == ODRIVE_NODE_ID)

#define ODRIVE_GET_CMD_ID(can_id) \
    ((can_id) & 0x1FU)

/* ============================================================
   GELEN VERİ STRUCT'LARI (ODrive → STM32)
   ============================================================ */

/* Heartbeat (0x001) — 8 byte
   ODrive'ın genel sağlık durumu */
typedef struct {
    uint32_t axis_error;        /* Hata kodu (0 = hata yok)       */
    uint8_t  axis_state;        /* Mevcut durum (IDLE, CLOSED_LOOP vb.) */
    uint8_t  procedure_result;  /* Son prosedür sonucu            */
    uint8_t  trajectory_done;   /* Yörünge tamamlandı mı?         */
} odrive_heartbeat_t;

/* Encoder Estimates (0x009) — 8 byte
   Motorun anlık pozisyon ve hız bilgisi */
typedef struct {
    float pos_estimate;   /* Pozisyon (turns)   */
    float vel_estimate;   /* Hız     (turns/s)  */
} odrive_encoder_t;

/* Get_Iq (0x014) — 8 byte
   Motor akım bilgisi */
typedef struct {
    float iq_setpoint;    /* Hedef akım (A)   */
    float iq_measured;    /* Ölçülen akım (A) */
} odrive_iq_t;

/* Get_Bus_Voltage_Current (0x017) — 8 byte */
typedef struct {
    float bus_voltage;    /* Bus voltajı (V) */
    float bus_current;    /* Bus akımı   (A) */
} odrive_bus_vi_t;


typedef struct {
    float  input_vel;          /* Hedef hız          (turns/s) */
    float  input_torque_ff;    /* Tork feedforward   (Nm)      */
} odrive_set_vel_t;

/* Set_Input_Torque (0x00E) — 4 byte */
typedef struct {
    float input_torque;        /* Hedef tork (Nm) */
} odrive_set_torque_t;

/* Set_Input_Pos (0x00C) — 8 byte */
typedef struct {
    float   input_pos;         /* Hedef pozisyon (turns)         */
    int16_t vel_ff;            /* Hız feedforward (0.001 turns/s) */
    int16_t torque_ff;         /* Tork feedforward (0.001 Nm)     */
} odrive_set_pos_t;

/* Set_Controller_Mode (0x01C) — 8 byte */
typedef struct {
    uint32_t control_mode;     /* ODRIVE_CTRL_MODE_* */
    uint32_t input_mode;       /* ODRIVE_INPUT_MODE_* */
} odrive_ctrl_mode_t;


typedef struct {
    odrive_heartbeat_t  heartbeat;
    odrive_encoder_t    encoder;
    odrive_iq_t         iq;
    odrive_bus_vi_t     bus_vi;
    uint8_t             is_connected;   /* 0: bağlı değil, 1: bağlı */
    uint32_t            last_heartbeat_tick;
} odrive_state_t;


/* Başlatma */
void odrive_init(void);

/* Gelen CAN mesajını işle — can_process() içinden çağrılır */
void odrive_process_rx(uint16_t can_id, const uint8_t *data, uint8_t len);

/* Komut gönderme */
int odrive_send_velocity(float vel_turns_per_s, float torque_ff_nm);
int odrive_send_torque(float torque_nm);
int odrive_send_position(float pos_turns, float vel_ff, float torque_ff);
int odrive_set_control_mode(uint32_t ctrl_mode, uint32_t input_mode);

/* Birim dönüşüm yardımcıları */
float odrive_mps_to_turns_per_s(float speed_mps);
float odrive_turns_per_s_to_mps(float turns_per_s);

/* Durum okuma */
const odrive_state_t* odrive_get_state(void);
uint8_t odrive_is_connected(void);
uint8_t odrive_has_error(void);

#endif /* ODRIVE_CAN_H */
