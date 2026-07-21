/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "system_config.h"
#include "bms_emus_can.h"
#include "can_comm.h"
#include "can_dma_buffer.h"
#include "heartbeat.h"
#include "vel_profile.h"
#include "state_machine.h"
#include "motor_control.h"
#include "brake_control.h"
#include "current_limiter.h"
#include "safety.h"
#include "failsafe.h"
#include "protocol.h"
#include "error_codes.h"
#include "pid.h"
#include "odrive_sm.h"
#include "iwdg.h"
#include "imu_filter.h"
#include "pid_tuner.h"
#include "weapon_control.h"
#include "drive_mode.h"
#include "safe_mode.h"
#include "blackbox.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_PERIOD_MS       10U
#define SAFETY_PERIOD_MS        50U
#define TELEMETRY_PERIOD_MS     100U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Hata takibi için can_comm.c'den alınan global değişkenler */
extern uint32_t can_rx_error_count;
extern uint32_t can_protocol_error_count;
extern FDCAN_HandleTypeDef hfdcan1;

/* BMS verisi — bmsMutex */
BMS_Data_t      g_bmsData;
/* Araç durum makinesi — stateMutex */
vehicle_state_t g_vehicleState;
/* ROS heartbeat izleyici — stateMutex */
heartbeat_t     g_heartbeat;
/* ROS'tan gelen hedef hız (mm/s) ve sürüş parametreleri — targetSpeedMutex */
float           g_targetSpeed_mm_s;
uint8_t         g_surusModu;
uint8_t         g_kontrol;
/* Hata maskesi — stateMutex */
uint8_t         g_errorMask;
uint8_t         g_errorDetail; //Detay hataları için bit çakışması olmaması içindir.

/* SADECE ControlTask'a ait — mutex gerekmez */
static vel_profile_t     s_velProfile;
static PID_Controller_t  s_pid;
static current_limiter_t s_currentLimiter;
static odrive_sm_t       s_odriveSm;
volatile float g_imu_angle_deg = 0.0f;
static imu_filter_t      s_imuFilter;

/* --- SİMÜLASYON VE TUNING EKLENTİLERİ --- */
#ifdef DEBUG_SIMULATION
static imu_sim_state_t   s_imuSim;
static pid_tuner_t       s_pidTuner;
static uint8_t           s_pid_step_started = 0u;
#endif

/* CubeMX'in SİLEMEYECEĞİ IMU Mutex Tanımlaması */
osMutexId_t imuMutexHandle;
const osMutexAttr_t imuMutex_attributes = {
  .name = "imuMutex",
  .attr_bits = osMutexRecursive,
};
/* ----------------------- */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for SafetyTask */
osThreadId_t SafetyTaskHandle;
const osThreadAttr_t SafetyTask_attributes = {
  .name = "SafetyTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canTxMutex */
osMutexId_t canTxMutexHandle;
const osMutexAttr_t canTxMutex_attributes = {
  .name = "canTxMutex",
  .attr_bits = osMutexRecursive,
};
/* Definitions for bmsMutex */
osMutexId_t bmsMutexHandle;
const osMutexAttr_t bmsMutex_attributes = {
  .name = "bmsMutex",
  .attr_bits = osMutexRecursive,
};
/* Definitions for stateMutex */
osMutexId_t stateMutexHandle;
const osMutexAttr_t stateMutex_attributes = {
  .name = "stateMutex",
  .attr_bits = osMutexRecursive,
};
/* Definitions for targetSpeedMutex */
osMutexId_t targetSpeedMutexHandle;
const osMutexAttr_t targetSpeedMutex_attributes = {
  .name = "targetSpeedMutex",
  .attr_bits = osMutexRecursive,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartControlTask(void *argument);
void StartSafetyTask(void *argument);
void StartTelemetryTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  BMS_Init(&g_bmsData);
  heartbeat_init(&g_heartbeat);
  g_targetSpeed_mm_s = 0.0f;
  g_surusModu        = 0u;
  g_kontrol          = 0u;
  g_errorMask        = 0u;
  /* USER CODE END Init */

  /* Create the recursive mutex(es) */
  /* creation of canTxMutex */
  canTxMutexHandle = osMutexNew(&canTxMutex_attributes);

  /* creation of bmsMutex */
  bmsMutexHandle = osMutexNew(&bmsMutex_attributes);

  /* creation of stateMutex */
  stateMutexHandle = osMutexNew(&stateMutex_attributes);

  /* creation of targetSpeedMutex */
  targetSpeedMutexHandle = osMutexNew(&targetSpeedMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  imuMutexHandle = osMutexNew(&imuMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ControlTask */
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);

  /* creation of SafetyTask */
  SafetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &SafetyTask_attributes);

  /* creation of TelemetryTask */
  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  for (;;) { osDelay(1000); }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControlTask */
/**
  * @brief ControlTask — 100 Hz ana kontrol dongusu.
  */
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
  /* USER CODE BEGIN StartControlTask */
  motor_init();
  brake_init();
  //TIM3 kullandığımızı varsaydığım için eskiden encoder_start ile TIM3 donanımı başlatılıyordu.
  failsafe_init();
  state_machine_init();
  weapon_init(); /* Taret silah sistemi ve röleleri başlat */

  vel_profile_init(&s_velProfile,
                   VEL_PROFILE_MAX_ACCEL_MM_S2,
                   VEL_PROFILE_MAX_DECEL_MM_S2,
                   VEL_PROFILE_MAX_JERK_MM_S3,
                   VEL_PROFILE_SCURVE);

  PID_Init(&s_pid, PID_KP, PID_KI, PID_KD, PID_OUT_MIN, PID_OUT_MAX);
  PID_SetHoldOffset(PID_HOLD_OFFSET);

  current_limiter_init(&s_currentLimiter,
                       CURRENT_LIM_NOMINAL_A,
                       CURRENT_LIM_PEAK_A,
                       CURRENT_LIM_MIN_A);

  odrive_sm_init(&s_odriveSm);

  imu_filter_init(&s_imuFilter, IMU_FILTER_ALPHA);

#ifdef DEBUG_SIMULATION
  imu_sim_init(&s_imuSim, IMU_SIM_NOISE_SCALE, 1u);
  pid_tuner_init(&s_pidTuner, PID_KP, PID_KI, PID_KD);
  pid_tuner_apply(&s_pidTuner, &s_pid);
  s_pid_step_started = 0u;
#endif

  const float DT_S = (float)CONTROL_PERIOD_MS / 1000.0f;
  TickType_t xLastWake = xTaskGetTickCount();

  for (;;)
  {
    /* 1) CAN RX ring buffer'i bosalt */
    {
      can_dma_frame_t frame;
      while (can_dma_buffer_pop(&frame))
      {
        if (frame.identifier >= BMS_BASE_ID &&
            frame.identifier <= (BMS_BASE_ID + BMS_OFFSET_TEMP_END))
        {
          osMutexAcquire(bmsMutexHandle, osWaitForever);
          BMS_ParseCanMessage(frame.identifier, frame.data, frame.data_length, &g_bmsData);
          osMutexRelease(bmsMutexHandle);
        }
        else if (frame.identifier == HEARTBEAT_CAN_ID)
        {
          osMutexAcquire(stateMutexHandle, osWaitForever);
          heartbeat_kick(&g_heartbeat, frame.data);
          osMutexRelease(stateMutexHandle);
        }
        else if (frame.identifier == 0x100U)
        {
          command_t cmd;
          if (proto_unpack_command(frame.data, &cmd) != 0)
          {
            osMutexAcquire(targetSpeedMutexHandle, osWaitForever);
            g_targetSpeed_mm_s = cmd.hizlan * 1000.0f;
            g_surusModu        = cmd.surus_modu;
            g_kontrol          = cmd.kontrol;
            osMutexRelease(targetSpeedMutexHandle);
            failsafe_kick();
          }
          else
          {
            can_protocol_error_count++;
          }
        }
        else if (ODRIVE_IS_MY_MSG(frame.identifier))
        {
          /* 1. Gelen mesajı (Encoder, IQ, Heartbeat, vb.) önbelleğe al */
          odrive_process_rx(frame.identifier, frame.data, frame.data_length);
        }
      }

      if (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) >= 32)
      {
          can_rx_error_count++;
      }

      if (can_dma_buffer_get_overrun_count() > 0U)
      {
          can_rx_error_count += can_dma_buffer_get_overrun_count();
          can_dma_buffer_reset_stats();
      }
    }

    /* 2) Arac durumunu oku */
    vehicle_state_t state = state_machine_get();

    /* --- ODrive durum makinesi her döngüde (10 ms) güncellenir --- */
    odrive_sm_update(&s_odriveSm, state);

    /* 3) Failsafe ve Heartbeat timeout kontrolü (Şartlardan bağımsız her döngüde çalışır) */
    if(osMutexAcquire(stateMutexHandle , pdMS_TO_TICKS(2)) == osOK)
    {
    	heartbeat_check(&g_heartbeat);
    	osMutexRelease(stateMutexHandle);

    }
    failsafe_check();


    if (state == STATE_DRIVING && !failsafe_is_active())
    {
      float target_mm_s;
      uint8_t current_surus_modu;
      uint8_t current_kontrol;

      osMutexAcquire(targetSpeedMutexHandle, osWaitForever);
      target_mm_s        = g_targetSpeed_mm_s;
      current_surus_modu = g_surusModu;
      current_kontrol    = g_kontrol;
      osMutexRelease(targetSpeedMutexHandle);

      /* --- MOD, LİMP VE SİLAH KONTROLLERİ (GÜNCELLENDİ) --- */
      osMutexAcquire(stateMutexHandle, osWaitForever);
      uint8_t anlik_hata   = g_errorMask;
      uint8_t anlik_detay  = g_errorDetail;   // YENİ: detay hataları da oku
      osMutexRelease(stateMutexHandle);

      // safe_mode_check artık 3 parametre alıyor
      uint8_t safe_status = safe_mode_check(anlik_hata, anlik_detay, &s_velProfile, &s_currentLimiter);

      /* Kritik değilse sürüş modunu uygula (Limp mode'u ezmemesi için) */
      if (safe_status == 2) {
          drive_mode_update(current_surus_modu, &s_velProfile, &s_currentLimiter);
      }

      weapon_task_check();
      weapon_laser_set((current_kontrol & 0x01) ? 1 : 0);
      /* -------------------------------------- */

      float smoothed_mm_s = vel_profile_step(&s_velProfile, target_mm_s, DT_S);

      /* TIM3 Harici Enkoder yerine ODrive'ın kendi CAN FD hız verisini kullanıyoruz */
      const odrive_state_t *odrv_feedback = odrive_get_state();
      float measured_mm_s = odrive_turns_per_s_to_mps(odrv_feedback->encoder.vel_estimate) * 1000.0f;

      float busVoltage;
      osMutexAcquire(bmsMutexHandle, osWaitForever);
      busVoltage        = BMS_GetAvgCellVoltage(&g_bmsData) * (float)g_bmsData.cells_received;
      float motorTemp   = BMS_GetMaxCellTemp(&g_bmsData);
      float measuredCur = g_bmsData.current_A;
      osMutexRelease(bmsMutexHandle);

      current_limiter_update(&s_currentLimiter, busVoltage, motorTemp,
                             measuredCur, (float)CONTROL_PERIOD_MS);

      float ff_rad = g_imu_angle_deg * 0.01745329f;
      float ff_sin = ff_rad - (ff_rad * ff_rad * ff_rad) / 6.0f;
      if (ff_sin >  1.0f) ff_sin =  1.0f;
      if (ff_sin < -1.0f) ff_sin = -1.0f;
      float gravity_ff = GRAVITY_FEEDFORWARD * ff_sin;
      float pid_raw = PID_Compute_WithFeedForward(&s_pid, smoothed_mm_s,
                                                          measured_mm_s, DT_S,
                                                          gravity_ff);


      float pid_out = current_limiter_apply(&s_currentLimiter, pid_raw);

#ifdef DEBUG_SIMULATION
      {
          if (!s_pid_step_started)
          {
              pid_tuner_start_step(&s_pidTuner, 500.0f, HAL_GetTick());
              s_pid_step_started = 1u;
          }
          pid_tuner_step(&s_pidTuner, measured_mm_s, HAL_GetTick());
      }
#endif /* DEBUG_SIMULATION */

      {
          float gyro_dps        = 0.0f;
          float accel_angle_deg = 0.0f;

#ifdef DEBUG_SIMULATION
          imu_sim_step(&s_imuSim, DT_S, &gyro_dps, &accel_angle_deg);
#endif
          float angle = imu_filter_update(&s_imuFilter, gyro_dps, accel_angle_deg, DT_S);

          /* IMU açısını Mutex ile koruyarak yaz */
          if (osMutexAcquire(imuMutexHandle, osWaitForever) == osOK) {
              g_imu_angle_deg = angle;
              osMutexRelease(imuMutexHandle);
          }
      }

      brake_release();
      motor_set_target_speed(smoothed_mm_s);
      motor_set_output(pid_out);
    }
    else
    {
      vel_profile_reset(&s_velProfile);
      PID_Reset(&s_pid);
      brake_apply();
      motor_apply_fail_safe(1u);

      if (failsafe_is_hard_locked())
        odrive_sm_force_idle(&s_odriveSm);

#ifdef DEBUG_SIMULATION
      s_pid_step_started = 0u;
#endif
      imu_filter_reset(&s_imuFilter, 0.0f);
    }

    HAL_IWDG_Refresh(&hiwdg1);
    vTaskDelayUntil(&xLastWake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
  }
  //Odrive her loop ta çağırılıyor artık heartbeat mesajlarına bağımlı olmadan sürekli ve hızlı güncelliyor.
  /* USER CODE END StartControlTask */
}

/* USER CODE BEGIN Header_StartSafetyTask */
/**
* @brief Function implementing the SafetyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSafetyTask */
void StartSafetyTask(void *argument)
{
  /* USER CODE BEGIN StartSafetyTask */
  Safety_Init();

  for (;;)
  {
    safety_status_t gpioStatus = Safety_Check();

    float minCellV, maxCellT;
    uint8_t bmsReceivedOk;
    osMutexAcquire(bmsMutexHandle, osWaitForever);
    minCellV      = BMS_GetMinCellVoltage(&g_bmsData);
    maxCellT      = BMS_GetMaxCellTemp(&g_bmsData);
    bmsReceivedOk = (g_bmsData.cells_received > 0u) ? 1u : 0u;
    osMutexRelease(bmsMutexHandle);

    uint8_t rosAlive = 0;
        /* Timeout ekleyerek SafetyTask'ın donmasını engelliyoruz */
        if (osMutexAcquire(stateMutexHandle, pdMS_TO_TICKS(5)) == osOK) {
            rosAlive = heartbeat_is_alive(&g_heartbeat);
            osMutexRelease(stateMutexHandle);
        }

    /* --- DEĞİŞEN KISIM: Ana hatalar ve detay hatalar ayrıldı --- */
    // ... (önceki kodlar)
    uint8_t mainErrors = 0u;
    uint8_t detailErrors = 0u;   // zaten var

    if (gpioStatus == SAFETY_ESTOP_ACTIVE)            ERR_SET(mainErrors, ERR_ESTOP_ACTIVE);
    if (gpioStatus == SAFETY_BMS_FAULT)               ERR_SET(mainErrors, ERR_BMS_FAULT);

    if (!rosAlive)                                    ERR_SET(detailErrors, ERR_DETAIL_HEARTBEAT_LOST);
    if (bmsReceivedOk && minCellV < BMS_MIN_CELL_VOLTAGE_V) ERR_SET(detailErrors, ERR_DETAIL_LOW_VOLTAGE);
    if (bmsReceivedOk && maxCellT > BMS_MAX_CELL_TEMP_C)    ERR_SET(detailErrors, ERR_DETAIL_HIGH_TEMP);

    osMutexAcquire(stateMutexHandle, osWaitForever);
    g_errorMask = mainErrors;
    g_errorDetail = detailErrors;   // YENİ satır

    // Durum makinesi geçişleri için ana hataları kullan (kritik hala aynı)
    if (ERR_IS_SET(mainErrors, ERR_ESTOP_ACTIVE) || ERR_IS_SET(mainErrors, ERR_BMS_FAULT))
    {
        state_machine_set(STATE_EMERGENCY);
        ERR_SET(g_errorMask, ERR_HARD_LATCH);
    }
    else if ( (detailErrors & (ERR_DETAIL_LOW_VOLTAGE | ERR_DETAIL_HIGH_TEMP | ERR_DETAIL_HEARTBEAT_LOST)) != 0u )
    {
        // Detay hatalardan biri varsa STANDBY'e geç, ama hard latch değil
        state_machine_set(STATE_STANDBY);
        ERR_SET(g_errorMask, ERR_FAILSAFE_ACTIVE);
    }
    else if (state_machine_get() != STATE_EMERGENCY)
    {
        state_machine_set(STATE_DRIVING);
    }

    osMutexRelease(stateMutexHandle);

    osDelay(pdMS_TO_TICKS(SAFETY_PERIOD_MS));
  }
  /* USER CODE END StartSafetyTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the TelemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */

  // DÜZELTME: blackbox_init() kaldırılırken silinen kritik adaptif telemetri değişkeni geri eklendi
  static uint8_t last_error_mask = 0u;

  for (;;)
  {
    /* 1) BMS Raporu */
    {
      bms_report_t bmsRpt = {0};
      osMutexAcquire(bmsMutexHandle, osWaitForever);
      bmsRpt.avg_voltage = BMS_GetAvgCellVoltage(&g_bmsData);
      bmsRpt.min_voltage = BMS_GetMinCellVoltage(&g_bmsData);
      bmsRpt.max_voltage = BMS_GetMaxCellVoltage(&g_bmsData);
      bmsRpt.current_a   = g_bmsData.current_A;
      bmsRpt.max_temp    = (int8_t)BMS_GetMaxCellTemp(&g_bmsData);
      bmsRpt.min_temp    = (int8_t)BMS_GetMinCellTemp(&g_bmsData);
      bmsRpt.cells       = g_bmsData.cells_received;
      bmsRpt.fault       = (BMS_GetMinCellVoltage(&g_bmsData) < BMS_MIN_CELL_VOLTAGE_V ||
                             BMS_GetMaxCellTemp(&g_bmsData)   > BMS_MAX_CELL_TEMP_C) ? 1u : 0u;
      osMutexRelease(bmsMutexHandle);

      if (osMutexAcquire(canTxMutexHandle, pdMS_TO_TICKS(5)) == osOK)
      {
        can_send_bms_report(&bmsRpt);
        osMutexRelease(canTxMutexHandle);
      }
    }

    /* 2) Hareket Telemetrisi */
        uint8_t current_errors = 0;
        vehicle_state_t current_state = state_machine_get();

        {
          report_t rpt = {0};

          /* --- DÜZELTME: Hedef hız yerine ÖLÇÜLEN hızı (enkoder) al --- */
          const odrive_state_t *odrv = odrive_get_state();
          if (odrv != NULL) {
              // ODrive enkoder hızını mm/s birimine çevirip yazıyoruz
              rpt.mevcut_hiz = odrive_turns_per_s_to_mps(odrv->encoder.vel_estimate) * 1000.0f;
          } else {
              rpt.mevcut_hiz = 0.0f;
          }
          /* ------------------------------------------------------------- */

          /* IMU açısını Mutex ile koruyarak oku */
          if (osMutexAcquire(imuMutexHandle, pdMS_TO_TICKS(5)) == osOK) {
              rpt.mevcut_egim = g_imu_angle_deg;
              osMutexRelease(imuMutexHandle);
          } else {
              rpt.mevcut_egim = 0.0f;
          }

          osMutexAcquire(stateMutexHandle, osWaitForever);
          current_errors = g_errorMask;
          rpt.hata    = (int8_t)g_errorMask;
          rpt.kontrol = (int8_t)current_state;
          osMutexRelease(stateMutexHandle);

          if (osMutexAcquire(canTxMutexHandle, pdMS_TO_TICKS(5)) == osOK)
          {
            can_send_report(&rpt);
            osMutexRelease(canTxMutexHandle);
          }
        }

    /* --- SİYAH KUTU SÜREKLİ LOGLAMA (10 Hz) --- */
    {
        blackbox_record_t r = {0};
        r.crash_timestamp_ms  = HAL_GetTick();
        r.vehicle_state       = current_state;
        r.error_mask          = current_errors;

        osMutexAcquire(targetSpeedMutexHandle, pdMS_TO_TICKS(5));
        r.target_speed_mm_s   = g_targetSpeed_mm_s;
        osMutexRelease(targetSpeedMutexHandle);

        const odrive_state_t *odrv = odrive_get_state();
        if (odrv != NULL) {
            r.measured_speed_mm_s = odrv->encoder.vel_estimate * 1000.0f;
            r.odrive_axis_error   = odrv->heartbeat.axis_error;
            r.odrive_axis_state   = odrv->heartbeat.axis_state;
        }

        osMutexAcquire(imuMutexHandle, pdMS_TO_TICKS(5));
        r.imu_angle_deg       = g_imu_angle_deg;
        osMutexRelease(imuMutexHandle);

        osMutexAcquire(bmsMutexHandle, pdMS_TO_TICKS(5));
        r.bus_voltage_v       = BMS_GetAvgCellVoltage(&g_bmsData) * (float)g_bmsData.cells_received;
        r.current_a           = g_bmsData.current_A;
        r.max_cell_temp_c     = (int8_t)BMS_GetMaxCellTemp(&g_bmsData);
        osMutexRelease(bmsMutexHandle);

        blackbox_log_step(&r);
    }

    /* --- ADAPTİF TELEMETRİ (DİNAMİK GECİKME) --- */
    uint32_t delay_ms = TELEMETRY_PERIOD_MS; /* Standart 100ms (10 Hz) */

    if (current_state == STATE_STANDBY && current_errors == 0u)
    {
        delay_ms = 500u;
    }

    if (current_errors != last_error_mask)
    {
        delay_ms = 10u;
    }

    last_error_mask = current_errors;

    osDelay(pdMS_TO_TICKS(delay_ms));
  }
  /* USER CODE END StartTelemetryTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

