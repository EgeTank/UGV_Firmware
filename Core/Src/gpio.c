/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  * of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "system_config.h" /* E-STOP pin tanımlamaları buradan geliyor */
/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Pinout Configuration
*/
void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* 1. Tüm İlgili Portların Saatlerini Aktifleştir */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE(); /* SİLAH VE LAZER İÇİN EKLENDİ */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE(); /* FREN VE KONTAKTÖR İÇİN EKLENDİ */

  /* 2. Çıkış Pinlerinin Başlangıç Durumlarını Güvenli Seviyeye (RESET) Çek */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5 | GPIO_PIN_8, GPIO_PIN_RESET);

  /* 3. GPIOC Konfigürasyonu (PC13: Lazer, PC14: Ateşleme Rölesi) */
  GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* 4. GPIOB Çıkış Konfigürasyonu (PB5: Fren, PB8: EV200 Kontaktör) */
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 5. GPIOB Giriş Konfigürasyonu (PB6: Fren Butonu) */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP; /* Butonun donanımsal bağlantısına göre PULLDOWN da yapabilirsin */
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* 6. GPIOA Konfigürasyonu (PA1: BMS FAULT) */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 7. GPIOA Kesme Konfigürasyonu (PA0: E-STOP) */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* 8. E-STOP Kesmesini NVIC Üzerinde Aktifleştir */
  /* Öncelik 5: FreeRTOS API'lerinin kullanılabileceği en yüksek güvenli öncelik */
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* 9. Atış Tetiği (Örn: GPIOC_PIN_15) Kesme Konfigürasyonu */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; /* Tetiğe basıldığında tetiklenir */
  GPIO_InitStruct.Pull = GPIO_PULLUP; /* Donanımında pull-down varsa PULLDOWN yapmalısın */
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); /* HANGİ PORTA BAĞLIYSA ONU YAZMALISIN (Örn: GPIOC) */

    /* 10. Tetik Kesmesini NVIC Üzerinde Aktifleştir */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);


}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
