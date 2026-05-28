/* USER CODE BEGIN Header */
/* USER CODE END Header */

#include "main.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "i2c-lcd.h"
/* USER CODE END Includes */

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

float Kp = 1.5f;
float Ki = 0.5f;
float Kd = 0.05f;

volatile float target_rpm = 100.0f;
volatile float current_rpm = 0.0f;

volatile float error = 0.0f;
float prev_error = 0.0f;
float integral = 0.0f;
float derivative = 0.0f;
volatile float pwm_output = 0.0f;
volatile int32_t applied_pwm = 0;
volatile uint8_t motor_fault = 0;

int32_t encoder_count = 0;
int32_t prev_encoder_count = 0;

const float PPR = 330.0f;

const uint32_t PID_TICKS = 20;
const float CONTROL_DT = 0.02f;

const int32_t PWM_MAX = 999;
const int32_t PWM_MIN_RUN = 350;
const int32_t SETPOINT_STEP_RPM = 100;
const int32_t SETPOINT_MIN_RPM = 0;
const int32_t SETPOINT_MAX_RPM = 1000;
const float RPM_OK_TOLERANCE = 0.05f;
const float OVERSPEED_RATIO = 1.25f;

/* USER CODE END PV */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);

/* USER CODE BEGIN PFP */
void Motor_Set_PWM(int32_t pwm);
void Buttons_Update(void);
void Status_LED_Update(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void Motor_Set_PWM(int32_t pwm)
{
    int32_t command = pwm;

    if (pwm > PWM_MAX) pwm = PWM_MAX;
    if (pwm < -PWM_MAX) pwm = -PWM_MAX;

    if (pwm == 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
        applied_pwm = 0;
        pwm_output = (float)command;
        return;
    }

    if (pwm > 0 && pwm < PWM_MIN_RUN) pwm = PWM_MIN_RUN;
    if (pwm < 0 && pwm > -PWM_MIN_RUN) pwm = -PWM_MIN_RUN;

    if (pwm >= 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);
        applied_pwm = pwm;
    }
    else
    {
        pwm = -pwm;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, pwm);
        applied_pwm = -pwm;
    }

    pwm_output = (float)command;
}

void Buttons_Update(void)
{
    static GPIO_PinState prev_up = GPIO_PIN_SET;
    static GPIO_PinState prev_down = GPIO_PIN_SET;
    static uint32_t last_press_time = 0;

    GPIO_PinState up = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    GPIO_PinState down = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
    uint32_t now = HAL_GetTick();

    if ((now - last_press_time) < 180)
    {
        prev_up = up;
        prev_down = down;
        return;
    }

    if (prev_up == GPIO_PIN_SET && up == GPIO_PIN_RESET)
    {
        if (motor_fault)
        {
            motor_fault = 0;
            integral = 0.0f;
            prev_error = 0.0f;
        }
        target_rpm += SETPOINT_STEP_RPM;
        if (target_rpm > SETPOINT_MAX_RPM) target_rpm = SETPOINT_MAX_RPM;
        last_press_time = now;
    }

    if (prev_down == GPIO_PIN_SET && down == GPIO_PIN_RESET)
    {
        if (motor_fault)
        {
            motor_fault = 0;
            integral = 0.0f;
            prev_error = 0.0f;
        }
        target_rpm -= SETPOINT_STEP_RPM;
        if (target_rpm < SETPOINT_MIN_RPM) target_rpm = SETPOINT_MIN_RPM;
        last_press_time = now;
    }

    prev_up = up;
    prev_down = down;
}

void Status_LED_Update(void)
{
    static uint32_t stall_start_time = 0;
    uint32_t now = HAL_GetTick();

    uint8_t target_is_running = target_rpm > 0.0f;
    uint8_t speed_is_ok = target_is_running &&
                          (fabsf(error) <= (target_rpm * RPM_OK_TOLERANCE));
    uint8_t overspeed = target_is_running &&
                        (current_rpm > (target_rpm * OVERSPEED_RATIO));
    uint8_t motor_commanded = target_is_running && (abs(applied_pwm) > PWM_MIN_RUN);
    uint8_t motor_not_moving = current_rpm < 10.0f;
    uint8_t stall = 0;

    if (motor_commanded && motor_not_moving)
    {
        if (stall_start_time == 0)
        {
            stall_start_time = now;
        }
        stall = (now - stall_start_time) > 2000;
        if (stall)
        {
            motor_fault = 1;
            Motor_Set_PWM(0);
        }
    }
    else
    {
        stall_start_time = 0;
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (speed_is_ok && !motor_fault) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    if (overspeed || stall || motor_fault)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, ((now / 250) % 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    }
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();

  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  Motor_Set_PWM(0);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

  __HAL_TIM_SET_COUNTER(&htim2, 0);
  prev_encoder_count = 0;

  lcd_init();
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("PID Motor Init");
  lcd_put_cur(1, 0);
  lcd_send_string("Target 100 RPM");
  HAL_Delay(1500);
  lcd_clear();

  HAL_TIM_Base_Start_IT(&htim3);

  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE BEGIN WHILE */

    char buffer[17];

    Buttons_Update();
    Status_LED_Update();

    lcd_put_cur(0, 0);
    snprintf(buffer, sizeof(buffer), "RPM:%4d S:%3d  ", (int)current_rpm, (int)target_rpm);
    lcd_send_string(buffer);

    lcd_put_cur(1, 0);
    if (motor_fault)
    {
        snprintf(buffer, sizeof(buffer), "FAULT STALL     ");
    }
    else
    {
        snprintf(buffer, sizeof(buffer), "PWM:%4d E:%4d ", (int)applied_pwm, (int)error);
    }
    lcd_send_string(buffer);

    HAL_Delay(200);

    /* USER CODE END WHILE */
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;

  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 95;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM2_Init(void)
{
  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;

  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;

  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;

  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 95;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 95;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;

  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim4);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9 | GPIO_PIN_10, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_15, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3)
    {
        static uint32_t pid_tick_count = 0;

        pid_tick_count++;
        if (pid_tick_count < PID_TICKS)
        {
            return;
        }
        pid_tick_count = 0;

        encoder_count = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);

        int32_t count_diff = encoder_count - prev_encoder_count;
        prev_encoder_count = encoder_count;

        float measured_rpm = ((float)count_diff * 60.0f) / (PPR * CONTROL_DT);
        current_rpm = current_rpm * 0.75f + measured_rpm * 0.25f;

        error = target_rpm - current_rpm;

        if (motor_fault)
        {
            integral = 0.0f;
            derivative = 0.0f;
            pwm_output = 0.0f;
            prev_error = error;
            Motor_Set_PWM(0);
            return;
        }

        integral += error * CONTROL_DT;

        if (integral > 1000.0f) integral = 1000.0f;
        if (integral < -1000.0f) integral = -1000.0f;

        derivative = (error - prev_error) / CONTROL_DT;

        pwm_output = Kp * error + Ki * integral + Kd * derivative;

        prev_error = error;

        Motor_Set_PWM((int32_t)pwm_output);
    }
}

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
