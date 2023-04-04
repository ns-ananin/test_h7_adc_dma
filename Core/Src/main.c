/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Проект тестового задания для НТО ИРЭ-Полюс.
	* @author Ананин Николай
	* @date 03.04.2023
  ******************************************************************************
  * @attention
  *	Задание:
	* 
	* Необходимо реализовать чтение значений со входа АЦП с использованием DMA с дискретизацией 10 КГц и 
	* установки полученных усреднённых значений на выходе ЦАП раз в милисекунду.  А также выдаче усреднённых за 
	* 1 секунду значений по SWD интерфейсу в терминал IDE Keil. Полученные значения выводить в мили вольтах, 
	* формат представления в ASCII кодах с символом перевода строки на конце (пример: «1500 mV\r\n»).
	* Выбор ножек на ваше усмотрение.
	* Проект необходимо сгенерировать в STM32CubeMX для среды Keil с использованием библиотек «LL» для АЦП, 
	* для остальной периферии на «HAL».
	* Описание полученного кода сформировать в формате Doxygen.
	* По факту выполнения выложить в доступный GIT репозиторий.
  * 
  * Описание решения:
  * 
  * Для решения поставленной задачи выбрана макетная плата NUCLEO-H743ZI2 (микроконтроллер STM32H743ZI).
	* Используется следующая перифирия TIM4, ADC1, DMA, DAC. Тактовая частота ядра была настроена на 200МГц, 
	* тактовая частота шин TIM4, ADC1 100 МГц. Измерение ADC1 запускается по триггеру с таймера, который
	* устанавливается с частотой 10 кГц. Измерения ADC записываются в буфер через DMA. Буфер имеет двойной
	* размер, чтобы не останавливать измерение ADC пока выполняется обработка ранее принятых данных. Прерывание DMA срабатывает
	* при заполнении половины буфера и полного буфера. DMA настроен на циклический режим, чтобы измерение
	* производилось без остановки. Вычисление среднего значения напряжения и вывод в терминал реализовано
	* в основном потоке, чтобы разгрузить обработку прерывания.
	* 
	* Получившийся проект добавил в систему контроля версий GIT (исключил папку с obj файлами и файлы настройки отладчика) и выложил в публичный репозиторий GitHub:
	* https://github.com/ns-ananin/test_h7_adc_dma.git
	*
	* Проект STM32CubeMX находится в репозитории.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "EventRecorder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/// Размер буфера для сохранения данных с АЦП.
#define sizeBuffer 10

/// Буфер для сохранения данных с АЦП. Двойной размер буфера выбран, чтобы в одну часть
/// писать данные с АЦП, а другую часть обрабатывать.
static uint16_t buffer[sizeBuffer * 2]__attribute__((section(".ARM.__at_0x24000000")));

/// Переменная для накопления суммы отсчетов, чтобы рассчитать среднее значение напряжения за секунду.
static uint32_t sumPerSecond = 0;
/// Кол-во измерений для вычисления среднего значения за секунду.
static uint32_t numberMeasurements = 0;
/// Кол-во полученных измерений за 1 секунду.
#define numberMeasPerSecond 1000
/// Среднее кол-во отсчетов АЦП за секунду.
static volatile uint32_t averageMeasPerSecond = 0;
/// Флаг, который показывает, что готово новое среднее за секунду.
static volatile char isReadyAverageMeasPerSecond = 0;
/// Флаг, который показывает, что идет расчет напряжения и вывод его на экран.
static volatile char isCulcVoltage = 0;
/// Максимальное значение, которое может быть получено с АЦП.
#define maximumADCvalue 65535
/// Значение опорного напряжения АЦП, в мВ.
#define ADCreferenceVoltage 3300
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_DAC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * Обработать новые данные с АЦП.
 * @param isFullBuffer true - заполнена вторая половина буфера, false - заполнена первая половина буфера.
 */
void processADCvalues(char isFullBuffer){
	// Получаем указатели на новые данные. Таймер и АЦП настроены на работу с дискретность 0,1 мс.
  uint16_t *data = isFullBuffer ? &buffer[sizeBuffer] : buffer;
	// Находим среднее значение отсчетов за 1 мс.
	uint32_t sum = 0;
	for(size_t i = 0; i < sizeBuffer; i++){
		sum += data[i];
	}
	uint32_t averageValue = sum / sizeBuffer; // Среднее значение отсчетов за 1 мс.
	
	// Выводим полученное значени в ЦАП.
	uint32_t valueForDac = averageValue >> 4; // Переводим значение, так как ADC 16 разрядный, а DAC 12 разрядный.
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, valueForDac);
	
	// Расчет среднего значения за секунду.
	sumPerSecond += averageValue;
	numberMeasurements++;
	if(numberMeasurements >= numberMeasPerSecond && // Если накопили отсчеты за секунду.
		!isCulcVoltage){ // Для синхронизации потоков (защита, если не успели обработать в основном цикле прошлое значение). Чтобы избежать конфликта в использовании переменной averageMeasPerSecond.
		averageMeasPerSecond = sumPerSecond / numberMeasPerSecond;
		sumPerSecond = numberMeasurements = 0;
		isReadyAverageMeasPerSecond = 1;
		// Расчет напряжения и вывод в терминал выполнен в основном цикле, чтобы сократить время обработки прерывания.
	}
	
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	EventRecorderInitialize( EventRecordAll, 1 );
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0,(uint32_t)&ADC1->DR);// Настраиваем у DMA адрес исочника данных.
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)buffer);// Указываем DMA на приемник данных.
	LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, sizeBuffer * 2); // Указываем DMA размер приемника.
	LL_DMA_EnableIT_HT(DMA1, LL_DMA_STREAM_0); // Включаем прерывание по заполнению половины буфера.
	LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0); // Включаем прерывание по заполнению всего буфера. 
	LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0); // Запускаем DMA.
	LL_ADC_Enable(ADC1);												// Запускаем ADC.

	while(LL_ADC_IsActiveFlag_ADRDY(ADC1) != SET);
	LL_ADC_REG_StartConversion(ADC1);// Запускаем измерение.
	
	HAL_TIM_Base_Start(&htim4);// Запускаем таймер.
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(isReadyAverageMeasPerSecond){// Если появилось измеренное значение напряжения.
			isReadyAverageMeasPerSecond = 0;// Сбрасываем флаг, готовности нового измерения.
			// Расчитываем значение среднего напряжения за секунду.
			isCulcVoltage = 1; // Блокируем изменение переменной averageMeasPerSecond в прерывании пока производится копирование.
			// На данной архитектуре можно было обойтись без синхронизации, так как переменная 4 байта и выровнена в памяти, 
			// то операция копирования является атомарной. Решил подстраховаться и написать универсальное решение.
			uint32_t voltage = averageMeasPerSecond;
			isCulcVoltage = 0;
			voltage = (voltage * ADCreferenceVoltage) / maximumADCvalue;
			printf("%dmV\r\n", voltage);
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = 4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  LL_ADC_InitTypeDef ADC_InitStruct = {0};
  LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = {0};
  LL_ADC_CommonInitTypeDef ADC_CommonInitStruct = {0};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL2.PLL2M = 4;
  PeriphClkInitStruct.PLL2.PLL2N = 80;
  PeriphClkInitStruct.PLL2.PLL2P = 4;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOMEDIUM;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Peripheral clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_ADC12);

  LL_AHB4_GRP1_EnableClock(LL_AHB4_GRP1_PERIPH_GPIOB);
  /**ADC1 GPIO Configuration
  PB0   ------> ADC1_INP9
  */
  GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* ADC1 DMA Init */

  /* ADC1 Init */
  LL_DMA_SetPeriphRequest(DMA1, LL_DMA_STREAM_0, LL_DMAMUX1_REQ_ADC1);

  LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_STREAM_0, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);

  LL_DMA_SetStreamPriorityLevel(DMA1, LL_DMA_STREAM_0, LL_DMA_PRIORITY_LOW);

  LL_DMA_SetMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MODE_CIRCULAR);

  LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_PERIPH_NOINCREMENT);

  LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_STREAM_0, LL_DMA_MEMORY_INCREMENT);

  LL_DMA_SetPeriphSize(DMA1, LL_DMA_STREAM_0, LL_DMA_PDATAALIGN_HALFWORD);

  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_0, LL_DMA_MDATAALIGN_HALFWORD);

  LL_DMA_DisableFifoMode(DMA1, LL_DMA_STREAM_0);

  /* USER CODE BEGIN ADC1_Init 1 */
	ADC_REG_InitStruct.DataTransferMode = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
	ADC_InitStruct.LeftBitShift = LL_ADC_LEFT_BIT_SHIFT_NONE;
  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
  ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_16B;
  ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
  LL_ADC_Init(ADC1, &ADC_InitStruct);
  ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_EXT_TIM4_TRGO;
  ADC_REG_InitStruct.SequencerLength = LL_ADC_REG_SEQ_SCAN_DISABLE;
  ADC_REG_InitStruct.SequencerDiscont = DISABLE;
  ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
  ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
  LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
  ADC_CommonInitStruct.CommonClock = LL_ADC_CLOCK_ASYNC_DIV1;
  ADC_CommonInitStruct.Multimode = LL_ADC_MULTI_INDEPENDENT;
  LL_ADC_CommonInit(__LL_ADC_COMMON_INSTANCE(ADC1), &ADC_CommonInitStruct);
  LL_ADC_REG_SetTriggerEdge(ADC1, LL_ADC_REG_TRIG_EXT_RISING);

  /* Disable ADC deep power down (enabled by default after reset state) */
  LL_ADC_DisableDeepPowerDown(ADC1);
  /* Enable ADC internal voltage regulator */
  LL_ADC_EnableInternalRegulator(ADC1);
  /* Delay for ADC internal voltage regulator stabilization. */
  /* Compute number of CPU cycles to wait for, from delay in us. */
  /* Note: Variable divided by 2 to compensate partially */
  /* CPU processing cycles (depends on compilation optimization). */
  /* Note: If system core clock frequency is below 200kHz, wait time */
  /* is only a few CPU processing cycles. */
  __IO uint32_t wait_loop_index;
  wait_loop_index = ((LL_ADC_DELAY_INTERNAL_REGUL_STAB_US * (SystemCoreClock / (100000 * 2))) / 10);
  while(wait_loop_index != 0)
  {
    wait_loop_index--;
  }

  /** Configure Regular Channel
  */
  LL_ADC_REG_SetSequencerRanks(ADC1, LL_ADC_REG_RANK_1, LL_ADC_CHANNEL_9);
  LL_ADC_SetChannelSamplingTime(ADC1, LL_ADC_CHANNEL_9, LL_ADC_SAMPLINGTIME_16CYCLES_5);
  LL_ADC_SetChannelSingleDiff(ADC1, LL_ADC_CHANNEL_9, LL_ADC_SINGLE_ENDED);
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */
	
  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
	
  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* Init with LL driver */
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  NVIC_SetPriority(DMA1_Stream0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),0, 0));
  NVIC_EnableIRQ(DMA1_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
