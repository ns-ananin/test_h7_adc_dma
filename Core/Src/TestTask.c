/**
\file
\brief Данный файл содержит реализацию тестового задания.
*/

#include "TestTask.h"
#include "PeripheralAPI.h"
#include "stdio.h"

/// Переменная для накопления суммы отсчетов, чтобы рассчитать среднее значение напряжения за секунду.
static uint32_t sumPerSecond = 0;
/// Кол-во измерений для вычисления среднего значения за секунду.
static uint32_t numberMeasurements = 0;
/// Среднее кол-во отсчетов АЦП за секунду.
static volatile uint32_t averageMeasPerSecond = 0;
/// Флаг, который показывает, что готово новое среднее за секунду.
static volatile char isReadyAverageMeasPerSecond = 0;
/// Флаг, который показывает, что идет расчет напряжения и вывод его на экран.
static volatile char isCulcVoltage = 0;
/// Флаг, true - идет инициализация данного модуля. false - инициализация модуля не идет. Данный флаг
/// позволяет переинициализировать модуль не останавливая измерение.
static char isInit = 0;
/// Измеренное и вычисленное значение напряжения на входе АЦП, мВ.
static uint32_t currentVoltage = 0;

/*
\brief Обработать новые данные с АЦП. Функция может быть вызвана из прерывания.

\details В данную функцию передаются новые данные для обработки. Функция может быть вызвана из прерывания.
@param data данные на новые данные с АЦП.
@param sizeData кол-во данных которые следует обработать.

 */
void TestTask_processADCvalues(uint16_t *data, size_t sizeData){
	if(!isInit){// Если не происходит инициализация в данный момент.
		// Находим среднее значение отсчетов за 1 мс.
		uint32_t sum = 0;
		for(size_t i = 0; i < sizeData; i++){
			sum += data[i];
		}
		uint32_t averageValue = sum / sizeData; // Среднее значение отсчетов за 1 мс.
		
		// Выводим полученное значени в ЦАП.
		uint32_t valueForDac =  (averageValue << TestTask_bitDAC) >> TestTask_bitADC ; // Переводим значение, так как ADC 16 разрядный, а DAC 12 разрядный.
		PeripheralAPI_setValueInDAC(valueForDac);
		
		// Расчет среднего значения за секунду.
		sumPerSecond += averageValue;
		numberMeasurements++;
		if(numberMeasurements >= nuberCall_ProcessADCvalues_ForCulcVoltage && // Если накопили отсчеты за секунду.
			!isCulcVoltage){ // Для синхронизации потоков (защита, если не успели обработать в основном цикле прошлое значение). Чтобы избежать конфликта в использовании переменной averageMeasPerSecond.
			averageMeasPerSecond = sumPerSecond / nuberCall_ProcessADCvalues_ForCulcVoltage;
			sumPerSecond = numberMeasurements = 0;
			isReadyAverageMeasPerSecond = 1;
			// Расчет напряжения и вывод в терминал выполнен в основном цикле, чтобы сократить время обработки прерывания.
		}
	}
}

/**
\brief Привести модуль в начальное состояние. Останавливать работу АЦП не требуется.

*/
void TestTask_init(){
	isInit = 1;// Устанавливаем флаг, что идет инициализация. Останавливаем обработку приходящих данных.
	sumPerSecond = 0;
	numberMeasurements = 0;
	averageMeasPerSecond = 0;
	isReadyAverageMeasPerSecond = 0;
	isCulcVoltage = 0;
	currentVoltage = 0;
	isInit = 0;// Разрешаем обработку приходящих данных.
}

/**
\brief Данная функция должна вызываться в основном цикле программы.

*/
void TestTask_execute(){
			if(isReadyAverageMeasPerSecond){// Если появилось измеренное значение напряжения.
			isReadyAverageMeasPerSecond = 0;// Сбрасываем флаг, готовности нового измерения.
			// Расчитываем значение среднего напряжения за секунду.
			isCulcVoltage = 1; // Блокируем изменение переменной averageMeasPerSecond в прерывании пока производится копирование.
			// На данной архитектуре можно было обойтись без синхронизации, так как переменная 4 байта и выровнена в памяти, 
			// то операция копирования является атомарной. Решил подстраховаться и написать универсальное решение.
			uint32_t voltage = averageMeasPerSecond;
			isCulcVoltage = 0;
			voltage = (voltage * TestTask_ADCreferenceVoltage) / ((1 << TestTask_bitADC) - 1);
			currentVoltage = voltage;
			printf("%dmV\r\n", currentVoltage);
		}
}

/**
\brief Получить среднее значение напряжение на входе АЦП.
@return среднее значение напряжение на входе АЦП, мВ.

*/
uint32_t TestTask_getVoltage(){
	return currentVoltage;
}