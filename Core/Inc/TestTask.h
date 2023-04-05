/**
\file
\brief Данный файл содержит описание API тестового задания.
\author Ананин Николай
\version 1.0
\date 05.04.2023
\defgroup test_task Тестовое задание

Функционал модуля:
 - находит среднее значение по переданным измерениям и дублирует его в DAC.
 - по последним переданным значениям вычисляет среднее значение напряжения на АЦП и выводит его в поток вывода
 
 Модуль настраивается #define в данном файле.
 */
///@{
#ifndef TEST_TASK_H
#define TEST_TASK_H

#include "stdint.h"
#include "stddef.h"

/// Кол-во вызовов TestTask_processADCvalues после которого выводится среднее значение в терминад.
#define nuberCall_ProcessADCvalues_ForCulcVoltage 1000

/// Кол-во разрядов АЦП.
#define TestTask_bitADC 16

/// Кол-во разрядов ЦАП.
#define TestTask_bitDAC 12

/// Значение опорного напряжения АЦП, в мВ.
#define TestTask_ADCreferenceVoltage 3300

/**
\brief Обработать новые данные с АЦП. Функция может быть вызвана из прерывания.

В данную функцию передаются новые данные для обработки. Функция может быть вызвана из прерывания.

@param data данные на новые данные с АЦП.
@param sizeData кол-во данных которые следует обработать.
 */
void TestTask_processADCvalues(uint16_t *data, size_t sizeData);

/**
Привести модуль в начальное состояние.
*/
void TestTask_init();

/**
Данная функция должна вызываться в основном цикле программы.
*/
void TestTask_execute();

/**
Получить среднее значение напряжение на входе АЦП за 10000 отсчетов АЦП.
@return среднее значение напряжение на входе АЦП за 10000 отсчетов АЦП, мВ.
*/
uint32_t TestTask_getVoltage();

/**
Флаг, который показывает, что значение напряжения вычислено и его можно использовать.
@return true - значение напряжение измерено, false - значение напряжения ещё не измерено.
*/
char TestTask_isReadyVoltage();

///@}
#endif