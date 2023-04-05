#ifndef Peripheral_API_H
#define Peripheral_API_H
/**
\file
\brief В данном файле содержится аппаратно-независимое API для работы с периферией микроконтроллера.
\author Ананин Николай
\version 1.0
\date 05.04.2023
*/

#include "stdint.h"

/**
Задать новое значение DAC.
@param newValue новое значение, которое необходимо передать в DAC, максимальное значение зависит от платформы.
*/
void PeripheralAPI_setValueInDAC(uint32_t newValue);

#endif