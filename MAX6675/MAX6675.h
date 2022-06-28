/*************************************************************************************
 Title	 :  MAXIM Integrated MAX6675 Library for STM32 Using HAL Libraries
 Author  :  Bardia Alikhan Afshar <bardia.a.afshar@gmail.com>
 Software:  STM32CubeIDE
 Hardware:  Any STM32 device
*************************************************************************************/
#ifndef INC_MAX6675_H_
#define INC_MAX6675_H_
#include "main.h"

// ------------------------- Defines -------------------------
#define SSPORT_1 GPIOB       // GPIO Port of Chip 1 Select(Slave Select)
#define SSPIN_1  GPIO_PIN_14  // GPIO PIN of Chip 1 Select(Slave Select)
#define SSPORT_2 GPIOA       // GPIO Port of Chip 2 Select(Slave Select)
#define SSPIN_2  GPIO_PIN_9  // GPIO PIN of Chip 2 Select(Slave Select)
// ------------------------- Functions  ----------------------
float Max6675_Read_Temp_1(void);
float Max6675_Read_Temp_2(void);
#endif
