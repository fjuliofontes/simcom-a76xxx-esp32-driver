/***********************************************************************************************************************
 *
 * @file    tasks_config.h
 * @brief   General Tasks Configurations.
 * @date    19/04/2022
 * @author  Fernando Fontes
 * @note    Platform Target: ESP32
 *
 * TAB SIZE: 4 SPACES
 *
***********************************************************************************************************************/

#ifndef TASKS_CONFIG_H_
#define TASKS_CONFIG_H_

/****************************************************** Includes ******************************************************/
// FreeRTOS includes.
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Application includes.
#include "freertos/FreeRTOSConfig.h"

/****************************************************** Defines *******************************************************/
#define COMPONENT_NAME_SIMCOM                           "SIMCOM_TASK"
#define TASK_STACK_SIZE_SIMCOM                          (4 * configMINIMAL_STACK_SIZE)
#define TASK_PRIORITY_SIMCOM                            (4 + tskIDLE_PRIORITY)
#define TASK_PERIOD_SIMCOM                              500


/****************************************************** Typedefs ******************************************************/

/***************************************************** Constants ******************************************************/

/***************************************************** Variables ******************************************************/

/***************************************************** Functions ******************************************************/

#endif