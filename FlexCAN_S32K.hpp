/*
 * FlexCAN_S32K.hpp
 *
 *  Created on: 16/08/2019
 *      Author: noxuz
 */

#ifndef FLEXCAN_S32K_HPP_
#define FLEXCAN_S32K_HPP_

#include "device_registers.h"

/* Macros for portability in the CAN-FD enabled FlexCAN modules in S32K1 familiy */

#if defined(MCU_S32K116) | defined(MCU_S32K118)

#define FEATURE_CAN_HAS_FD_ARRAY	{FEATURE_CAN0_HAS_FD}

#elif defined(MCU_S32K142)

#define FEATURE_CAN_HAS_FD_ARRAY	{FEATURE_CAN0_HAS_FD, FEATURE_CAN1_HAS_FD}

#elif defined(MCU_S32K144) | defined(MCU_S32K146) | defined(MCU_S32K148)

#define FEATURE_CAN_HAS_FD_ARRAY	{FEATURE_CAN0_HAS_FD, FEATURE_CAN1_HAS_FD, FEATURE_CAN2_HAS_FD}

#endif


#endif /* FLEXCAN_S32K_HPP_ */
