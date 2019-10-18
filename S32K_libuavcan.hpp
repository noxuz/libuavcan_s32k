/*
 * Copyright (c) 2016 - 2019, NXP.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY NXP "AS IS" AND ANY EXPRESSED OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL NXP OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef S32K_LIBUAVCAN_HPP_INCLUDED
#define S32K_LIBUAVCAN_HPP_INCLUDED

/**
 * Integration Note, this driver utilizes the next modules.
 * 	LPIT: Channels 0,1 and 3
 * 	FlexCAN: All message buffers from each instance.
 * 			 ISR priority not set, thus, it is determined by its position in the vector.
 *
 * 	Sets the MCU clocking in Normal RUN mode with the next prescalers:
 * 	CORE_CLK:  80Mhz
 * 	SYS_CLK:   80Mhz
 * 	BUS_CLK:   40Mhz
 * 	FLASH_CLK: 26.67Mhz
 *
 *  Dividers:
 *  SIRCDIV2 = 8
 *  SOSCDIV2 = 8
 *
 *	LPIT source = SOSCDIV2 (1Mhz)
 *	FlexCAN source = SYS_CLK (80Mhz)
 *
 *	FlexCAN interrupt priority Is
 *
 *	Async dividers not mentioned are left unset and SCG registers are locked
 */

/* Include desired target S32K1xx registers and features header files,
 * defaults to S32K144 from NXP's UAVCAN node board */
#include "S32K144.h"

/* libuavcan core header files */
#include "libuavcan/media/can.hpp"
#include "libuavcan/media/interfaces.hpp"
#include "libuavcan/platform/memory.hpp"

/* STL queue for the intermediate ISR buffer */
#include <deque>

/* Macros from MCU header for portability in the CAN-FD enabled FlexCAN modules in S32K1 familiy */
#if defined(MCU_S32K116) || defined(MCU_S32K118) || defined(MCU_S32K142) || defined(MCU_S32K144)
	#define TARGET_S32K_CAN_FD_COUNT	(1u)
#elif defined(MCU_S32K146)
	#define TARGET_S32K_CAN_FD_COUNT	(2u)
#elif defined(MCU_S32K148)
	#define TARGET_S32K_CAN_FD_COUNT	(3u)
#endif


namespace libuavcan
{
namespace media
{

/* Maximun capacity of intermediate frames */
static constexpr std::uint32_t Frame_Capactiy = 500u;

/* Counter for number of frames received */
static volatile std::fast8_t RX_ISRframeCount = 0;

/**
 * S32K CanFD driver layer InterfaceGroup
 * Class instantiation with the next template parameters:
 *
 * FrameT = Frame: MTUBytesParam = MaxFrameSizeBytes (64 bytes)
 *  	     	   FlagBitsCompareMask = 0x00 (default)
 * MaxTxFrames = 1 (default)
 * MaxRxFrames = 1 (default)
 */
class S32K_InterfaceGroup : public InterfaceGroup< CAN::Frame< CAN::TypeFD::MaxFrameSizeBytes> >{
private:

	/* libuavcan constants for S32K driver layer in current class */
	constexpr static std::uint_fast8_t MB_SIZE_WORDS        = 18u; /* Size in words (4 bytes) of the offset between message buffers */
	constexpr static std::uint_fast8_t MB_DATA_OFFSET       = 2u;  /* Offset in words for reaching the payload of a message buffer */
	constexpr static std::uint_fast8_t S32K_FILTER_COUNT	= 5u;  /* Number of filters supported by a single FlexCAN instace */
	constexpr static std::uint_fast8_t S32K_CANFD_COUNT 	= TARGET_S32K_CAN_FD_COUNT; /* Defined at precompilation by included target MCU header */

public:

	/**
	 *  Get the number of CAN-FD capable FlexCAN modules in MCU
	 */
	virtual std::uint_fast8_t getInterfaceCount() const override
	{
		return S32K_CANFD_COUNT;
	}

	/* Function for sending a frame through FLEXCAN, current implementation supports
	 * MaxTxFrames = 1 so frames_len must be 1
	 */
	virtual libuavcan::Result write(std::uint_fast8_t interface_index,
	                                    const FrameT (&frames)[MaxTxFrames],
	                                    std::size_t  frames_len,
	                                    std::size_t& out_frames_written) override
    {
		/* Initialize return value status */
		libuavcan::Result Status = libuavcan::Result::BufferFull;

		/* Input validation */
		if(frames_len > MaxTxFrames)
		{
			Status = libuavcan::Result::BadArgument;
		}

		std::uint32_t CODE_MB0 = ((CAN0->RAMn[0*MB_SIZE_WORDS] & CAN_RAMn_DATA_BYTE_0(0xF)) >> CAN_RAMn_DATA_BYTE_0_SHIFT );
		std::uint32_t CODE_MB1 = ((CAN0->RAMn[1*MB_SIZE_WORDS] & CAN_RAMn_DATA_BYTE_0(0xF)) >> CAN_RAMn_DATA_BYTE_0_SHIFT );
		std::uint32_t flag = 0;

		/* Check if Tx Message buffer status CODE is inactive (0b1000) */
		if( CODE_MB0 == 0x8 )
		{
			/* Transmit through MB0 */
			CAN0->IFLAG1 |= CAN_IFLAG1_BUF0I_MASK; /* Ensure interurpt flag for MB0 is cleared (write to clear register) */


			/* Get data length of the frame wished to be written */
			std::uint_fast8_t payloadLength = frames[0].getDataLength();

			/* Fill up payload from MSB to LSB in function of frame's dlc */
			for(std::uint8_t i = 0; i < (payloadLength/4); i++)
			{
				/* Build up each 32 bit word with 4 indices from frame.data uint8_t array */
				CAN0->RAMn[0*MB_SIZE_WORDS + MB_DATA_OFFSET + i] = ((std::uint32_t)(frames[0].data[(i*4) + 0] << 24))  |
													  ((std::uint32_t)(frames[0].data[(i*4) + 1] << 16))  |
													  ((std::uint32_t)(frames[0].data[(i*4) + 2] << 8))	  |
													  	  	  	  	  (frames[0].data[(i*4) + 3] << 0);
			}

			/* Fill up payload of frame's bytes that dont fill up a 32-bit word, cases of 0,1,2,3,5,6,7 byte data length */
			for(std::uint8_t i = 0; i < (payloadLength%4) ; i++)
			{
				CAN0->RAMn[0*MB_SIZE_WORDS + MB_DATA_OFFSET + (payloadLength/4)] |= (std::uint32_t)(frames[0].data[ ((payloadLength/4) * 4) + i ] << ((3-i)*8) );
			}

			/* Fill up frame ID */
			CAN0->RAMn[0*MB_SIZE_WORDS + 1] = frames[0].id & CAN_WMBn_ID_ID_MASK;

		   /* Fill up word 0 of frame and transmit it
			* Extended Data Length       (EDL) = 1
			* Bit Rate Switch 		     (BRS) = 1
			* Error State Indicator      (ESI) = 0
			* Message Buffer Code	    (CODE) = 12 ( Transmit data frame )
			* Substitute Remote Request  (SRR) = 1  ( Mandatory 1 for Tx )
			* ID Extended Bit			 (IDE) = 1
			* Remote Tx Request	         (RTR) = 0
			* Data Length Code			 (DLC) = frame.getdlc()
		    * Counter Time Stamp  (TIME STAMP) = 0 ( Handled by hardware )
			*/
			CAN0->RAMn[0*MB_SIZE_WORDS + 0] = CAN_RAMn_DATA_BYTE_1(0x60) 			  |
											  CAN_WMBn_CS_DLC(frames[0].getdlc()) |
					  	  	  	  	  	  	  CAN_RAMn_DATA_BYTE_0(0xCC);

			/* Set the return status as successfull */
			Status = libuavcan::Result::Success;

			/* Argument assignment to 1 Frame transmitted successfully */
			out_frames_written = 1;

			/* Ensure the interrupt flag is cleared after a successfull transmission */
			CAN0->IFLAG1 |= CAN_IFLAG1_BUF0I_MASK;

			/* Turn on flag for not retransmitting on next MB*/
			flag = 1;
		}
		else if ( (CODE_MB1 == 0x8 ) && (flag == 0) )
		{

			/* Transmit through MB1 */
			CAN0->IFLAG1 |= CAN_IFLAG1_BUF4TO1I(1);  /* Ensure interurpt flag for MB1 is cleared (write to clear register) */

			/* Get data length of the frame wished to be written */
			std::uint_fast8_t payloadLength = frames[0].getDataLength();

			/* Fill up payload from MSB to LSB in function of frame's dlc */
			for(std::uint8_t i = 0; i < (payloadLength/4); i++)
			{
				/* Build up each 32 bit word with 4 indices from frame.data uint8_t array */
				CAN0->RAMn[1*MB_SIZE_WORDS + MB_DATA_OFFSET + i] = ((std::uint32_t)(frames[0].data[(i*4) + 0] << 24))  |
													  ((std::uint32_t)(frames[0].data[(i*4) + 1] << 16))  |
													  ((std::uint32_t)(frames[0].data[(i*4) + 2] << 8))	  |
													  	  	  	  	  (frames[0].data[(i*4) + 3] << 0);
			}

			/* Fill up payload of frame's bytes that dont fill up a 32-bit word, cases of 0,1,2,3,5,6,7 byte data length */
			for(std::uint8_t i = 0; i < (payloadLength%4) ; i++)
			{
				CAN0->RAMn[1*MB_SIZE_WORDS + MB_DATA_OFFSET + (payloadLength/4)] |= (std::uint32_t)(frames[0].data[ ((payloadLength/4) * 4) + i ] << ((3-i)*8) );
			}

			/* Fill up frame ID */
			CAN0->RAMn[1*MB_SIZE_WORDS + 1] = frames[0].id & CAN_WMBn_ID_ID_MASK;

		   /* Fill up word 0 of frame and transmit it
			* Extended Data Length       (EDL) = 1
			* Bit Rate Switch 		     (BRS) = 1
			* Error State Indicator      (ESI) = 0
			* Message Buffer Code	    (CODE) = 12 ( Transmit data frame )
			* Substitute Remote Request  (SRR) = 1  ( Mandatory 1 for Tx )
			* ID Extended Bit			 (IDE) = 1
			* Remote Tx Request	         (RTR) = 0
			* Data Length Code			 (DLC) = frame.getdlc()
		    * Counter Time Stamp  (TIME STAMP) = 0 ( Handled by hardware )
			*/
			CAN0->RAMn[1*MB_SIZE_WORDS + 0] = CAN_RAMn_DATA_BYTE_1(0x60) 			  |
											  CAN_WMBn_CS_DLC(frames[0].getdlc()) |
					  	  	  	  	  	  	  CAN_RAMn_DATA_BYTE_0(0xCC);

			/* Set the return status as successfull */
			Status = libuavcan::Result::Success;

			/* Argument assignment to 1 Frame transmitted successfully */
			out_frames_written = 1;

			/* Ensure the interrupt flag is cleared after a successfull transmission */
			CAN0->IFLAG1 |= CAN_IFLAG1_BUF4TO1I(1);
		}

		/* Return status code */
		return Status;

    }

	/* Read from the intermediate ISR Frame buffer */
	virtual libuavcan::Result read(std::uint_fast8_t interface_index,
	                                   FrameT (&out_frames)[MaxRxFrames],
	                                   std::size_t& out_frames_read) override
    {
		/* Initialize return value and out_frames_read output reference value */
		libuavcan::Result Status = libuav::Result::Failure;
		out_frames_read = 0;

		/* Check if the ISR buffer isn't empty */
		if(!S32K_InterfaceManager::frame_ISRbuffer.empty())
		{
			/* Get the front element of the queue buffer */
			out_frames[1] = S32K_InterfaceManager::frame_ISRbuffer.front();

			/* Pop the front element of the queue buffer */
			S32K_InterfaceManager::frame_ISRbuffer.pop_front();

			/* Decrease frame count */
			RX_ISRframeCount--;

			/* Default minimal RX number of frames read */
			out_frames_read = MaxRxFrames;

			/* If read is successful, status is success */
			Status = libuavcan::Result::Success;
		}

		/* Return status code */
	    return Status;

    }

	/* Reconfigure reception filters for dynamic subscription of nodes
	 * NOTE: Since filter Iindex to reconfigure isn't provided, only up
	 * 		 to which filter to modify, the filters in the range
	 * 		 (filter_config_length, S32K_FILTER_COUNT] are left unaltered
	 */
	virtual libuavcan::Result reconfigureFilters(const typename FrameType::Filter* filter_config,
	                                                 std::size_t                   filter_config_length) override
	{
		/* Initialize return value status */
		libuavcan::Result Status = libuavcan::Result::Success;

		/* Input validation */
		if( filter_config_length > S32K_FILTER_COUNT )
		{
			Status = libuavcan::Result::BadArgument;
		}

		/* Enter freeze mode for filter reconfiguration */
		CAN0->MCR |= CAN_MCR_HALT_MASK;

		/* Block for freeze mode entry, halts any transmission or  */
		if ( isSuccess(Status) )
		{
			Status = flagPollTimeout_Set(CAN0->MCR,CAN_MCR_FRZACK_MASK);
		}

		for( std::uint_fast8_t i = 0; i < filter_config_length; i++ )
		{
			/* Setup reception MB's mask from input argument */
			CAN0->RXIMR[i+2] = filter_config[i]->mask;

			/* Setup word 0 (4 Bytes) for ith MB
			 * Extended Data Length      (EDL) = 1
			 * Bit Rate Switch 		     (BRS) = 1
			 * Error State Indicator     (ESI) = 0
			 * Message Buffer Code	    (CODE) = 4 ( Active for reception and empty )
			 * Substitute Remote Request (SRR) = 0
			 * ID Extended Bit			 (IDE) = 1
			 * Remote Tx Request	     (RTR) = 0
			 * Data Length Code			 (DLC) = 0 ( Valid for transmission only )
			 * Counter Time Stamp (TIME STAMP) = 0 ( Handled by hardware )
			 */
			CAN0->RAMn[(i+2)*MB_SIZE_WORDS] = CAN_RAMn_DATA_BYTE_0(0xC4) |
											  CAN_RAMn_DATA_BYTE_1(0x20);

			/* Setup Message buffers 2-7 29-bit extended ID from parameter */
			CAN0->RAMn[(i+2)*MB_SIZE_WORDS + 1] = filter_config[i]->id;
		}

		/* Freeze mode exit request */
		CAN0->MCR &= ~CAN_MCR_HALT_MASK;

		/* Block for freeze mode exit */
		if ( isSuccess(Status) )
		{
		Status = flagPollTimeout_Clear(CAN0->MCR,CAN_MCR_FRZACK_MASK);
		}

		/* Block until module is ready */
		if ( isSuccess(Status) )
		{
		Status = flagPollTimeout_Clear(CAN0->MCR,CAN_MCR_NOTRDY_MASK);
		}

		/* Return status code */
		return Status;
	}

	/* Block with timeout for available Message buffers */
	virtual libuavcan::Result select(libuavcan::duration::Monotonic timeout, bool ignore_write_available) override
	{
		/* Obtain timeout from object */
		constexpr uint32_t cycles_timeout = static_cast<std::uint32_t>(timeout.toMicrosecond());
		volatile  uint32_t delta          = 0; /* Declaration of delta for comparision */

		/* Disable LPIT channel 3 for loading */
		LPIT0->CLRTEN |= LPIT_CLRTEN_CLR_T_EN_3(1);

		/* Load LPIT with its maximum value */
		LPIT0->TMR[3].TVAL = LPIT_TMR_CVAL_TMR_CUR_VAL_MAS;

		/* Enable LPIT channel 3 for timeout start */
		LPIT0->SETTEN |= LPIT_SETTEN_SET_T_EN_3(1);

		/* Start of timed block */
		while( delta<cycles_timeout )
		{
			/* If ignore = true, check only RX buffers (2th-6th) */
			if(ignore_write_available)
			{
				/* Get CODE from Control and Status word of each MB */
				std::uint32_t flagMB2 = (CAN0->RAMn[2*MB_SIZE_WORDS]) & CAN_RAMn_DATA_BYTE_0(0xF);
				std::uint32_t flagMB3 = (CAN0->RAMn[3*MB_SIZE_WORDS]) & CAN_RAMn_DATA_BYTE_0(0xF);
				std::uint32_t flagMB4 = (CAN0->RAMn[4*MB_SIZE_WORDS]) & CAN_RAMn_DATA_BYTE_0(0xF);
				std::uint32_t flagMB5 = (CAN0->RAMn[5*MB_SIZE_WORDS]) & CAN_RAMn_DATA_BYTE_0(0xF);
				std::uint32_t flagMB6 = (CAN0->RAMn[6*MB_SIZE_WORDS]) & CAN_RAMn_DATA_BYTE_0(0xF);

				/* Any CODE must be 0 */
				std::uint32_t flag = (flagMB2 == 0)||(flagMB3 == 0)||(flagMB4 == 0)||(flagMB5 == 0)||(flagMB6 == 0);
			}

			/* All MB's CODE get checked for availability */
			else
			{
				/* Check inactive message buffer IMB flag, checks code 0 for Rx or 0b1000 for Tx */
				std::uint32_t flag = (CAN0->ESR2 & CAN_ESR2_IMB_MASK)
			}

			if (flag)
			{
				return libuavcan::Result::Success;
			}

			/* Get current value of delta */
			delta = LPIT_TMR_CVAL_TMR_CUR_VAL_MAS - (LPIT0->TMR[3].CVAL);

		}

		/* If this section is reached, means timeout ocurred
		 * and return timeout status */
		return libuavcan::Result::SuccessTimeout;

	}
};

/**
 * S32K CanFD driver layer InterfaceManager
 * Class instantiation with the next template parameters
 *
 * InterfaceGroupT = S32K_InterfaceGroup (previously instantiated class)
 * InterfaceGroupPtrT = S32K_InterfaceGroup* (raw pointer)
 */
class S32K_InterfaceManager : public InterfaceManager< S32K_InterfaceGroup, S32K_InterfaceGroup* >{
private:

	/* Object member created from manager instantiation */
	InterfaceGroupType S32K_InterfaceGroupObj;

	/* Frame capacity for the intermediate ISR buffer */
	constexpr static std::size_t S32K_FRAME_CAPACITY  = 500;

public:

	/* Intermediate RX buffer for ISR reception with static memory pool */
	static std::deque<FrameType, platform::PoolAllocator< S32K_FRAME_CAPACITY, sizeof(FrameType)> > frame_ISRbuffer;

	/* Initializes the peripherals needed for libuavcan driver layer */
	virtual libuavcan::Result startInterfaceGroup(const typename InterfaceGroupType::FrameType::Filter* filter_config,
	                                                  std::size_t            filter_config_length,
	                                                  InterfaceGroupPtrType& out_group) override
    {

		/* Initialize return values */
		libuavcan::Result Status = libuavcan::Result::Success;
		out_group = nullptr;

		/* Input validation */
		if( filter_config_length > S32K_FILTER_COUNT )
		{
			Status = libuavcan::Result::BadArgument;
		}

		/**
		 * LPIT0 channel initialization for timeout timer
		 */
		PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC(1); /* Clock gating to LPIT module */
		SCG->SIRCDIV |= SCG_SIRCDIV2(4) 			  /* Enable and divide by 8, 1Mhz */
		PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_PCS(2); /* Enable and select SIRCDIV2_CLK (8Mhz) */
		LPIT0->MCR |= LPIT_MCR_M_CEN(1); 			  /* Enable module for setup */

		/* Select 32-bit peridic timer mode (default) */
		LPIT0->TMR[3].TCTRL |= LPIT_TMR_TCTRL_MODE(0);


		/**
		 * SysClock initialization for feeding 80Mhz to FlexCAN
		 */

		/* System Oscillator (SOSC) initialization for 8Mhz external crystal */
		SCG->SOSCCSR &= ~SCG_SOSCCSR_LK_MASK;     /* Ensure the register is unlocked */
		SCG->SOSCCSR &= ~SCG_SOSCCSR_SOSCEN_MASK; /* Disable SOSC for setup */
		SCG->SOSCCFG  =	 SCG_SOSCCFG_EREFS_MASK | /* Setup external crystal for SOSC reference */
						 SCG_SOSCCFG_RANGE(2);	  /* Select 8Mhz range */
		SCG->SOSCDIV |=  SCG_SOSCDIV_SOSCDIV2(4); /* Divider of 8 for LPIT clock source, gets 1Mhz reference */
		SCG->SOSCCSR  =  SCG_SOSCCSR_SOSCEN_MASK; /* Enable SOSC reference */
		SCG->SOSCCSR |=  SCG_SOSCCSR_LK_MASK;	  /* Lock the register from accidental writes */

		if ( isSuccess(Status) )
		{
			Status = flagPollTimeout_Set(SCG->SOSCCSR,SCG_SOSCCSR_SOSCVLD_MASK);	/* Poll for valid SOSC reference, needs 4096 cycles*/
		}

		/* System PLL (SPLL) initialization for to 160Mhz reference */
		SCG->SPLLCSR &= ~SCG_SPLLCSR_LK_MASK;     /* Ensure the register is unlocked */
		SCG->SPLLCSR &= ~SCG_SPLLCSR_SPLLEN_MASK; /* Disable PLL for setup */
		SCG->SPLLCFG  =  SCG_SPLLCFG_MULT(24);	  /* Select multiply factor of 40 for 160Mhz SPLL_CLK */
		SCG->SPLLCSR |=  SCG_SPLLCSR_SPLLEN_MASK; /* Enable PLL */
		SCG->SPLLCSR |=  SCG_SPLLCSR_LK_MASK;     /* Lock register from accidental writes */

		if ( isSuccess(Status) )
		{
		Status = flagPollTimeout_Set(SCG->SPLLCSR,SCG_SPLLCSR_SPLLVLD_MASK); /* Poll for valid SPLL reference */
		}

		/* Normal RUN configuration for output clocks */
		SCG->RCCR    |=  SCG_RCCR_SCS(6)     |    /* Select SPLL as system clock source */
					 	 SCG_RCCR_DIVCORE(1) |	  /* Additional dividers for Normal Run mode */
						 SCG_RCCR_DIVBUS(1)  |
						 SCG_RCCR_DIVSLOW(2);

		/**
		 * CAN frames timestamping 64-bit timer initialization
		 * using chained LPIT channel 0 and 1
		 */

		LPIT0->MCR &= ~LPIT_MCR_M_CEN_MASK;			  /* Disable module for setup */
		/* Clock source option 1: (SOSCDIV2) at 1Mhz clearing previous bit configuration */
		PCC->PCCn[PCC_LPIT_INDEX] = ( PCC->PCCn[PCC_LPIT_INDEX] & ~PCC_PCCn_PCS_MASK ) | PCC_PCCn_PCS(1);
		LPIT0->MCR |= LPIT_MCR_M_CEN(1); 			  /* Enable module */

		/* Select 32-bit periodic Timer for both chained channels (default)  */
		LPIT0->TMR[0].TCTRL |= LPIT_TMR_TCTRL_MODE(0);
		LPIT0->TMR[1].TCTRL |= LPIT_TMR_TCTRL_MODE(0);

		/* Select chain mode for channel 1, this becomes the most significant 32 bits */
		LPIT0->TMR[1].TCTRL |= LPIT_TMR_TCTRL_CHAIN(1);

		/* Setup max reload value for both channels 0xFFFFFFFF */
	    LPIT0->TMR[0].TVAL = LPIT_TMR_TVAL_TMR_VAL_MASK;
	    LPIT0->TMR[1].TVAL = LPIT_TMR_TVAL_TMR_VAL_MASK;

		/* Start the timers */
		LPIT0->SETTEN |= LPIT_SETTEN_SET_T_EN_0(1) |
						 LPIT_SETTEN_SET_T_EN_1(1);

		/* Verify that the least significant 32-bit timer is counting (not locked at 0) */
		if ( isSuccess(Status) )
		{
			Status = flagPollTimeoutSet(LPIT0->TMR[0],LPIT_TMR_TVAL_TMR_VAL_MASK);
		}

		/**
		 * FlexCAN instances initialization
		 */
		PCC->PCCn[PCC_FlexCAN0_INDEX] = PCC_PCCn_CGC_MASK; /* FlexCAN0 clock gating */
		CAN0->MCR   |=  CAN_MCR_MDIS_MASK;				   /* Disable FlexCAN0 module for clock source selection */
		CAN0->CTRL1 |=  CAN_CTRL1_CLKSRC_MASK;			   /* Select SYS_CLK as source (80Mhz)*/
		CAN0->MCR 	&= ~CAN_MCR_MDIS_MASK;				   /* Enable FlexCAN0 and automatic transition to freeze mode for setup */

		/* Block for freeze mode entry */
		if ( isSuccess(Status) )
		{
		Status = flagPollTimeout_Set(CAN0->MCR,CAN_MCR_FRZACK_MASK);
		}

		/* Next configurations are only permitted in freeze mode */
		CAN0->MCR	|= CAN_MCR_FDEN_MASK  | 	  /* Habilitate CANFD feature */
					   CAN_MCR_FRZ_MASK;		  /* Enable freeze mode entry when HALT bit is asserted */
		CAN0->CTRL2 |= CAN_CTRL2_ISOCANFDEN_MASK; /* Activate the use of ISO 11898-1 CAN-FD standard */

		/* CAN Bit Timing (CBT) configuration for a nominal phase of 1 Mbit/s with 80 time quantas,
		   in accordance with Bosch 2012 specification, sample point at 83.75% */

		CAN0->CBT	|= CAN_CBT_BTF_MASK     |	 /* Enable extended bit timing configurations for CAN-FD
		 	 	 	 	 	 	 	 	 	 	    for setting up separetely nominal and data phase */
					   CAN_CBT_EPRESDIV(0)  |	 /* Prescaler divisor factor of 1 */
					   CAN_CBT_EPROPSEG(46) |	 /* Propagation segment of 47 time quantas */
					   CAN_CBT_EPSEG1(18)	|	 /* Phase buffer segment 1 of 19 time quantas */
					   CAN_CBT_EPSEG2(12)	|	 /* Phase buffer segment 2 of 13 time quantas */
					   CAN_CBT_ERJW(12);	     /* Resynchronization jump width same as PSEG2 */



		/* CAN-FD Bit Timing (FDCBT) for a data phase of 4 Mbit/s with 20 time quantas,
		   in accordance with Bosch 2012 specification, sample point at 75% */

		CAN0->FDCBT |= CAN_FDCBT_FPRESDIV(0) |  /* Prescaler divisor factor of 1 */
					   CAN_FDCBT_FPROPSEG(7) |  /* Propagation semgment of 7 time quantas (only register that doesn't add 1) */
					   CAN_FDCBT_FPSEG1(6)	 |  /* Phase buffer segment 1 of 7 time quantas */
					   CAN_FDCBT_FPSEG2(4)   |  /* Phase buffer segment 2 of 5 time quantas */
					   CAN_FDCBT_FRJW(4);       /* Resynchorinzation jump width same as PSEG2 */

		/* Additional CAN-FD configurations */
		CAN0->FDCTRL |= CAN_FDCTRL_FDRATE_MASK | /* Enable bit rate switch in data phase of frame */
						CAN_FDCTRL_TDCEN_MASK  | /* Enable transceiver delay compensation */
						CAN_FDCTRL_TDCOFF(5)   | /* Setup 5 FlexCAN clock cycles for delay compensation in data phase sampling */
						CAN_FDCTRL_MBDSR0(3);    /* Setup 64 bytes per message buffer for a maximum of 7 MB's */

		/* Message buffers are located in a dedicated RAM inside FlexCAN, they aren't affected by reset,
		 * so they must be explicitly initialized, they total 128 slots of 4 words each, which sum to 512 bytes,
		 * each MB is 72 byte in size ( 64 payload and 8 for headers )
		 */
		for(std::uint_fast8_t i = 0; i<CAN_RAMn_COUNT; i++ )
		{
			CAN0->RAMn[i] = 0;
		}

		/* Setup maximum number of message buffers as 7, 0th and 1st for transmission and 2nd-7th for reception */
		CAN0->MCR |= CAN_MCR_MAXMB(6)    |
					 CAN_MCR_SRXDIS_MASK | /* Disable self-reception of frames if ID matches */
					 CAN_MCR_IRMQ_MASK;	   /* Enable individual message buffer masking */

		/* Setup Message buffers 2-7 for reception and set filters */
		for( std::uint_fast8_t i = 0; i < filter_config_length; i++ )
		{
			/* Setup reception MB's mask from input argument */
			CAN0->RXIMR[i+2] = filter_config[i]->mask;

			/* Setup word 0 (4 Bytes) for ith MB
			 * Extended Data Length      (EDL) = 1
			 * Bit Rate Switch 		     (BRS) = 1
			 * Error State Indicator     (ESI) = 0
			 * Message Buffer Code	    (CODE) = 4 ( Active for reception and empty )
			 * Substitute Remote Request (SRR) = 0
			 * ID Extended Bit			 (IDE) = 1
			 * Remote Tx Request	     (RTR) = 0
			 * Data Length Code			 (DLC) = 0 ( Valid for transmission only )
			 * Counter Time Stamp (TIME STAMP) = 0 ( Handled by hardware )
			 */
			CAN0->RAMn[(i+2)*MB_SIZE_WORDS] = CAN_RAMn_DATA_BYTE_0(0xC4) |
											  CAN_RAMn_DATA_BYTE_1(0x20);

			/* Setup Message buffers 2-7 29-bit extended ID from parameter */
			CAN0->RAMn[(i+2)*MB_SIZE_WORDS + 1] = filter_config[i]->id;
		}

		/* Enable interrupt in NVIC for FlexCAN0 reception with default priority (ID = 81) */
		S32_NVIC->ISER[2] = 0x20000;
		//S32_NVIC->ISER[2] = 0x1000000; // FLEXCAN1 ID = 88

	    /* Enable interrupts of reception MB's (0b1111100) */
		CAN0->IMASK1 = CAN_IMASK1_BUF31TO0M(124);


		/* Exit from freeze mode */
		CAN->MCR &= ~CAN_MCR_HALT_MASK;

		/* Block for freeze mode exit */
		if ( isSuccess(Status) )
		{
		Status = flagPollTimeout_Clear(CAN0->MCR,CAN_MCR_FRZACK_MASK);
		}

		/* Block until module is ready */
		if ( isSuccess(Status) )
		{
		Status = flagPollTimeout_Clear(CAN0->MCR,CAN_MCR_NOTRDY_MASK);
		}

		/* If function ended successfully, return address of private object member */
		out_group = &S32K_InterfaceGroupObj;

		/* Return code for start of S32K_InterfaceGroup */
		return Status;
    }

	/**
	 * Deinitializes the peripherals needed for the libuavcan driver layer
	 */
	virtual libuavcan::Result stopInterfaceGroup(InterfaceGroupPtrType& inout_group) override
	{
		/* Default return value status */
		libuavcan::Result Status = libuavcan::Result::Success;

		/* FlexCAN0 module deinitialization */
		CAN0->MCR 	    |= CAN_MCR_MDIS_MASK;				         /* Disable FlexCAN0 module */
		Status = flagPollTimeout_Set(CAN0->MCR,CAN_MCR_LPMACK_MASK); /* Poll for Low Power ACK, waits for current transmission/reception to finish */
		PCC->PCCn[PCC_FlexCAN0_INDEX] &= ~PCC_PCCn_CGC_MASK; 		 /* Disable FlexCAN0 clock gating */

		/* Assign to null the pointer output argument */
		inout_group = nullptr;

		/* Return code for a successful stop of S32K_InterfaceGroup */
		return Status;
	}

	/* A single UAVCAN node is configured for having a single ID for
	 * transmission and reception of frames
	 */
	virtual std::size_t getMaxFrameFilters() const override
	{
		return S32K_FILTER_COUNT;
	}



	/**
	 * Function for block polling a bit flag until its set with a timeout using S32_Systick
	 *
	 * Parameters:
	 * 		flagRegister: Register where the flag is located
	 * 		flagMask: Mask to AND'nd with the register for isolating the flag
	 */
	libuavcan::Result flagPollTimeout_Set(volatile uint32_t &flagRegister, uint32_t flag_Mask) const
	{
		constexpr uint32_t cycles_timeout = 0xFFFFF; /* Timeout of 1/(1Mhz) * 2^20 = 1.04 seconds approx */
		volatile  uint32_t delta          = 0; 		 /* Declaration of delta for comparision */

		/* Disable LPIT channel 3 for loading */
		LPIT0->CLRTEN |= LPIT_CLRTEN_CLR_T_EN_3(1);

		/* Load LPIT with its maximum value */
		LPIT0->TMR[3].TVAL = LPIT_TMR_CVAL_TMR_CUR_VAL_MAS;

		/* Enable LPIT channel 3 for timeout start */
		LPIT0->SETTEN |= LPIT_SETTEN_SET_T_EN_3(1);

		/* Start of timed block */
		while( delta<cycles_timeout )
		{
			/* Check if the flag has been set */
			if (flagRegister & flag_Mask)
			{
				return libuavcan::Result::Success;
			}

			/* Get current value of delta */
			delta = LPIT_TMR_CVAL_TMR_CUR_VAL_MAS - (LPIT0->TMR[3].CVAL);

		}

		/* If this section is reached, means timeout ocurred
		 * and return error status is returned */
		return libuavcan::Result::Failure;
	}


	/**
	 * Function for block polling a bit flag until its cleared with a timeout using S32_Systick
	 *
	 * Parameters:
	 * 		flagRegister: Register where the flag is located
	 * 		flagMask: Mask to AND'nd with the register for isolating the flag
	 */
	libuavcan::Result flagPollTimeout_Clear(volatile uint32_t &flagRegister, uint32_t flag_Mask) const
	{
		constexpr uint32_t cycles_timeout = 0xFFFFF; /* Timeout of 1/(1Mhz) * 2^20 = 1.04 seconds approx */
		volatile  uint32_t delta          = 0; 		 /* Declaration of delta for comparision */

		/* Disable LPIT channel 3 for loading */
		LPIT0->CLRTEN |= LPIT_CLRTEN_CLR_T_EN_3(1);

		/* Load LPIT with its maximum value */
		LPIT0->TMR[3].TVAL = LPIT_TMR_CVAL_TMR_CUR_VAL_MAS;

		/* Enable LPIT channel 3 for timeout start */
		LPIT0->SETTEN |= LPIT_SETTEN_SET_T_EN_3(1);

		/* Start of timed block */
		while( delta<cycles_timeout )
		{
			/* Check if the flag has been set */
			if (!(flagRegister & flag_Mask))
			{
				return libuavcan::Result::Success;
			}

			/* Get current value of delta */
			delta = LPIT_TMR_CVAL_TMR_CUR_VAL_MAS - (LPIT0->TMR[3].CVAL);

		}

		/* If this section is reached, means timeout ocurred
		 * and return error status is returned */
		return libuavcan::Result::Failure;
	}


};

} /* END namespace media */
} /* END namespace libuavcan */

/* ISR for FlexCAN0 successful reception */
void CAN0_ORed_0_15_MB_IRQHandler(void)
{

	/* Before anything, get a timestamp  */
	libuavcan::time::Monotonic timestamp_ISR = (uint64_t)( ( (uint64_t)(0xFFFFFFFF - LPIT0->TMR[1].CVAL) << 32)  | (  0xFFFFFFFF - LPIT0->TMR[0].CVAL ));

	/* Initialize variable for finding which MB received */
	std::uint8_t MB_index = 0;

	/* Check which MB caused the interrupt */
	switch( CAN0->IFLAG1 )
		case 0x4:
			MB_index = 2;
		case 0x8:
			MB_index = 3;
		case 0x10:
			MB_index = 4;
		case 0x20:
			MB_index = 5;
		case 0x40:
			MB_index = 6;

	/* Receive a frame only if the buffer its under its capacity */
	if (RX_ISRframeCount <= Frame_Capactiy )
	{
		/* Increase frame count */
		RX_ISRframeCount++;

		/* Parse the Message buffer, read of the Control and status register locks the MB */

		/* Get dlc and convert to data length in bytes */
		CAN::FrameDLC dlc_ISR = ((CAN0->RAMn[MB_index*MB_SIZE_WORDS + 0]) & CAN_WMBn_CS_DLC_MASK ) >> CAN_WMBn_CS_DLC_SHIFT;
		std::uint_fast8_t payloadLength_ISR = CAN::dlcToLength(dlc_ISR);

		/* Get the id */
		std::uint32_t id_ISR = (CAN0->RAMn[MB_index*MB_SIZE_WORDS + 1]) & CAN_WMBn_ID_ID_MASK;

		/* Array for parsing from native uint32_t to uint8_t */
		std::uint8_t data_ISR_byte[payloadLength_ISR];

		/* Parse the full words of the MB in bytes */
		for(std::uint_fast8_t i = 0; i < payloadLength_ISR; i++)
		{
			data_ISR_byte[i] = ( CAN0->RAMn[MB_index*MB_SIZE_WORDS + MB_DATA_OFFSET + i/4] & (0xFF << 8*(3 - i%4) ) ) >> 8*(3 - i%4) ;
		}

		/* Parse remaining bytes that don't complete up to a word if there are */
		for(std::uint_fast8_t i = 0; i < (payloadLength_ISR%4); i++)
		{
			data_ISR_byte[ payloadLength_ISR - (payloadLength_ISR%4) + i] = ( CAN0->RAMn[MB_index*MB_SIZE_WORDS + MB_DATA_OFFSET + (payloadLength_ISR/4)] & (0xFF << 8*(3-i)) ) >> 8*(3-i);
		}

		/* Create Frame object with constructor */
		CAN::Frame< CAN::TypeFD::MaxFrameSizeBytes> FrameISR(id_ISR,data_ISR_byte,dlc_ISR,timestamp_ISR);

		/* Insert the frame into the queue */
		S32K_InterfaceManager::frame_ISRbuffer.push_back(FrameISR);
	}

	/* Unlock the MB by reading the timer register */
	(std::void)(CAN0->TIMER);

	/* Clear MB interrupt flag (write 1 to clear)*/
	CAN0->IFLAG1 |= (1<<MB_index);

}

#endif
