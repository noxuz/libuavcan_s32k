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

#include "S32K146.h"

#include "libuavcan/media/can.hpp"
#include "libuavcan/media/interfaces.hpp"


namespace libuavcan
{
namespace media
{
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

	/* libuavcan constants for S32K driver layer */
	constexpr std::uint_fast8_t S32K146_CANFD_COUNT = 2u;  /* Number of CAN-FD capable FlexCAN modules  */
	constexpr std::uint_fast8_t MB_SIZE_WORDS       = 18u; /* Size in words (4 bytes) of the offset between message buffers */
	constexpr std::size_t		S32K_FILTER_COUNT	= 1u;  /* Number of filters supported by a single UAVCAN node */

public:

	/**
	 *  Get the number of CAN-FD capable FlexCAN modules in MCU
	 */
	virtual std::uint_fast8_t getInterfaceCount() const override
	{
		/* 2 FlexCAN instances in S32K146 are CAN-FD capable */
		return S32K146_CANFD_COUNT;
	}

	virtual libuavcan::Result write(std::uint_fast8_t interface_index,
	                                    const FrameT (&frames)[MaxTxFrames],
	                                    std::size_t  frames_len,
	                                    std::size_t& out_frames_written) override
    {

		/* Initialize return value status */
		libuavcan::Result Status = libuavcan::Result::Success;

		/* Verify that the input frame is of 64-byte payload
		 * o tal vez llenar con bytepaddingPattern del CAN */
		if (frames[0].getDLC() != CAN::FrameDLC::CodeForLength64)
		{
			Status = libuavcan::Result::BadArgument;
		}




		/* Return status code */
		return Status;

    }

	virtual libuavcan::Result read(std::uint_fast8_t interface_index,
	                                   FrameT (&out_frames)[MaxRxFrames],
	                                   std::size_t& out_frames_read) override
    {


    }

	/* Reconfigure reception filters for dynamic subscription of nodes */
	virtual libuavcan::Result reconfigureFilters(const typename FrameType::Filter* filter_config,
	                                                 std::size_t                   filter_config_length) override
	{
		/* Initialize return value status */
		libuavcan::Result Status = libuavcan::Result::Success;

		/* Enter freeze mode for filter reconfiguration */
		CAN0->MCR |= CAN_MCR_HALT_MASK;

		/* Block for freeze mode entry, halts any transmission or  */
		if ( isSuccess(Status) )
		{
			Status = flagPollTimeout_Set(CAN0->MCR,CAN_MCR_FRZACK_MASK);
		}

		/* Setup word 0 (4 Bytes) for MB0
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
		CAN0->RAMn[0] = CAN_RAMn_DATA_BYTE_0(0xC4) |
						CAN_RAMn_DATA_BYTE_1(0x20);

		/* Setup Message buffer 29-bit extended ID from parameter */
		CAN0->RAMn[1] = filter_config->id;

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

	virtual libuavcan::Result select(libuavcan::duration::Monotonic timeout, bool ignore_write_available) override
	{

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

public:

	/* Initializes the peripherals needed for libuavcan driver layer */
	virtual libuavcan::Result startInterfaceGroup(const typename InterfaceGroupType::FrameType::Filter* filter_config,
	                                                  std::size_t            filter_config_length,
	                                                  InterfaceGroupPtrType& out_group) override
    {
		/* Initialize return values */
		libuavcan::Result Status = libuavcan::Result::Success;
		out_group = nullptr;

		/**
		 * SysClock initialization for feeding 80Mhz to FlexCAN
		 */

		/* System Oscillator (SOSC) initialization for 8Mhz external crystal */
		SCG->SOSCCSR &= ~SCG_SOSCCSR_LK_MASK;     /* Ensure the register is unlocked */
		SCG->SOSCCSR &= ~SCG_SOSCCSR_SOSCEN_MASK; /* Disable SOSC for setup */
		SCG->SOSCCFG  =	 SCG_SOSCCFG_EREFS_MASK | /* Setup external crystal for SOSC reference */
						 SCG_SOSCCFG_RANGE(2);	  /* Select 8Mhz range */
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
		 * FlexCAN instances initialization
		 */
		PCC->PCCn[PCC_FlexCAN0_INDEX] = PCC_PCCn_CGC_MASK; /* FlexCAN0 clock gating */
		CAN0->MCR   |=  CAN_MCR_MDIS_MASK;				   /* Disable FlexCAN0 module for clock source selection */
		CAN0->CTRL1 |=  CAN_CTRL1_CLKSRC_MASK;			   /* Select peripheral clock source (undivided FIRC at 48Mhz)*/
		CAN0->MCR 	&= ~CAN_MCR_MDIS_MASK;				   /* Enable FlexCAN0 and automatic transition to freeze mode for setup */

		/* Block for freeze mode entry */
		if ( isSuccess(Status) )
		{
		Status = flagPollTimeout_Set(CAN0->MCR,CAN_MCR_FRZACK_MASK);
		}

		/* Next configurations are only permitted in freeze mode */
		CAN0->MCR	|= CAN_MCR_FDEN_MASK  | 	  /* Habilitate CANFD feature and leave default 16 MB's */
					   CAN_MCR_FRZ_MASK;		  /* Enable freeze mode entry when HALT bit is asserted */
		CAN0->CTRL2 |= CAN_CTRL2_ISOCANFDEN_MASK; /* Activate ISO CAN-FD operation*/

		/* CAN Bit Timing (CBT) configuration for a nominal phase of 1 Mbit/s with 24 time quantas,
		   in accordance with Bosch 2012 specification, sample point at 70.8% */

		CAN0->CBT	|= CAN_CBT_BTF_MASK     |	 /* Enable extended bit timing configurations for CAN-FD
		 	 	 	 	 	 	 	 	 	 	    for setting up separetely nominal and data phase */
					   CAN_CBT_EPRESDIV(0)  |	 /* Prescaler divisor factor of 1 for */
					   CAN_CBT_EPROPSEG(46) |	 /* Propagation segment of 47 time quantas */
					   CAN_CBT_EPSEG1(18)	|	 /* Phase buffer segment 1 of 19 time quantas */
					   CAN_CBT_EPSEG2(12)	|	 /* Phase buffer segment 2 of 13 time quantas */
					   CAN_CBT_ERJW(12);	     /* Resynchronization jump width same as PSEG2 */



		/* CAN-FD Bit Timing (FDCBT) for a data phase of 4 Mbit/s with 12 time quantas,
		   in accordance with Bosch 2012 specification */

		CAN0->FDCBT |= CAN_FDCBT_FPRESDIV(0) |  /* Prescaler divisor factor of 1 */
					   CAN_FDCBT_FPROPSEG(7) |  /* Propagation semgment of 8 time quantas */
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
		for(int i = 0; i<128; i++ )
		{
			CAN0->RAMn[i] = 0;
		}

		/* Setup maximum number of message buffers as 2, 0th for reception and 1th for transmission
		 * excludes MB's 3th-7th from the arbitration process */
		CAN0->MCR |= CAN_MCR_MAXMB(1) |
					 CAN_MCR_SRXDIS_MASK; /* Disable self-reception of frames if ID matches */


		/* Setup Message buffer 0 for reception */
		CAN0->MCR |= CAN_MCR_IRMQ_MASK; /* Enable individual message buffer masking */
		CAN0->RXIMR[0] = filter_config->mask; /* Setup MB0 reception mask from argument */

		/* Setup word 0 (4 Bytes) for MB0
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
		CAN0->RAMn[0] = CAN_RAMn_DATA_BYTE_0(0xC4) |
					    CAN_RAMn_DATA_BYTE_1(0x20);

		/* Setup Message buffer 29-bit extended ID from parameter */
		CAN0->RAMn[1] = filter_config->id;

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


		/**
		 * eDMA module initialization:
		 * Uses Transfer Control Diagram instance 0 (TCD0) for
		 * 1 minor loop data transfers burst of 64 bytes
		 */
		SIM->PLATCGC	|= SIM_PLATCGC_CGCDMA_MASK; /* DMA Clock gating */

		DMA->CR			&= ~DMA_CR_CX_MASK;			/* DMA module in normal operation */
		DMA->TCD[0].CSR &= ~DMA_TCD_CSR_START_MASK; /* Disable channel for configuration */
		DMA->TCD[0].CSR |=  DMA_TCD_CSR_DREQ(1);    /* Enable automatic request flag clear by hardware in each major loop */


		/* eDMA source and destination addresses setup */
		DMA->TCD[0].SADDR = DMA_TCD_SADDR_SADDR(0); /* Source address (CAN message buffer address) */
		DMA->TCD[0].SOFF  = DMA_TCD_SOFF_SOFF(1);   /* Source address offset for internal transfer (1 bytes)*/

		DMA->TCD[0].DADDR = DMA_TCD_DADDR_DADDR(0); /* Destination address */
		DMA->TCD[0].DOFF  = DMA_TCD_DOFF_DOFF(1);   /* Destination address offset for each internal transfer (1 bytes) */

        /* eDMA data size configuration for 1 burst of 64 bytes, handled by eDMA as 64 bursts of a single byte */
		DMA->TCD[0].ATTR  = DMA_TCD_ATTR_SSIZE(0) | /* Data size burst in source (1 byte) */
							DMA_TCD_ATTR_DSIZE(0);  /* Data size burst in destination (1 byte) */

		/* eDMA minor and major loop configuration */
		DMA->TCD[0].NBYTES.MLNO   = DMA_TCD_NBYTES_MLNO_NBYTES(64); /* eDMA counts up to 64 bytes transferred for each minor loop (16 internal transfers) */
		DMA->TCD[0].SLAST		  = DMA_TCD_SLAST_SLAST(-64);		/* 64-byte source address adjustment at each major loop */
		DMA->TCD[0].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO(1);       /* Minor loop reload counter for each major loop, each major loop consists of 1 minor loop */
		DMA->TCD[0].CITER.ELINKNO = DMA_TCD_CITER_ELINKNO(1);       /* Minor loop current counter value, must coincide with previous BITER */

		/* If function ended successfully, set pointer to this base class address */
		out_group = static_cast<InterfaceGroupPtrType>(this);

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

		/* DMA module deinitialization */
		DMA->CR 		|= DMA_CR_CX_MASK; 			/* Force the minor loop to finish and cancels the remaining data transfer */
		SIM->PLATCGC	&= SIM_PLATCGC_CGCDMA_MASK; /* Disable clock for DMA module */

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
		/* Tuning parameter for the number of cpu clock cycles to block for the flag */
		constexpr uint32_t cycles_timeout = S32_SysTick_RVR_RELOAD_MASK; /* Max value = 0xFFFFFF due to 24 bit SysTick which is approx 0.3 seconds */
		volatile  uint32_t delta          = 0; 						   	 /* Declaration of delta for comparision */

		/* Initialize SysTick*/
		S32_SysTick->CSR = 0; /* Systick disable for setup */

		/* Load maximun reload value, SysTick is decremental counter */
		S32_SysTick->RVR = S32_SysTick_RVR_RELOAD_MASK;

		/* Enable SysTick and select processor clock as source clock (48Mhz) */
		S32_SysTick->CSR |= S32_SysTick_CSR_ENABLE_MASK | S32_SysTick_CSR_CLKSOURCE_MASK;

		/* Start of timed block */
		while( delta<cycles_timeout )
		{
			/* Check if the flag has been set */
			if (flagRegister & flag_Mask)
			{
				return libuavcan::Result::Success;
			}

			/* Get current value of delta */
			delta = S32_SysTick_RVR_RELOAD_MASK - (S32_SysTick->CVR);

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
		/* Tuning parameter for the number of cpu clock cycles to block for the flag */
		constexpr uint32_t cycles_timeout = S32_SysTick_RVR_RELOAD_MASK; /* Max value = 0xFFFFFF due to 24 bit SysTick which is approx 0.3 seconds */
		volatile  uint32_t delta          = 0; 						   	 /* Declaration of delta for comparision */

		/* Initialize SysTick*/
		S32_SysTick->CSR = 0; /* Systick disable for setup */

		/* Load maximun reload value, SysTick is decremental counter */
		S32_SysTick->RVR = S32_SysTick_RVR_RELOAD_MASK;

		/* Enable SysTick and select processor clock as source clock (48Mhz) */
		S32_SysTick->CSR |= S32_SysTick_CSR_ENABLE_MASK | S32_SysTick_CSR_CLKSOURCE_MASK;

		/* Start of timed block */
		while( delta<cycles_timeout )
		{
			/* Check if the flag has been set */
			if (!(flagRegister & flag_Mask))
			{
				return libuavcan::Result::Success;
			}

			/* Get current value of delta */
			delta = S32_SysTick_RVR_RELOAD_MASK - (S32_SysTick->CVR);

		}

		/* If this section is reached, means timeout ocurred
		 * and return error status is returned */
		return libuavcan::Result::Failure;
	}



};

} /* END namespace media */
} /* END namespace libuavcan */

#endif
