/* NXP Semiconductor copyright 2019 */
#include "S32K142.h"
#include "S32K142_features.h"
#include "FlexCAN_S32K.hpp"

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

public:

	/**
	 *  Get the number of CAN-FD capable FlexCAN modules in MCU
	 */
	virtual std::uint_fast8_t getInterfaceCount() const override
	{

		/**
		 * Initialize array which contains macros from MCU header which tells
		 * if each FlexCAN instance is capable of CANFD or not
		 */
		uint_fast8_t FlexCAN_CANFD_Instances[CAN_INSTANCE_COUNT] = FEATURE_CAN_HAS_FD_ARRAY;

		/* Initialize accumulator variable */
		uint_fast8_t CANFD_Instances = 0;

		/* Sum through the FlexCAN instances */
		for(std::uint_fast8_t i = 0; i<CAN_INSTANCE_COUNT ;i++)
		{
			CANFD_Instances += FlexCAN_CANFD_Instances[i];
		}

		/* Return accumulator */
		return CANFD_Instances;
	}

	virtual libuavcan::Result write(std::uint_fast8_t interface_index,
	                                    const FrameT (&frames)[MaxTxFrames],
	                                    std::size_t  frames_len,
	                                    std::size_t& out_frames_written) override
    {}

	virtual libuavcan::Result read(std::uint_fast8_t interface_index,
	                                   FrameT (&out_frames)[MaxRxFrames],
	                                   std::size_t& out_frames_read) override
    {

		/* Possible implementation with mailbonx inactivation */
    }

	/* No dynamic subscription of nodes after initialization for the current implementation */
	virtual libuavcan::Result reconfigureFilters(const typename FrameType::Filter* filter_config,
	                                                 std::size_t                   filter_config_length) override
	{
		return libuavcan::Result::NotImplemented;
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
		/* Initialize return value status */
		libuavcan::Result Status;


		/**
		 * PLL initialization for 80Mhz SysClock
		 */

		// 8 Mhz Oscillator init
			/*!
			 * SOSC Initialization (8 MHz):
			 * ===================================================
			 */
			SCG->SOSCDIV = SCG_SOSCDIV_SOSCDIV1(1)|
						   SCG_SOSCDIV_SOSCDIV2(1);  	/* SOSCDIV1 & SOSCDIV2 =1: divide by 1 		*/
			SCG->SOSCCFG  =	SCG_SOSCCFG_RANGE(2)|		/* Range=2: Medium freq (SOSC betw 1MHz-8MHz) 	*/
							SCG_SOSCCFG_EREFS_MASK;		/* HGO=0:   Config xtal osc for low power 		*/
		  	  	  	  	  	  	  	  	  	  	  	  	/* EREFS=1: Input is external XTAL 			*/

		  while(SCG->SOSCCSR & SCG_SOSCCSR_LK_MASK); 	/* Ensure SOSCCSR unlocked 							*/
		  SCG->SOSCCSR = SCG_SOSCCSR_SOSCEN_MASK; 		/* LK=0:          SOSCCSR can be written 				*/
														/* SOSCCMRE=0:    OSC CLK monitor IRQ if enabled 		*/
														/* SOSCCM=0:      OSC CLK monitor disabled 			*/
														/* SOSCERCLKEN=0: Sys OSC 3V ERCLK output clk disabled */
														/* SOSCLPEN=0:    Sys OSC disabled in VLP modes 		*/
														/* SOSCSTEN=0:    Sys OSC disabled in Stop modes 		*/
														/* SOSCEN=1:      Enable oscillator 					*/

		while(!(SCG->SOSCCSR & SCG_SOSCCSR_SOSCVLD_MASK));	/* Wait for sys OSC clk valid */


		// 160 Mhz PLL init
			/*!
			 * SPLL Initialization (160 MHz):
			 * ===================================================
			 */
		  while(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK); 	/* Ensure SPLLCSR unlocked 				*/
		  SCG->SPLLCSR &= ~SCG_SPLLCSR_SPLLEN_MASK;  	/* SPLLEN=0: SPLL is disabled (default) 	*/

		  SCG->SPLLDIV |= 	SCG_SPLLDIV_SPLLDIV1(2)|	/* SPLLDIV1 divide by 2 */
							SCG_SPLLDIV_SPLLDIV2(3);  	/* SPLLDIV2 divide by 4 */

		  SCG->SPLLCFG = SCG_SPLLCFG_MULT(24);			/* PREDIV=0: Divide SOSC_CLK by 0+1=1 		*/
		  	  	  	  	  	  	  	  	  	  	  		/* MULT=24:  Multiply sys pll by 4+24=40 	*/
												  		/* SPLL_CLK = 8MHz / 1 * 40 / 2 = 160 MHz 	*/

		  while(SCG->SPLLCSR & SCG_SPLLCSR_LK_MASK); 	/* Ensure SPLLCSR unlocked 						*/
		  SCG->SPLLCSR |= SCG_SPLLCSR_SPLLEN_MASK; 		/* LK=0:        SPLLCSR can be written 			*/
		                             	 	 	 		/* SPLLCMRE=0:  SPLL CLK monitor IRQ if enabled 	*/
		                             	 	 	 	 	/* SPLLCM=0:    SPLL CLK monitor disabled 			*/
		                             	 	 	 	 	/* SPLLSTEN=0:  SPLL disabled in Stop modes 		*/
		                             	 	 	 	 	/* SPLLEN=1:    Enable SPLL 						*/

		  while(!(SCG->SPLLCSR & SCG_SPLLCSR_SPLLVLD_MASK)); /* Wait for SPLL valid */


		// Normal run mode 80Mhz
		/*! Slow IRC is enabled with high range (8 MHz) in reset.
		 *	Enable SIRCDIV2_CLK and SIRCDIV1_CLK, divide by 1 = 8MHz
		 *  asynchronous clock source.
		 * ==========================================
		*/
			SCG->SIRCDIV = SCG_SIRCDIV_SIRCDIV1(1)
						 | SCG_SIRCDIV_SIRCDIV2(1);

		/*!
		 *  Change to normal RUN mode with 8MHz SOSC, 80 MHz PLL:
		 *  ====================================================
		 */
		  SCG->RCCR=SCG_RCCR_SCS(6)      /* Select PLL as clock source 								*/
		    |SCG_RCCR_DIVCORE(0b01)      /* DIVCORE=1, div. by 2: Core clock = 160/2 MHz = 80 MHz 		*/
		    |SCG_RCCR_DIVBUS(0b01)       /* DIVBUS=1, div. by 2: bus clock = 40 MHz 					*/
		    |SCG_RCCR_DIVSLOW(0b10);     /* DIVSLOW=2, div. by 2: SCG slow, flash clock= 26 2/3 MHz	*/

		  while (((SCG->CSR & SCG_CSR_SCS_MASK) >> SCG_CSR_SCS_SHIFT ) != 6) {}	/* Wait for sys clk src = SPLL */




		/**
		 * FlexCAN0 initialization with ISO CAN-FD
		 *
		 * 16 default message buffers (MB's)
		 * Undivided FIRC at 48Mhz as default clock source for the module
		 */

		/* Check for existence of ISO CAN-FD feature in MCU */
		if (SIM->SDID & SIM_SDID_FEATURES(40))
		{

		/* Choosing which instance to init yet to be implemented */

		PCC->PCCn[PCC_FlexCAN0_INDEX] = PCC_PCCn_CGC_MASK; /* FlexCAN0 clock gating */
		CAN0->MCR   |=  CAN_MCR_MDIS_MASK;				   /* Disable FlexCAN0 module for clock source selection */
		CAN0->CTRL1 |=  CAN_CTRL1_CLKSRC_MASK;			   /* Select peripheral clock source (undivided FIRC at 48Mhz)*/
		CAN0->MCR 	&= ~CAN_MCR_MDIS_MASK;				   /* Enable FlexCAN0 and automatic transition to freeze mode for setup */

		/* Block for freeze mode entry */
		Status = flagPollTimeout_Set(CAN0->MCR,CAN_MCR_FRZACK_MASK);

		/* Next configurations are only permitted in freeze mode */
		CAN0->MCR	|= CAN_MCR_FDEN_MASK; 		  /* Habilitate CANFD feature and leave default 16 MB's */
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

		/* FDCTRL and rest */



		/* Exit from freeze mode */
		CAN->MCR &= ~CAN_MCR_HALT_MASK;

		/* Block for freeze mode exit */
		Status = flagPollTimeout_Clear(CAN0->MCR,CAN_MCR_FRZACK_MASK);

		/* Block until module is ready */
		Status = flagPollTimeout_Clear(CAN0->MCR,CAN_MCR_NOTRDY_MASK);

		}

		/**
		 * eDMA module initialization:
		 * Uses Transfer Control Diagram instance 0 (TCD0) for
		 * 2 minor loop data transfers bursts of 32 bytes each
		 * which is the maximum data transfer size possible
		 */

		/* Initialize a DMA channel for each CANFD instance yet to be implemented */

		SIM->PLATCGC	|= SIM_PLATCGC_CGCDMA_MASK; /* DMA Clock gating */

		DMA->CR			&= ~DMA_CR_CX_MASK;			/* DMA module in normal operation */
		DMA->TCD[0].CSR &= ~DMA_TCD_CSR_START_MASK; /* Disable channel for configuration */
		DMA->TCD[0].CSR |=  DMA_TCD_CSR_DREQ(1);    /* Enable automatic request flag clear by hardware in each major loop */


		/* eDMA source and destination addresses setup */
		DMA->TCD[0].SADDR = DMA_TCD_SADDR_SADDR(0); /* Source address (CAN message buffer address) */
		DMA->TCD[0].SOFF  = DMA_TCD_SOFF_SOFF(32);  /* Source address offset for each minor loop (32 bytes)*/

		DMA->TCD[0].DADDR = DMA_TCD_DADDR_DADDR(0); /* Destination address */
		DMA->TCD[0].DOFF  = DMA_TCD_DOFF_DOFF(32);  /* Destination address offset for each minor loop (32 bytes) */

        /* eDMA data size configuration for 2 burst of 32 bytes */
		DMA->TCD[0].ATTR  = DMA_TCD_ATTR_SSIZE(5) | /* Data size in source (32 bytes) */
							DMA_TCD_ATTR_DSIZE(5);  /* Data size in destination (32 bytes) */

		/* eDMA minor and major loop configuration */
		DMA->TCD[0].NBYTES.MLNO   = DMA_TCD_NBYTES_MLNO_NBYTES(32); /* 32-byte transfer for each minor loop data burst */
		DMA->TCD[0].SLAST		  = DMA_TCD_SLAST_SLAST(-64);		/* 64-byte source address adjustment at each major loop */
		DMA->TCD[0].BITER.ELINKNO = DMA_TCD_BITER_ELINKNO(2);       /* Minor loop reload counter for each major loop */
		DMA->TCD[0].CITER.ELINKNO = DMA_TCD_CITER_ELINKNO(2);       /* Minor loop current counter value, must coincide with previous BITER */

		/* If reached end of function, status is successful */
		Status = libuavcan::Result::Success;

		/* Return code for start of S32K_InterfaceGroup */
		return Status;
    }

	/**
	 * Deinitializes the peripherals needed for the libuavcan driver layer
	 */
	virtual libuavcan::Result stopInterfaceGroup(InterfaceGroupPtrType& inout_group) override
	{
		/* Default return value status */
		libuavcan::Result Status = libuavcan::Result::Failure;


		/* DMA module deinitialization */
		DMA->CR 		|= DMA_CR_CX_MASK; /* Force the minor loop to finish and cancels the remaining data transfer */
		SIM->PLATCGC	&= SIM_PLATCGC_CGCDMA_MASK; /* Disable clock for DMA module */

		/* FlexCAN0 module deinitialization */
		/* Possible abortion of pending transmissions before clock gating */
		PCC->PCCn[PCC_FlexCAN0_INDEX] &= ~PCC_PCCn_CGC_MASK; /* Disable FlexCAN0 clock gating */


		/* If reached end of function, status is successful */
		Status = libuavcan::Result::Success;

		/* Return code for a successful stop of S32K_InterfaceGroup */
		return Status;
	}

	/* Only one filter for the Message Buffers */
	virtual std::size_t getMaxFrameFilters() const override
	{
		return 1u;
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
		return libuavcan::Result::Error;
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
		return libuavcan::Result::Error;
	}



};

} /* END namespace media */
} /* END namespace libuavcan */
