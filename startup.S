//vector table, find way to overwrite default vector with later defined vector in code like in c version.
//setup clocks, prefetch
//zero bss
//branch to main

.syntax unified
.cpu cortex-m4
.thumb

.global reset_handler
.global default_handler
.global g_pfnVectors

.section .vector_table,"a",%progbits
.type  g_pfnVectors, %object
.size  g_pfnVectors, .-g_pfnVectors//why?, aslo on the functions?

g_pfnVectors:
	.word  _estack
	.word  reset_handler
	.word  NMI_Handler
  	.word  HardFault_Handler
  	.word  MemManage_Handler
  	.word  BusFault_Handler
  	.word  UsageFault_Handler
  	.word  0
  	.word  0
  	.word  0
  	.word  0
  	.word  SVC_Handler
  	.word  DebugMon_Handler
  	.word  0
  	.word  PendSV_Handler
  	.word  SysTick_Handler

  /* External Interrupts */
  	.word     WWDG_IRQHandler                   /* Window WatchDog              */
  	.word     PVD_IRQHandler                    /* PVD through EXTI Line detection */
  	.word     TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */
  	.word     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */
  	.word     FLASH_IRQHandler                  /* FLASH                        */
  	.word     RCC_IRQHandler                    /* RCC                          */
  	.word     EXTI0_IRQHandler                  /* EXTI Line0                   */
  	.word     EXTI1_IRQHandler                  /* EXTI Line1                   */
  	.word     EXTI2_IRQHandler                  /* EXTI Line2                   */
  	.word     EXTI3_IRQHandler                  /* EXTI Line3                   */
  	.word     EXTI4_IRQHandler                  /* EXTI Line4                   */
  	.word     DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */
  	.word     DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */
  	.word     DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */
  	.word     DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */
  	.word     DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */
  	.word     DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */
  	.word     DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */
  	.word     ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */
  	.word     CAN1_TX_IRQHandler                /* CAN1 TX                      */
  	.word     CAN1_RX0_IRQHandler               /* CAN1 RX0                     */
  	.word     CAN1_RX1_IRQHandler               /* CAN1 RX1                     */
  	.word     CAN1_SCE_IRQHandler               /* CAN1 SCE                     */
  	.word     EXTI9_5_IRQHandler                /* External Line[9:5]s          */
  	.word     TIM1_BRK_TIM9_IRQHandler          /* TIM1 Break and TIM9          */
  	.word     TIM1_UP_TIM10_IRQHandler          /* TIM1 Update and TIM10        */
  	.word     TIM1_TRG_COM_TIM11_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
  	.word     TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */
  	.word     TIM2_IRQHandler                   /* TIM2                         */
  	.word     TIM3_IRQHandler                   /* TIM3                         */
  	.word     TIM4_IRQHandler                   /* TIM4                         */
  	.word     I2C1_EV_IRQHandler                /* I2C1 Event                   */
  	.word     I2C1_ER_IRQHandler                /* I2C1 Error                   */
  	.word     I2C2_EV_IRQHandler                /* I2C2 Event                   */
  	.word     I2C2_ER_IRQHandler                /* I2C2 Error                   */
  	.word     SPI1_IRQHandler                   /* SPI1                         */
  	.word     SPI2_IRQHandler                   /* SPI2                         */
  	.word     USART1_IRQHandler                 /* USART1                       */
  	.word     USART2_IRQHandler                 /* USART2                       */
  	.word     USART3_IRQHandler                 /* USART3                       */
  	.word     EXTI15_10_IRQHandler              /* External Line[15:10]s        */
  	.word     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */
  	.word     OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI line */
  	.word     TIM8_BRK_TIM12_IRQHandler         /* TIM8 Break and TIM12         */
  	.word     TIM8_UP_TIM13_IRQHandler          /* TIM8 Update and TIM13        */
  	.word     TIM8_TRG_COM_TIM14_IRQHandler     /* TIM8 Trigger and Commutation and TIM14 */
  	.word     TIM8_CC_IRQHandler                /* TIM8 Capture Compare         */
  	.word     DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */
  	.word     FSMC_IRQHandler                   /* FSMC                         */
  	.word     SDIO_IRQHandler                   /* SDIO                         */
  	.word     TIM5_IRQHandler                   /* TIM5                         */
  	.word     SPI3_IRQHandler                   /* SPI3                         */
  	.word     UART4_IRQHandler                  /* UART4                        */
  	.word     UART5_IRQHandler                  /* UART5                        */
  	.word     TIM6_DAC_IRQHandler               /* TIM6 and DAC1&2 underrun errors */
  	.word     TIM7_IRQHandler                   /* TIM7                         */
  	.word     DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */
  	.word     DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */
  	.word     DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */
  	.word     DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */
  	.word     DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */
  	.word     ETH_IRQHandler                    /* Ethernet                     */
  	.word     ETH_WKUP_IRQHandler               /* Ethernet Wakeup through EXTI line */
  	.word     CAN2_TX_IRQHandler                /* CAN2 TX                      */
  	.word     CAN2_RX0_IRQHandler               /* CAN2 RX0                     */
  	.word     CAN2_RX1_IRQHandler               /* CAN2 RX1                     */
  	.word     CAN2_SCE_IRQHandler               /* CAN2 SCE                     */
  	.word     OTG_FS_IRQHandler                 /* USB OTG FS                   */
  	.word     DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */
  	.word     DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */
  	.word     DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */
  	.word     USART6_IRQHandler                 /* USART6                       */
  	.word     I2C3_EV_IRQHandler                /* I2C3 event                   */
  	.word     I2C3_ER_IRQHandler                /* I2C3 error                   */
  	.word     OTG_HS_EP1_OUT_IRQHandler         /* USB OTG HS End Point 1 Out   */
  	.word     OTG_HS_EP1_IN_IRQHandler          /* USB OTG HS End Point 1 In    */
  	.word     OTG_HS_WKUP_IRQHandler            /* USB OTG HS Wakeup through EXTI */
  	.word     OTG_HS_IRQHandler                 /* USB OTG HS                   */
  	.word     DCMI_IRQHandler                   /* DCMI                         */
  	.word     CRYP_IRQHandler                   /* CRYP crypto                  */
  	.word     HASH_RNG_IRQHandler               /* Hash and Rng                 */
  	.word     FPU_IRQHandler                    /* FPU                          */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the default_handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/
   .weak      NMI_Handler
   .thumb_set NMI_Handler,default_handler

   .weak      HardFault_Handler
   .thumb_set HardFault_Handler,default_handler

   .weak      MemManage_Handler
   .thumb_set MemManage_Handler,default_handler

   .weak      BusFault_Handler
   .thumb_set BusFault_Handler,default_handler

   .weak      UsageFault_Handler
   .thumb_set UsageFault_Handler,default_handler

   .weak      SVC_Handler
   .thumb_set SVC_Handler,default_handler

   .weak      DebugMon_Handler
   .thumb_set DebugMon_Handler,default_handler

   .weak      PendSV_Handler
   .thumb_set PendSV_Handler,default_handler

   .weak      SysTick_Handler
   .thumb_set SysTick_Handler,default_handler

   .weak      WWDG_IRQHandler
   .thumb_set WWDG_IRQHandler,default_handler

   .weak      PVD_IRQHandler
   .thumb_set PVD_IRQHandler,default_handler

   .weak      TAMP_STAMP_IRQHandler
   .thumb_set TAMP_STAMP_IRQHandler,default_handler

   .weak      RTC_WKUP_IRQHandler
   .thumb_set RTC_WKUP_IRQHandler,default_handler

   .weak      FLASH_IRQHandler
   .thumb_set FLASH_IRQHandler,default_handler

   .weak      RCC_IRQHandler
   .thumb_set RCC_IRQHandler,default_handler

   .weak      EXTI0_IRQHandler
   .thumb_set EXTI0_IRQHandler,default_handler

   .weak      EXTI1_IRQHandler
   .thumb_set EXTI1_IRQHandler,default_handler

   .weak      EXTI2_IRQHandler
   .thumb_set EXTI2_IRQHandler,default_handler

   .weak      EXTI3_IRQHandler
   .thumb_set EXTI3_IRQHandler,default_handler

   .weak      EXTI4_IRQHandler
   .thumb_set EXTI4_IRQHandler,default_handler

   .weak      DMA1_Stream0_IRQHandler
   .thumb_set DMA1_Stream0_IRQHandler,default_handler

   .weak      DMA1_Stream1_IRQHandler
   .thumb_set DMA1_Stream1_IRQHandler,default_handler

   .weak      DMA1_Stream2_IRQHandler
   .thumb_set DMA1_Stream2_IRQHandler,default_handler

   .weak      DMA1_Stream3_IRQHandler
   .thumb_set DMA1_Stream3_IRQHandler,default_handler

   .weak      DMA1_Stream4_IRQHandler
   .thumb_set DMA1_Stream4_IRQHandler,default_handler

   .weak      DMA1_Stream5_IRQHandler
   .thumb_set DMA1_Stream5_IRQHandler,default_handler

   .weak      DMA1_Stream6_IRQHandler
   .thumb_set DMA1_Stream6_IRQHandler,default_handler

   .weak      ADC_IRQHandler
   .thumb_set ADC_IRQHandler,default_handler

   .weak      CAN1_TX_IRQHandler
   .thumb_set CAN1_TX_IRQHandler,default_handler

   .weak      CAN1_RX0_IRQHandler
   .thumb_set CAN1_RX0_IRQHandler,default_handler

   .weak      CAN1_RX1_IRQHandler
   .thumb_set CAN1_RX1_IRQHandler,default_handler

   .weak      CAN1_SCE_IRQHandler
   .thumb_set CAN1_SCE_IRQHandler,default_handler

   .weak      EXTI9_5_IRQHandler
   .thumb_set EXTI9_5_IRQHandler,default_handler

   .weak      TIM1_BRK_TIM9_IRQHandler
   .thumb_set TIM1_BRK_TIM9_IRQHandler,default_handler

   .weak      TIM1_UP_TIM10_IRQHandler
   .thumb_set TIM1_UP_TIM10_IRQHandler,default_handler

   .weak      TIM1_TRG_COM_TIM11_IRQHandler
   .thumb_set TIM1_TRG_COM_TIM11_IRQHandler,default_handler

   .weak      TIM1_CC_IRQHandler
   .thumb_set TIM1_CC_IRQHandler,default_handler

   .weak      TIM2_IRQHandler
   .thumb_set TIM2_IRQHandler,default_handler

   .weak      TIM3_IRQHandler
   .thumb_set TIM3_IRQHandler,default_handler

   .weak      TIM4_IRQHandler
   .thumb_set TIM4_IRQHandler,default_handler

   .weak      I2C1_EV_IRQHandler
   .thumb_set I2C1_EV_IRQHandler,default_handler

   .weak      I2C1_ER_IRQHandler
   .thumb_set I2C1_ER_IRQHandler,default_handler

   .weak      I2C2_EV_IRQHandler
   .thumb_set I2C2_EV_IRQHandler,default_handler

   .weak      I2C2_ER_IRQHandler
   .thumb_set I2C2_ER_IRQHandler,default_handler

   .weak      SPI1_IRQHandler
   .thumb_set SPI1_IRQHandler,default_handler

   .weak      SPI2_IRQHandler
   .thumb_set SPI2_IRQHandler,default_handler

   .weak      USART1_IRQHandler
   .thumb_set USART1_IRQHandler,default_handler

   .weak      USART2_IRQHandler
   .thumb_set USART2_IRQHandler,default_handler

   .weak      USART3_IRQHandler
   .thumb_set USART3_IRQHandler,default_handler

   .weak      EXTI15_10_IRQHandler
   .thumb_set EXTI15_10_IRQHandler,default_handler

   .weak      RTC_Alarm_IRQHandler
   .thumb_set RTC_Alarm_IRQHandler,default_handler

   .weak      OTG_FS_WKUP_IRQHandler
   .thumb_set OTG_FS_WKUP_IRQHandler,default_handler

   .weak      TIM8_BRK_TIM12_IRQHandler
   .thumb_set TIM8_BRK_TIM12_IRQHandler,default_handler

   .weak      TIM8_UP_TIM13_IRQHandler
   .thumb_set TIM8_UP_TIM13_IRQHandler,default_handler

   .weak      TIM8_TRG_COM_TIM14_IRQHandler
   .thumb_set TIM8_TRG_COM_TIM14_IRQHandler,default_handler

   .weak      TIM8_CC_IRQHandler
   .thumb_set TIM8_CC_IRQHandler,default_handler

   .weak      DMA1_Stream7_IRQHandler
   .thumb_set DMA1_Stream7_IRQHandler,default_handler

   .weak      FSMC_IRQHandler
   .thumb_set FSMC_IRQHandler,default_handler

   .weak      SDIO_IRQHandler
   .thumb_set SDIO_IRQHandler,default_handler

   .weak      TIM5_IRQHandler
   .thumb_set TIM5_IRQHandler,default_handler

   .weak      SPI3_IRQHandler
   .thumb_set SPI3_IRQHandler,default_handler

   .weak      UART4_IRQHandler
   .thumb_set UART4_IRQHandler,default_handler

   .weak      UART5_IRQHandler
   .thumb_set UART5_IRQHandler,default_handler

   .weak      TIM6_DAC_IRQHandler
   .thumb_set TIM6_DAC_IRQHandler,default_handler

   .weak      TIM7_IRQHandler
   .thumb_set TIM7_IRQHandler,default_handler

   .weak      DMA2_Stream0_IRQHandler
   .thumb_set DMA2_Stream0_IRQHandler,default_handler

   .weak      DMA2_Stream1_IRQHandler
   .thumb_set DMA2_Stream1_IRQHandler,default_handler

   .weak      DMA2_Stream2_IRQHandler
   .thumb_set DMA2_Stream2_IRQHandler,default_handler

   .weak      DMA2_Stream3_IRQHandler
   .thumb_set DMA2_Stream3_IRQHandler,default_handler

   .weak      DMA2_Stream4_IRQHandler
   .thumb_set DMA2_Stream4_IRQHandler,default_handler

   .weak      ETH_IRQHandler
   .thumb_set ETH_IRQHandler,default_handler

   .weak      ETH_WKUP_IRQHandler
   .thumb_set ETH_WKUP_IRQHandler,default_handler

   .weak      CAN2_TX_IRQHandler
   .thumb_set CAN2_TX_IRQHandler,default_handler

   .weak      CAN2_RX0_IRQHandler
   .thumb_set CAN2_RX0_IRQHandler,default_handler

   .weak      CAN2_RX1_IRQHandler
   .thumb_set CAN2_RX1_IRQHandler,default_handler

   .weak      CAN2_SCE_IRQHandler
   .thumb_set CAN2_SCE_IRQHandler,default_handler

   .weak      OTG_FS_IRQHandler
   .thumb_set OTG_FS_IRQHandler,default_handler

   .weak      DMA2_Stream5_IRQHandler
   .thumb_set DMA2_Stream5_IRQHandler,default_handler

   .weak      DMA2_Stream6_IRQHandler
   .thumb_set DMA2_Stream6_IRQHandler,default_handler

   .weak      DMA2_Stream7_IRQHandler
   .thumb_set DMA2_Stream7_IRQHandler,default_handler

   .weak      USART6_IRQHandler
   .thumb_set USART6_IRQHandler,default_handler

   .weak      I2C3_EV_IRQHandler
   .thumb_set I2C3_EV_IRQHandler,default_handler

   .weak      I2C3_ER_IRQHandler
   .thumb_set I2C3_ER_IRQHandler,default_handler

   .weak      OTG_HS_EP1_OUT_IRQHandler
   .thumb_set OTG_HS_EP1_OUT_IRQHandler,default_handler

   .weak      OTG_HS_EP1_IN_IRQHandler
   .thumb_set OTG_HS_EP1_IN_IRQHandler,default_handler

   .weak      OTG_HS_WKUP_IRQHandler
   .thumb_set OTG_HS_WKUP_IRQHandler,default_handler

   .weak      OTG_HS_IRQHandler
   .thumb_set OTG_HS_IRQHandler,default_handler

   .weak      DCMI_IRQHandler
   .thumb_set DCMI_IRQHandler,default_handler

   .weak      CRYP_IRQHandler
   .thumb_set CRYP_IRQHandler,default_handler

   .weak      HASH_RNG_IRQHandler
   .thumb_set HASH_RNG_IRQHandler,default_handler

   .weak      FPU_IRQHandler
   .thumb_set FPU_IRQHandler,default_handler

.section .text.reset_handler,"ax",%progbits
.balign 4
.thumb_func
	reset_handler:

	ldr r0, =_sidata
	ldr r1, =_sdata
	ldr r2, =_edata
	ldr r3, =_ebss//load multiple?

	loop:
	ldr r4, [r0], 4
	str r4, [r1], 4
	subs r5, r2, r0
	bne loop

	loop2:
	str r5, [r2], 4
	cmp r3, r2
	bne loop2


	b main
	.balign 4
	.pool//align pool?(i thins its not neccecairy when you use the 32 bit encoding of ldr), just use word?

.section .text.default_handler,"ax",%progbits
.balign 2
.thumb_func// different bc this is an interrupt?
	default_handler:
	b.n default_handler
