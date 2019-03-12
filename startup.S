//vector table, find way to overwrite default vector with later defined vector in code like in c version.
//setup clocks, prefetch
//zero bss
//branch to main

.syntax unified
.cpu cortex-m4
.thumb

.global reset_handler
.global default_handler
.global Vectors

.section .vector_table,"a",%progbits
.type  Vectors, %object
.size  Vectors, .-Vectors//why?, aslo on the functions?

Vectors:
	.4byte  _estack
	.4byte  main
	.4byte  NMI_Handler
  	.4byte  HardFault_Handler
  	.4byte  MemManage_Handler
  	.4byte  BusFault_Handler
  	.4byte  UsageFault_Handler
  	.4byte  0
  	.4byte  0
  	.4byte  0
  	.4byte  0
  	.4byte  SVC_Handler
  	.4byte  DebugMon_Handler
  	.4byte  0
  	.4byte  PendSV_Handler
  	.4byte  SysTick_Handler

  /* External Interrupts */
  	.4byte     WWDG_IRQHandler                   /* Window WatchDog              */
  	.4byte     PVD_IRQHandler                    /* PVD through EXTI Line detection */
  	.4byte     TAMP_STAMP_IRQHandler             /* Tamper and TimeStamps through the EXTI line */
  	.4byte     RTC_WKUP_IRQHandler               /* RTC Wakeup through the EXTI line */
  	.4byte     FLASH_IRQHandler                  /* FLASH                        */
  	.4byte     RCC_IRQHandler                    /* RCC                          */
  	.4byte     EXTI0_IRQHandler                  /* EXTI Line0                   */
  	.4byte     EXTI1_IRQHandler                  /* EXTI Line1                   */
  	.4byte     EXTI2_IRQHandler                  /* EXTI Line2                   */
  	.4byte     EXTI3_IRQHandler                  /* EXTI Line3                   */
  	.4byte     EXTI4_IRQHandler                  /* EXTI Line4                   */
  	.4byte     DMA1_Stream0_IRQHandler           /* DMA1 Stream 0                */
  	.4byte     DMA1_Stream1_IRQHandler           /* DMA1 Stream 1                */
  	.4byte     DMA1_Stream2_IRQHandler           /* DMA1 Stream 2                */
  	.4byte     DMA1_Stream3_IRQHandler           /* DMA1 Stream 3                */
  	.4byte     DMA1_Stream4_IRQHandler           /* DMA1 Stream 4                */
  	.4byte     DMA1_Stream5_IRQHandler           /* DMA1 Stream 5                */
  	.4byte     DMA1_Stream6_IRQHandler           /* DMA1 Stream 6                */
  	.4byte     ADC_IRQHandler                    /* ADC1, ADC2 and ADC3s         */
  	.4byte     CAN1_TX_IRQHandler                /* CAN1 TX                      */
  	.4byte     CAN1_RX0_IRQHandler               /* CAN1 RX0                     */
  	.4byte     CAN1_RX1_IRQHandler               /* CAN1 RX1                     */
  	.4byte     CAN1_SCE_IRQHandler               /* CAN1 SCE                     */
  	.4byte     EXTI9_5_IRQHandler                /* External Line[9:5]s          */
  	.4byte     TIM1_BRK_TIM9_IRQHandler          /* TIM1 Break and TIM9          */
  	.4byte     TIM1_UP_TIM10_IRQHandler          /* TIM1 Update and TIM10        */
  	.4byte     TIM1_TRG_COM_TIM11_IRQHandler     /* TIM1 Trigger and Commutation and TIM11 */
  	.4byte     TIM1_CC_IRQHandler                /* TIM1 Capture Compare         */
  	.4byte     TIM2_IRQHandler                   /* TIM2                         */
  	.4byte     TIM3_IRQHandler                   /* TIM3                         */
  	.4byte     TIM4_IRQHandler                   /* TIM4                         */
  	.4byte     I2C1_EV_IRQHandler                /* I2C1 Event                   */
  	.4byte     I2C1_ER_IRQHandler                /* I2C1 Error                   */
  	.4byte     I2C2_EV_IRQHandler                /* I2C2 Event                   */
  	.4byte     I2C2_ER_IRQHandler                /* I2C2 Error                   */
  	.4byte     SPI1_IRQHandler                   /* SPI1                         */
  	.4byte     SPI2_IRQHandler                   /* SPI2                         */
  	.4byte     USART1_IRQHandler                 /* USART1                       */
  	.4byte     USART2_IRQHandler                 /* USART2                       */
  	.4byte     USART3_IRQHandler                 /* USART3                       */
  	.4byte     EXTI15_10_IRQHandler              /* External Line[15:10]s        */
  	.4byte     RTC_Alarm_IRQHandler              /* RTC Alarm (A and B) through EXTI Line */
  	.4byte     OTG_FS_WKUP_IRQHandler            /* USB OTG FS Wakeup through EXTI line */
  	.4byte     TIM8_BRK_TIM12_IRQHandler         /* TIM8 Break and TIM12         */
  	.4byte     TIM8_UP_TIM13_IRQHandler          /* TIM8 Update and TIM13        */
  	.4byte     TIM8_TRG_COM_TIM14_IRQHandler     /* TIM8 Trigger and Commutation and TIM14 */
  	.4byte     TIM8_CC_IRQHandler                /* TIM8 Capture Compare         */
  	.4byte     DMA1_Stream7_IRQHandler           /* DMA1 Stream7                 */
  	.4byte     FSMC_IRQHandler                   /* FSMC                         */
  	.4byte     SDIO_IRQHandler                   /* SDIO                         */
  	.4byte     TIM5_IRQHandler                   /* TIM5                         */
  	.4byte     SPI3_IRQHandler                   /* SPI3                         */
  	.4byte     UART4_IRQHandler                  /* UART4                        */
  	.4byte     UART5_IRQHandler                  /* UART5                        */
  	.4byte     TIM6_DAC_IRQHandler               /* TIM6 and DAC1&2 underrun errors */
  	.4byte     TIM7_IRQHandler                   /* TIM7                         */
  	.4byte     DMA2_Stream0_IRQHandler           /* DMA2 Stream 0                */
  	.4byte     DMA2_Stream1_IRQHandler           /* DMA2 Stream 1                */
  	.4byte     DMA2_Stream2_IRQHandler           /* DMA2 Stream 2                */
  	.4byte     DMA2_Stream3_IRQHandler           /* DMA2 Stream 3                */
  	.4byte     DMA2_Stream4_IRQHandler           /* DMA2 Stream 4                */
  	.4byte     ETH_IRQHandler                    /* Ethernet                     */
  	.4byte     ETH_WKUP_IRQHandler               /* Ethernet Wakeup through EXTI line */
  	.4byte     CAN2_TX_IRQHandler                /* CAN2 TX                      */
  	.4byte     CAN2_RX0_IRQHandler               /* CAN2 RX0                     */
  	.4byte     CAN2_RX1_IRQHandler               /* CAN2 RX1                     */
  	.4byte     CAN2_SCE_IRQHandler               /* CAN2 SCE                     */
  	.4byte     OTG_FS_IRQHandler                 /* USB OTG FS                   */
  	.4byte     DMA2_Stream5_IRQHandler           /* DMA2 Stream 5                */
  	.4byte     DMA2_Stream6_IRQHandler           /* DMA2 Stream 6                */
  	.4byte     DMA2_Stream7_IRQHandler           /* DMA2 Stream 7                */
  	.4byte     USART6_IRQHandler                 /* USART6                       */
  	.4byte     I2C3_EV_IRQHandler                /* I2C3 event                   */
  	.4byte     I2C3_ER_IRQHandler                /* I2C3 error                   */
  	.4byte     OTG_HS_EP1_OUT_IRQHandler         /* USB OTG HS End Point 1 Out   */
  	.4byte     OTG_HS_EP1_IN_IRQHandler          /* USB OTG HS End Point 1 In    */
  	.4byte     OTG_HS_WKUP_IRQHandler            /* USB OTG HS Wakeup through EXTI */
  	.4byte     OTG_HS_IRQHandler                 /* USB OTG HS                   */
  	.4byte     DCMI_IRQHandler                   /* DCMI                         */
  	.4byte     CRYP_IRQHandler                   /* CRYP crypto                  */
  	.4byte     HASH_RNG_IRQHandler               /* Hash and Rng                 */
  	.4byte     FPU_IRQHandler                    /* FPU                          */

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

//add handlers for system exeptions(?)

.section .text.default_handler,"ax",%progbits
.balign 4
.thumb_func
	default_handler:
	b.n default_handler
