/**
 * FILE: main.s
 *
 * DESCRIPTION:
 * This file contains the assembly code for a simple load and store firmware
 * utilizing the STM32F401CC6 microcontroller.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: March 3, 2024
 * UPDATE DATE: March 31, 2024
 *
 * ASSEMBLE AND LINK w/ SYMBOLS:
 * 1. arm-none-eabi-as -g main.s -o main.o
 * 2. arm-none-eabi-ld main.o -o main.elf -T STM32F401CCUX_FLASH.ld
 * 3. openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program main.elf verify reset exit"
 * ASSEMBLE AND LINK w/o SYMBOLS:
 * 1. arm-none-eabi-as -g main.s -o main.o
 * 2. arm-none-eabi-ld main.o -o main.elf -T STM32F401CCUX_FLASH.ld
 * 3. arm-none-eabi-objcopy -O binary --strip-all main.elf main.bin
 * 3. openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program main.bin 0x08000000 verify reset exit"
 * DEBUG w/ SYMBOLS:
 * 1. openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
 * 2. arm-none-eabi-gdb main.elf
 * 3. target remote :3333
 * 4. monitor reset halt
 * 5. l
 * DEBUG w/o SYMBOLS:
 * 1. openocd -f interface/stlink.cfg -f target/stm32f4x.cfg
 * 2. arm-none-eabi-gdb main.bin
 * 3. target remote :3333
 * 4. monitor reset halt
 * 5. x/8i $pc
 */


.syntax unified
.cpu cortex-m4
.fpu softvfp
.thumb


/**
 * The start address for the .data section defined in linker script.
 */
.word _sdata

/**
 * The end address for the .data section defined in linker script.
 */
.word _edata

/**
 * The start address for the initialization values of the .data section defined in linker script.
 */
.word _sidata

/**
 * The start address for the .bss section defined in linker script.
 */
.word _sbss

/**
 * The end address for the .bss section defined in linker script.
 */
.word _ebss


/**
 * Provide weak aliases for each Exception handler to the Default_Handler. As they are weak aliases, any function
 * with the same name will override this definition.
 */
.macro weak name
  .global \name
  .weak \name
  .thumb_set \name, Default_Handler
  .word \name
.endm


/**
 * Initialize the .isr_vector section. The .isr_vector section contains vector table.
 */
.section .isr_vector, "a"

/**
 * The STM32F401CCUx vector table. Note that the proper constructs must be placed on this to ensure that it ends up
 * at physical address 0x00000000.
 */
.global isr_vector
.type isr_vector, %object
isr_vector:
  .word _estack
  .word Reset_Handler
   weak NMI_Handler
   weak HardFault_Handler
   weak MemManage_Handler
   weak BusFault_Handler
   weak UsageFault_Handler
  .word 0
  .word 0
  .word 0
  .word 0
   weak SVC_Handler
   weak DebugMon_Handler
  .word 0
   weak PendSV_Handler
   weak SysTick_Handler
  .word 0
   weak EXTI16_PVD_IRQHandler                              // EXTI Line 16 interrupt PVD through EXTI line detection 
   weak TAMP_STAMP_IRQHandler                              // Tamper and TimeStamp interrupts through the EXTI line
   weak EXTI22_RTC_WKUP_IRQHandler                         // EXTI Line 22 interrupt RTC Wakeup interrupt, EXTI line
   weak FLASH_IRQHandler                                   // FLASH global interrupt
   weak RCC_IRQHandler                                     // RCC global interrupt
   weak EXTI0_IRQHandler                                   // EXTI Line0 interrupt
   weak EXTI1_IRQHandler                                   // EXTI Line1 interrupt
   weak EXTI2_IRQHandler                                   // EXTI Line2 interrupt
   weak EXTI3_IRQHandler                                   // EXTI Line3 interrupt
   weak EXTI4_IRQHandler                                   // EXTI Line4 interrupt
   weak DMA1_Stream0_IRQHandler                            // DMA1 Stream0 global interrupt
   weak DMA1_Stream1_IRQHandler                            // DMA1 Stream1 global interrupt
   weak DMA1_Stream2_IRQHandler                            // DMA1 Stream2 global interrupt
   weak DMA1_Stream3_IRQHandler                            // DMA1 Stream3 global interrupt
   weak DMA1_Stream4_IRQHandler                            // DMA1 Stream4 global interrupt
   weak DMA1_Stream5_IRQHandler                            // DMA1 Stream5 global interrupt
   weak DMA1_Stream6_IRQHandler                            // DMA1 Stream6 global interrupt
   weak ADC_IRQHandler                                     // ADC1 global interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak EXTI9_5_IRQHandler                                 // EXTI Line[9:5] interrupts
   weak TIM1_BRK_TIM9_IRQHandle                            // TIM1 Break interrupt and TIM9 global interrupt
   weak TIM1_UP_TIM10_IRQHandler                           // TIM1 Update interrupt and TIM10 global interrupt
   weak TIM1_TRG_COM_TIM11_IRQHandler                      // TIM1 T/C interrupts, TIM11 global interrupt
   weak TIM1_CC_IRQHandler                                 // TIM1 Capture Compare interrupt
   weak TIM2_IRQHandler                                    // TIM2 global interrupt
   weak TIM3_IRQHandler                                    // TIM3 global interrupt
   weak TIM4_IRQHandler                                    // TIM4 global interrupt
   weak I2C1_EV_IRQHandler                                 // I2C1 event interrupt
   weak I2C1_ER_IRQHandler                                 // I2C1 error interrupt
   weak I2C2_EV_IRQHandler                                 // I2C2 event interrupt
   weak I2C2_ER_IRQHandler                                 // I2C2 error interrupt
   weak SPI1_IRQHandler                                    // SPI1 global interrupt
   weak SPI2_IRQHandler                                    // SPI2 global interrupt
   weak USART1_IRQHandler                                  // USART1 global interrupt
   weak USART2_IRQHandler                                  // USART2 global interrupt
  .word 0                                                  // reserved
   weak EXTI15_10_IRQHandler                               // EXTI Line[15:10] interrupts
   weak EXTI17_RTC_Alarm_IRQHandler                        // EXTI Line 17 interrupt / RTC Alarms (A and B) EXTI
   weak EXTI18_OTG_FS_WKUP_IRQHandler                      // EXTI Line 18 interrupt / USBUSB OTG FS Wakeup EXTI
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak DMA1_Stream7_IRQHandler                            // DMA1 Stream7 global interrupt
  .word 0                                                  // reserved
   weak SDIO_IRQHandler                                    // SDIO global interrupt
   weak TIM5_IRQHandler                                    // TIM5 global interrupt
   weak SPI3_IRQHandler                                    // SPI3 global interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak DMA2_Stream0_IRQHandler                            // DMA2 Stream0 global interrupt
   weak DMA2_Stream1_IRQHandler                            // DMA2 Stream1 global interrupt
   weak DMA2_Stream2_IRQHandler                            // DMA2 Stream2 global interrupt
   weak DMA2_Stream3_IRQHandler                            // DMA2 Stream3 global interrupt
   weak DMA2_Stream4_IRQHandler                            // DMA2 Stream4 global interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak OTG_FS_IRQHandler                                  // USB On The Go FS global interrupt
   weak DMA2_Stream5_IRQHandler                            // DMA2 Stream5 global interrupt
   weak DMA2_Stream6_IRQHandler                            // DMA2 Stream6 global interrupt
   weak DMA2_Stream7_IRQHandler                            // DMA2 Stream7 global interrupt
   weak USART6_IRQHandler                                  // USART6 global interrupt
   weak I2C3_EV_IRQHandler                                 // I2C3 event interrupt
   weak I2C3_ER_IRQHandler                                 // I2C3 error interrupt
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
  .word 0                                                  // reserved
   weak SPI4_IRQHandler                                    // SPI4 global interrupt

/**
 * @brief  This code is called when processor starts execution.
 *
 *         This is the code that gets called when the processor first starts execution following a reset event. We 
 *         first define and init the bss section and then define and init the data section, after which the 
 *         application supplied main routine is called.
 *
 * @param  None
 * @retval None
 */
.type Reset_Handler, %function
.global Reset_Handler
Reset_Handler:
  LDR   R4, =_estack                                       // load address at end of the stack into R0
  MOV   SP, R4                                             // move address at end of stack into SP
  LDR   R4, =_sdata                                        // copy the data segment initializers from flash to SRAM
  LDR   R5, =_edata                                        // copy the data segment initializers from flash to SRAM
  LDR   R6, =_sidata                                       // copy the data segment initializers from flash to SRAM
  MOVS  R7, #0                                             // copy the data segment initializers from flash to SRAM
  B     .Reset_Handler_Loop_Copy_Data_Init                 // branch
.Reset_Handler_Copy_Data_Init:
  LDR   R8, [R6, R7]                                       // copy the data segment initializers into registers
  STR   R8, [R4, R7]                                       // copy the data segment initializers into registers
  ADDS  R7, R7, #4                                         // copy the data segment initializers into registers
.Reset_Handler_Loop_Copy_Data_Init:
  ADDS  R8, R4, R7                                         // initialize the data segment
  CMP   R8, R5                                             // initialize the data segment
  BCC   .Reset_Handler_Copy_Data_Init                      // branch if carry is clear
  LDR   R6, =_sbss                                         // copy the bss segment initializers from flash to SRAM
  LDR   R8, =_ebss                                         // copy the bss segment initializers from flash to SRAM
  MOVS  R7, #0                                             // copy the bss segment initializers from flash to SRAM
  B     .Reset_Handler_Loop_Fill_Zero_BSS                  // branch
.Reset_Handler_Fill_Zero_BSS:
  STR   R7, [R6]                                           // zero fill the bss segment
  ADDS  R6, R6, #4                                         // zero fill the bss segment
.Reset_Handler_Loop_Fill_Zero_BSS:
  CMP   R6, R8                                             // zero fill the bss segment
  BCC   .Reset_Handler_Fill_Zero_BSS                       // branch if carry is clear
  BL    __start                                            // call function

/**
 * @brief  This code is called when the processor receives and unexpected interrupt.
 *
 *         This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop, preserving
 *         the system state for examination by a debugger.
 *
 * @param  None
 * @retval None
 */
.type Default_Handler, %function
.global Default_Handler
Default_Handler:
  BKPT                                                     // set processor into debug state
  B.N   Default_Handler                                    // call function, force thumb state


/**
 * Initialize the .text section.
 * The .text section contains executable code.
 */
.section .text

/**
 * @brief  Entry point for initialization and setup of specific functions.
 *
 *         This function is the entry point for initializing and setting up specific functions.
 *         It calls other functions to enable certain features and then enters a loop for further execution.
 *
 * @param  None
 * @retval None
 */
.type __start, %function
__start:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR  R4, =0x40023830                                     // load address of RCC_AHB1ENR register
  LDR  R5, [R4]                                            // load value inside RCC_AHB1ENR register
  ORR  R5, #(1<<0)                                         // set the GPIOAEN bit
  STR  R5, [R4]                                            // store value into RCC_AHB1ENR register
  B    .                                                   // branch infinite loop
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller
