/**
 * FILE: main.s
 *
 * DESCRIPTION:
 * This file contains the assembly code for interrupts and a STUXNET simulation
 * utilizing the STM32F401 Nucleo-64 microcontroller.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: September 1, 2023
 * UPDATE Date: April 1, 2024
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
  .word EXTI15_10_IRQHandler                               // EXTI Line[15:10] interrupts
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
  BL    main                                            // call function

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
 * @brief   This code is called when the interrupt handler for the EXTI lines 15 to 10
 *          is triggered.
 *
 * @details This is the interrupt handler function for EXTI lines 15 to 10. It is
 *          triggered when an interrupt request is received on any of these lines.
 *          The function checks if the interrupt was caused by line 13 (PR13 bit),
 *          and if so, it sets the corresponding bit in the EXTI_PR register to
 *          acknowledge the interrupt. After that, it calls the EXTI_Callback function.
 *
 * @param   None
 * @retval  None
 */
.section .text.EXTI15_10_IRQHandler
.weak EXTI15_10_IRQHandler
.type EXTI15_10_IRQHandler, %function
EXTI15_10_IRQHandler:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40013C14                                    // load the address of EXTI_PR register
  LDR   R5, [R4]                                           // load value inside EXTI_PR register
  TST   R5, #(1<<13)                                       // read the PR13 bit, if 0, then BEQ
  BEQ   .PR13_0                                            // branch if equal
  ORR   R5, #(1<<13)                                       // set the PR13 bit
  STR   R5, [R4]                                           // store value into EXTI_PR register
  BL    EXTI_Callback                                      // call function
.PR13_0:
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller


/**
 * Initialize the .text section.
 * The .text section contains executable code.
 */
.section .text

/**
 * @brief   Entry point for initialization and setup of specific functions.
 *
 * @details This function is the entry point for initializing and setting up specific functions.
 *          It calls other functions to enable certain features and then enters a loop for further execution.
 *
 * @param   None
 * @retval  None
 */
main:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  BL    GPIOA_Enable                                       // call function
  BL    GPIOC_Enable                                       // call function
  BL    GPIOA_PA9_Alt_Function_Mode_Enable                 // call function
  BL    GPIOA_PA10_Alt_Function_Mode_Enable                // call function
  BL    GPIOC_PC0_General_Purpose_Output_Mode_Enable       // call function
  BL    GPIOC_PC1_General_Purpose_Output_Mode_Enable       // call function
  BL    GPIOC_PC2_General_Purpose_Output_Mode_Enable       // call function
  BL    GPIOC_PC3_General_Purpose_Output_Mode_Enable       // call function
  BL    USART1_Enable                                      // call function
  BL    GPIOC_PC13_EXTI_Init                               // call function
  B     .                                                  // branch infinite loop
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables the GPIOA peripheral by setting the corresponding RCC_AHB1ENR bit.
 *
 * @details This function enables the GPIOA peripheral by setting the corresponding
 *          RCC_AHB1ENR bit.  It loads the address of the RCC_AHB1ENR register, retrieves
 *          the current value of the register, sets the GPIOAEN bit, and stores the
 *          updated value back into the register.
 *
 * @param   None
 * @retval  None
 */
GPIOA_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40023830                                    // load address of RCC_AHB1ENR register
  LDR   R5, [R4]                                           // load value inside RCC_AHB1ENR register
  ORR   R5, #(1<<0)                                        // set the GPIOAEN bit
  STR   R5, [R4]                                           // store value into RCC_AHB1ENR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables the GPIOC peripheral by setting the corresponding RCC_AHB1ENR bit.
 *
 * @details This function enables the GPIOC peripheral by setting the corresponding
 *          RCC_AHB1ENR bit.  It loads the address of the RCC_AHB1ENR register, retrieves
 *          the current value of the register, sets the GPIOCEN bit, and stores the
 *          updated value back into the register.
 *
 * @param   None
 * @retval  None
 */
GPIOC_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40023830                                    // load address of RCC_AHB1ENR register
  LDR   R5, [R4]                                           // load value inside RCC_AHB1ENR register
  ORR   R5, #(1<<2)                                        // set the GPIOCEN bit
  STR   R5, [R4]                                           // store value into RCC_AHB1ENR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOA pin 9 to operate in alternate function mode.
 *
 * @details This function configures GPIOA pin 9 to operate in alternate function mode.
 *          It modifies the GPIOA_MODER and GPIOA_AFRH registers to set the necessary bits
 *          for alternate function mode on pin 9. The MODER9 bit is set to select alternate
 *          function mode, and the AFRH9 bits are set to specify the desired alternate function.
 *
 * @param   None
 * @retval  None
 */
GPIOA_PA9_Alt_Function_Mode_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020000                                    // load address of GPIOA_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOA_MODER register
  ORR   R5, #(1<<19)                                       // set the MODER9 bit
  AND   R5, #~(1<<18)                                      // clear the MODER9 bit
  STR   R5, [R4]                                           // store value into GPIOA_MODER register
  LDR   R4, =0x40020024                                    // load address of GPIOA_AFRH register
  LDR   R5, [R4]                                           // load value inside GPIOA_AFRH register
  AND   R5, #~(1<<7)                                       // clear the AFRH9 bit
  ORR   R5, #(1<<6)                                        // set the AFRH9 bit
  ORR   R5, #(1<<5)                                        // set the AFRH9 bit
  ORR   R5, #(1<<4)                                        // set the AFRH9 bit
  STR   R5, [R4]                                           // store value into GPIOA_AFRH register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOA pin 10 to operate in alternate function mode.
 *
 * @details This function configures GPIOA pin 10 to operate in alternate function mode.
 *          It modifies the GPIOA_MODER and GPIOA_AFRH registers to set the necessary bits
 *          for alternate function mode on pin 10. The MODER10 bit is set to select alternate
 *          function mode, and the AFRH9 bits are set to specify the desired alternate function.
 *
 * @param   None
 * @retval  None
 */
GPIOA_PA10_Alt_Function_Mode_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020000                                    // load address of GPIOA_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOA_MODER register
  ORR   R5, #(1<<21)                                       // set the MODER10 bit
  AND   R5, #~(1<<20)                                      // clear the MODER10 bit
  STR   R5, [R4]                                           // store value into GPIOA_MODER register
  LDR   R4, =0x40020024                                    // load address of GPIOA_AFRH register
  LDR   R5, [R4]                                           // load value inside GPIOA_AFRH register
  AND   R5, #~(1<<11)                                      // clear the AFRH10 bit
  ORR   R5, #(1<<10)                                       // set the AFRH10 bit
  ORR   R5, #(1<<9)                                        // set the AFRH10 bit
  ORR   R5, #(1<<8)                                        // set the AFRH10 bit
  STR   R5, [R4]                                           // store value into GPIOA_AFRH register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOC pin 0 to operate in general purpose output mode.
 *
 * @details This function configures GPIOC pin 0 to operate in general purpose output mode.
 *          It modifies the GPIOC_MODER to set the necessary bits for general purpose output mode 
 *          on pin 0. The MODER0 bit is set to select general purpose output mode.
 *
 * @param   None
 * @retval  None
 */
GPIOC_PC0_General_Purpose_Output_Mode_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020800                                    // load address of GPIOC_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOC_MODER register
  AND   R5, #~(1<<1)                                       // clear the MODER0 bit
  ORR   R5, #(1<<0)                                        // set the MODER0 bit
  STR   R5, [R4]                                           // store value into GPIOC_MODER register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOC pin 1 to operate in general purpose output mode.
 *
 * @details This function configures GPIOC pin 1 to operate in general purpose output mode.
 *          It modifies the GPIOC_MODER to set the necessary bits for general purpose output mode 
 *          on pin 1. The MODER1 bit is set to select general purpose output mode.
 *
 * @param   None
 * @retval  None
 */
GPIOC_PC1_General_Purpose_Output_Mode_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020800                                    // load address of GPIOC_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOC_MODER register
  AND   R5, #~(1<<3)                                       // clear the MODER1 bit
  ORR   R5, #(1<<2)                                        // set the MODER1 bit
  STR   R5, [R4]                                           // store value into GPIOC_MODER register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOC pin 2 to operate in general purpose output mode.
 *
 * @details This function configures GPIOC pin 2 to operate in general purpose output mode.
 *          It modifies the GPIAC_MODER to set the necessary bits for general purpose output mode 
 *          on pin 2. The MODER2 bit is set to select general purpose output mode.
 *
 * @param   None
 * @retval  None
 */
GPIOC_PC2_General_Purpose_Output_Mode_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020800                                    // load address of GPIOC_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOC_MODER register
  AND   R5, #~(1<<5)                                       // clear the MODER2 bit
  ORR   R5, #(1<<4)                                        // set the MODER2 bit
  STR   R5, [R4]                                           // store value into GPIOC_MODER register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIOC pin 3 to operate in general purpose output mode.
 *
 * @details This function configures GPIOC pin 3 to operate in general purpose output mode.
 *          It modifies the GPIOC_MODER to set the necessary bits for general purpose output mode 
 *          on pin 3. The MODER3 bit is set to select general purpose output mode.
 *
 * @param   None
 * @retval  None
 */
GPIOC_PC3_General_Purpose_Output_Mode_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020800                                    // load address of GPIOC_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOC_MODER register
  AND   R5, #~(1<<7)                                       // clear the MODER3 bit
  ORR   R5, #(1<<6)                                        // set the MODER3 bit
  STR   R5, [R4]                                           // store value into GPIOC_MODER register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enables USART1 peripheral and configures its settings for communication.
 *
 * @details This function enables the USART1 peripheral by setting the corresponding
 *          RCC_APB2ENR bit.  It also configures the USART1 settings, including the baud
 *          rate and control register settings.  The USART1_BRR register is set to achieve
 *          a baud rate of 9600, and the USART1_CR1 register is modified to enable USART1
 *          (UE bit) and enable transmission (TE bit).
 *
 * @param   None
 * @retval  None
 */
USART1_Enable:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40023844                                    // load address of RCC_APB2ENR register
  LDR   R5, [R4]                                           // load value inside RCC_APB2ENR register
  ORR   R5, #(1<<4)                                        // set the USART1EN bit
  STR   R5, [R4]                                           // store value into RCC_AHB1ENR register
  LDR   R4, =0x40011008                                    // load address of USART1_BRR register 
  LDR   R5, [R4]                                           // load value inside USART1_BRR register
  MOV   R5, #0x683                                         // set register to 9600 baud
  STR   R5, [R4]                                           // store value into USART1_BRR register
  LDR   R4, =0x4001100C                                    // load address of USART1_CR1 register
  LDR   R5, [R4]                                           // load value inside USART1_CR1 register
  ORR   R5, #(1<<13)                                       // set the UE bit
  ORR   R5, #(1<<3)                                        // set the TE bit
  ORR   R5, #(1<<2)                                        // set the RE bit
  STR   R5, [R4]                                           // store value into USART1_CR1 register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Sends a single character over USART1.
 *
 * @details This function sends a single character over USART1 by writing it to the USART1_DR
 *          register.  It first checks if the transmit buffer is empty (TXE bit) in the
 *          USART1_SR register.  If the buffer is not empty, it waits until it becomes empty
 *          before writing the character to USART1_DR.
 *
 * @param   R0: The character to be sent over USART1.
 * @retval  None
 */
USART1_Transmit_Character:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40011000                                    // load address of USART1_SR register
.USART1_Transmit_Character_Loop:
  LDR   R5, [R4]                                           // load value inside USART1_SR register
  AND   R5, #(1<<7)                                        // read TXE bit
  CMP   R5, #0x00                                          // test TX FIFO is not full
  BEQ   .USART1_Transmit_Character_Loop                    // branch if equal
  LDR   R4, =0x40011004                                    // load value inside USART1_DR register
  STR   R0, [R4]                                           // store value into USART1_DR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Receives a character over USART1.
 *
 * @details This function receives a character over USART1 by reading the USART1_DR register.
 *          It first checks if the receive buffer is not empty (RXNE bit) in the USART1_SR
 *          register.  If the buffer is empty, it waits until it becomes non-empty before
 *          reading the character from the USART1_DR register. The received character is then
 *          returned.
 *
 * @param   None
 * @retval  R0: The received character over USART1.
 */
USART1_Receive_Character:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40011000                                    // load address of USART1_SR register
.USART1_Receive_Character_Loop:
  LDR   R5, [R4]                                           // load value inside USART1_SR register
  AND   R5, #(1<<5)                                        // read the RXNE bit
  CMP   R5, #0x00                                          // test TX FIFO is not full
  BEQ   .USART1_Receive_Character_Loop                     // branch if equal
  LDR   R4, =0x40011004                                    // load value inside USART1_DR register
  LDR   R0, [R4]                                           // read value inside USART1_DR register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Configures GPIO pins for clockwise full drive sequence mode.
 *
 * @details In full drive sequence mode, two coils are energized at a time, providing full torque to
 *          the stepper motor. This function configures the GPIO pins to control a UNL2003 driver
 *          for clockwise full drive sequence mode operation.
 *
 * @param   R0: The millisecond delay value.
 * @retval  None
 */
Clockwise_Rotation_Sequence:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0x40020814                                    // load address of GPIOC_ODR register
  LDR   R5, [R4]                                           // load value inside GPIOC_ODR register
  MOV   R5, #0x08                                          // set the ODR register
  STR   R5, [R4]                                           // store value into GPIOC_ODR register
  BL    Delay_MS                                           // call function
  LDR   R4, =0x40020814                                    // load address of GPIOC_ODR register
  LDR   R5, [R4]                                           // load value inside GPIOC_ODR register
  MOV   R5, #0x04                                          // set the ODR register
  STR   R5, [R4]                                           // store value into GPIOC_ODR register
  BL    Delay_MS                                           // call function
  LDR   R4, =0x40020814                                    // load address of GPIOC_ODR register
  LDR   R5, [R4]                                           // load value inside GPIOC_ODR register
  MOV   R5, #0x02                                          // set the ODR register
  STR   R5, [R4]                                           // store value into GPIOC_ODR register
  BL    Delay_MS                                           // call function
  LDR   R5, =0x40020814                                    // load address of GPIOC_ODR register
  LDR   R5, [R4]                                           // load value inside GPIOC_ODR register
  MOV   R5, #0x01                                          // set the ODR register
  STR   R5, [R4]                                           // store value into GPIOC_ODR register
  BL    Delay_MS                                           // call function
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Initializes GPIOC PC13 for EXTI interrupt.
 *
 * @details This function configures GPIOC PC13 for EXTI interrupt. It sets the pin's mode
 *          to input and enables the internal pull-up resistor. Additionally, it enables the
 *          EXTI interrupt for PC13, configures SYSCFG_EXTICR4, and sets the corresponding
 *          EXTI and NVIC settings to enable interrupt handling for PC13.
 * 
 * @param   None
 * @retval  None
 */
GPIOC_PC13_EXTI_Init:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  CPSID I                                                  // disable global interrupts
  LDR   R4, =0x40020800                                    // load address of GPIOC_MODER register
  LDR   R5, [R4]                                           // load value inside GPIOC_MODER register
  AND   R5, #~(1<<27)                                      // clear the MODER13 bit
  AND   R5, #~(1<<26)                                      // clear the MODER13 bit
  STR   R5, [R4]                                           // store value into GPIOC_MODER register
  LDR   R4, =0x4002080C                                    // load address of GPIOC_PUPDR register
  LDR   R5, [R4]                                           // load value inside GPIOC_PUPDR register
  AND   R5, #~(1<<27)                                      // clear the PUPDR13 bit
  ORR   R5, #(1<<26)                                       // set the PUPDR13 bit
  STR   R5, [R4]                                           // store value into GPIOC_PUPDR register
  LDR   R4, =0x40023844                                    // load address of RCC_ABP2ENR
  LDR   R5, [R4]                                           // load value inside RCC_ABP2ENR register
  ORR   R5, #(1<<14)                                       // set SYSCFGEN bit
  STR   R5, [R4]                                           // store value into RCC_APB2ENR register
  LDR   R4, =0x40013814                                    // load address of SYSCFG_EXTICR4
  LDR   R5, [R4]                                           // load value inside SYSCFG_EXTICR4 register
  ORR   R5, #(1<<5)                                        // set EXTI13 bit
  STR   R5, [R4]                                           // store value into SYSCFG_EXTICR4 register
  LDR   R4, =0x40013C00                                    // load address of EXTI_IMR register
  LDR   R5, [R4]                                           // load value inside EXTI_IMR register
  ORR   R5, #(1<<13)                                       // set MR13 bit
  STR   R5, [R4]                                           // store value into EXTI_IMR register
  LDR   R4, =0x40013C0C                                    // load address of EXTI_FTSR register
  LDR   R5, [R4]                                           // load value inside EXTI_FTSR register
  ORR   R5, #(1<<13)                                       // set TR13 bit
  STR   R5, [R4]                                           // store value into EXTI_IMR register
  BL    NVIC_EnableIRQ_EXTI15_10                           // call function
  CPSIE I                                                  // enable global interrupts
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   EXTI callback function for centrifuge control.
 *
 * @details This EXTI callback function simulates centrifuge control and communication.
 *          It includes a loop that mimics a sensor reading delay and sends appropriate
 *          characters over USART1 based on the sensor's value. If the sensor value is
 *          within the normal range, it sends "NORMAL" and if it's high, it sends "HIGH".
 *          The function also checks for a kill switch condition and finishes if engaged.
 *
 * @param   None
 * @retval  None
 */
EXTI_Callback:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
.EXTI_Callback_Loop:
  MOV   R0, #0x40                                          // 64 ms delay variable mock sensor read speed, 8 ms would damage centrifuge
  BL    Clockwise_Rotation_Sequence                        // call function
  CMP   R0, #0x40                                          // compare speed to normal value
  BNE   .EXTI_Callback_Loop_High_Value                     // branch if not equal
  BLE   .EXTI_Callback_Loop_Normal_Value                   // branch if less than or equal
.EXTI_Callback_Loop_Normal_Value:
  MOV   R0, #0x4E                                          // 'N'
  BL    USART1_Transmit_Character                          // call function
  MOV   R0, #0x4F                                          // 'O'
  BL    USART1_Transmit_Character                          // call function
  MOV   R0, #0x52                                          // 'R'
  BL    USART1_Transmit_Character                          // call function
  MOV   R0, #0x4D                                          // 'M'
  BL    USART1_Transmit_Character                          // call function
  MOV   R0, #0x41                                          // 'A'
  BL    USART1_Transmit_Character                          // call function
  MOV   R0, #0x4C                                          // 'L'
  BL    USART1_Transmit_Character                          // call function
  MOV   R0, #0x0D                                          // '\r'
  BL    USART1_Transmit_Character                          // call function
  MOV   R0, #0x0A                                          // '\n'
  BL    USART1_Transmit_Character                          // call function
  BL    Kill_Switch                                        // call function
  CMP   R0, #0x01                                          // compare if kill switch was engaged
  BEQ   .EXTI_Callback_Finish                              // branch if equal
  B     .EXTI_Callback_Loop                                // unconditional branch
.EXTI_Callback_Loop_High_Value:
  MOV   R0, #0x48                                          // 'H'
  BL    USART1_Transmit_Character                          // call function
  MOV   R0, #0x49                                          // 'I'
  BL    USART1_Transmit_Character                          // call function
  MOV   R0, #0x47                                          // 'G'
  BL    USART1_Transmit_Character                          // call function
  MOV   R0, #0x48                                          // 'H'
  BL    USART1_Transmit_Character                          // call function
  MOV   R0, #0x0D                                          // '\r'
  BL    USART1_Transmit_Character                          // call function
  MOV   R0, #0x0A                                          // '\n'
  BL    USART1_Transmit_Character                          // call function
  BL    Kill_Switch                                        // call function
  CMP   R0, #0x01                                          // compare if kill switch was engaged
  BEQ   .EXTI_Callback_Finish                              // branch if equal
  B     .EXTI_Callback_Loop                                // unconditional branch
.EXTI_Callback_Finish:
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Kill switch handler function.
 *
 * @details This function checks for a specific character received over USART1, and if it
 *          matches '1', it sets a return value in R7 to indicate that the centrifuge should
 *          be stopped. This function is used to implement a kill switch functionality to
 *          stop the centrifuge from spinning.
 *
 * @param   None
 * @retval  R0: A return value indicating whether the centrifuge should be stopped (0x01) or not.
 */
Kill_Switch:      
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack                                         
  LDR   R4, =0x40011004                                    // load value inside USART1_DR register
  LDR   R4, [R4]                                           // read value inside USART1_DR register
  CMP   R4, #0x31                                          // compare received character with '1'
  BNE   .Kill_Switch_Finish                                // branch if not equal
  MOV   R0, #1                                             // return value to kill the centrifuge from spinning
.Kill_Switch_Finish:
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Delay function in milliseconds.
 *
 * @details This function provides a software-based delay in milliseconds. It takes an
 *          argument in R7, representing the number of milliseconds to delay. The function
 *          uses nested loops to create the delay, where the inner loop accounts for the
 *          approximate execution time of 1 millisecond at a 16 MHz clock frequency.
 *
 * @param   R0: The number of milliseconds to delay.
 * @retval  None
 */
Delay_MS:
  PUSH  {R0, R4-R12, LR}                                   // push registers R0, R4-R12, LR to the stack
.Delay_MS_Outer_Loop:
  CMP   R0, #0                                             // compare R0 to 0x00
  BLE   .Delay_MS_Exit                                     // branch if less than or equal to
  MOV   R4, #0xA28                                         // move 1 ms value (at 16 MHz clock) into R2
.Delay_MS_Inner_Loop: 
  SUBS  R4, #1                                             // decrement the inner loop counter
  BNE   .Delay_MS_Inner_Loop                               // branch if not equal
  SUBS  R0, #1                                             // decrement the outer loop counter
  B     .Delay_MS_Outer_Loop                               // branch
.Delay_MS_Exit:
  POP   {R0, R4-R12, LR}                                   // pop registers R0, R4-R12, LR from the stack
  BX    LR                                                 // return to caller

/**
 * @brief   Enable NVIC (Nested Vectored Interrupt Controller) for EXTI15_10 interrupts.
 *
 * @details This function enables the NVIC for EXTI15_10 interrupts. It specifically targets
 *          the NVIC_ISER1 register, which controls interrupts 32 to 63, and sets the bit
 *          corresponding to EXTI15_10 (bit 8) to enable the interrupt handling for EXTI lines
 *          15 to 10.
 *
 * @param   None
 * @retval  None
 */
NVIC_EnableIRQ_EXTI15_10:
  PUSH  {R4-R12, LR}                                       // push registers R4-R12, LR to the stack
  LDR   R4, =0xE000E104                                    // NVIC_ISER1, p 683 M7 Arch ref manual, ISER1 interrupts 32-63
  LDR   R5, [R4]                                           // load value inside NVIC_ISER1 register
  ORR   R5, #(1<<8)                                        // 0x100=EXTI15-10 (p 204 Ref Manual), p 210 M4 Programming manual, ISER1 8 is 40
  STR   R5, [R4]                                           // store value into NVIC_ISER1 register
  POP   {R4-R12, LR}                                       // pop registers R4-R12, LR from the stack
  BX    LR                                                 // return to caller
