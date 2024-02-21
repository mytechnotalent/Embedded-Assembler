/**
 * FILE: main.s
 *
 * DESCRIPTION:
 * This file contains the assembly code for a basic Assembler primer
 * utilizing the STM32F401 Nucleo-64 microcontroller.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: September 5, 2023
 * UPDATE Date: September 5, 2023
 *
 * ASSEMBLE AND LINK w/ SYMBOLS:
 * 1. arm-none-eabi-as -g main.s -o main.o
 * 2. arm-none-eabi-ld main.o -o main.elf -T stm32f401ccux.ld
 * 3. openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program main.elf verify reset exit"
 * ASSEMBLE AND LINK w/o SYMBOLS:
 * 1. arm-none-eabi-as -g main.s -o main.o
 * 2. arm-none-eabi-ld main.o -o main.elf -T stm32f401ccux.ld
 * 3. arm-none-eabi-objcopy -O binary --strip-all main.elf main.bin
 * 3. openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program main.bin 0x08000000 verify reset exit"
 * DEBUG w/ SYMBOLS:
 * 1. openocd -f board/stm32f4x.cfg
 * 2. arm-none-eabi-gdb main.elf
 * 3. target remote :3333
 * 4. monitor reset halt
 * 5. l
 * DEBUG w/o SYMBOLS:
 * 1. openocd -f board/stm32f4x.cfg
 * 2. arm-none-eabi-gdb main.bin
 * 3. target remote :3333
 * 4. monitor reset halt
 * 5. x/8i $pc
 */

.syntax unified
.cpu cortex-m4
.thumb

/**
 * Provide weak aliases for each Exception handler to the Default_Handler.
 * As they are weak aliases, any function with the same name will override
 * this definition.
 */
.macro weak name
  .global \name
  .weak \name
  .thumb_set \name, Default_Handler
  .word \name
.endm

/**
 * The STM32F401CCUx vector table.  Note that the proper constructs
 * must be placed on this to ensure that it ends up at physical address
 * 0x00000000.
 */
.global isr_vector 
.section .isr_vector, "a"
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
   weak EXTI16_PVD_IRQHandler                              // EXTI Line 16 interrupt /PVD through EXTI line detection interrupt
   weak TAMP_STAMP_IRQHandler                              // Tamper and TimeStamp interrupts through the EXTI line
   weak EXTI22_RTC_WKUP_IRQHandler                         // EXTI Line 22 interrupt /RTC Wakeup interrupt through the EXTI line
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
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
   weak EXTI9_5_IRQHandler                                 // EXTI Line[9:5] interrupts
   weak TIM1_BRK_TIM9_IRQHandle                            // TIM1 Break interrupt and TIM9 global interrupt
   weak TIM1_UP_TIM10_IRQHandler                           // TIM1 Update interrupt and TIM10 global interrupt
   weak TIM1_TRG_COM_TIM11_IRQHandler                      // TIM1 Trigger and Commutation interrupts and TIM11 global interrupt
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
  .word USART1_IRQHandler                                  // USART1 global interrupt
   weak USART2_IRQHandler                                  // USART2 global interrupt
  .word 0                                                  // Reserved
  .word EXTI15_10_IRQHandler                               // EXTI Line[15:10] interrupts
   weak EXTI17_RTC_Alarm_IRQHandler                        // EXTI Line 17 interrupt / RTC Alarms (A and B) through EXTI line interrupt
   weak EXTI18_OTG_FS_WKUP_IRQHandler                      // EXTI Line 18 interrupt / USBUSB OTG FS Wakeup through EXTI line interrupt
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
   weak DMA1_Stream7_IRQHandler                            // DMA1 Stream7 global interrupt
  .word 0                                                  // Reserved
   weak SDIO_IRQHandler                                    // SDIO global interrupt
   weak TIM5_IRQHandler                                    // TIM5 global interrupt
   weak SPI3_IRQHandler                                    // SPI3 global interrupt
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
   weak DMA2_Stream0_IRQHandler                            // DMA2 Stream0 global interrupt
   weak DMA2_Stream1_IRQHandler                            // DMA2 Stream1 global interrupt
   weak DMA2_Stream2_IRQHandler                            // DMA2 Stream2 global interrupt
   weak DMA2_Stream3_IRQHandler                            // DMA2 Stream3 global interrupt
   weak DMA2_Stream4_IRQHandler                            // DMA2 Stream4 global interrupt
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
   weak OTG_FS_IRQHandler                                  // USB On The Go FS global interrupt
   weak DMA2_Stream5_IRQHandler                            // DMA2 Stream5 global interrupt
   weak DMA2_Stream6_IRQHandler                            // DMA2 Stream6 global interrupt
   weak DMA2_Stream7_IRQHandler                            // DMA2 Stream7 global interrupt
   weak USART6_IRQHandler                                  // USART6 global interrupt
   weak I2C3_EV_IRQHandler                                 // I2C3 event interrupt
   weak I2C3_ER_IRQHandler                                 // I2C3 error interrupt
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
  .word 0                                                  // Reserved
   weak SPI4_IRQHandler                                    // SPI4 global interrupt

.section .text

/**
 * @brief   This code is called when processor starts execution.
 *
 * @details This is the code that gets called when the processor first
 *          starts execution following a reset event. Only the absolutely
 *          necessary set is performed, after which the application
 *          supplied __start routine is called.
 * 
 * @param   None
 * @retval  None
 */
.type Reset_Handler, %function
.global Reset_Handler
Reset_Handler:
  LDR   R0, =_estack                                       // load address at end of the stack into R0
  MOV   SP, R0                                             // move address at end of stack into SP
  BL    __start                                            // call function

/**
 * @brief   This code is called when the processor receives and unexpected interrupt.
 *
 * @details This is the code that gets called when the processor receives an
 *          unexpected interrupt.  This simply enters an infinite loop, preserving
 *          the system state for examination by a debugger.
 *
 * @param   None
 * @retval  None
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
  PUSH  {LR}                                               // push return address onto stack
  LDR   R0, =0x40013C14                                    // load the address of EXTI_PR register
  LDR   R1, [R0]                                           // load value inside EXTI_PR register
  TST   R1, #(1<<13)                                       // read the PR13 bit, if 0, then BEQ
  BEQ   .PR13_0                                            // branch if equal
  ORR   R1, #(1<<13)                                       // set the PR13 bit
  STR   R1, [R0]                                           // store value inside R1 into R0
  BL    EXTI_Callback                                      // call function
.PR13_0:
  POP   {LR}                                               // pop return address from stack                                         
  BX    LR                                                 // return to caller

/**
 * @brief   USART1 interrupt handler.
 *
 * @details This is the interrupt handler function for USART1 interrupts. It is called when
 *          USART1 generates an interrupt request. Inside this handler, it calls the
 *          `USART1_Callback` function to handle the USART1 interrupt, and then returns to
 *          the main program execution.
 *
 * @param   None
 * @retval  None
 */
.section .text.USART1_IRQHandler
.weak USART1_IRQHandler
.type USART1_IRQHandler, %function
USART1_IRQHandler:
  PUSH  {LR}                                               // push return address onto stack
  LDR   R0, =0x40011000                                    // load address of USART1_SR register
  LDR   R1, [R0]                                           // load value inside USART1_SR register
  TST   R1, #(1<<5)                                        // read the RXNE register
  BL    USART1_Callback                                    // call function
  POP   {LR}                                               // pop return address from stack    
  BX    LR                                                 // return to caller

/**
 * @brief   Entry point for initialization and setup of specific functions.
 *
 * @details This function is the entry point for initializing and setting up specific functions.
 *          It calls other functions to enable certain features and then enters a loop for further execution.
 *
 * @param   None
 * @retval  None
 */
__start:
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
  BL    USART1_Int_Init                                    // call function
loop:
  MOV   R7, #0x42                                          // 'B'
  BL    USART1_Transmit_Character                          // call function
  BL    MOV_Immediate                                      // call function
  BL    MOVS_Register                                      // call function
  BL    LDR_Immediate                                      // call function
  BL    LDR_Offset                                         // call function
  BL    MOVW_MOVT                                          // call function
  BL    Flags                                              // call function
  BL    Bit_Set_Clear                                      // call function
  BL    Conditional_Branching                              // call function
  BL    Loops                                              // call function
  BL    Interrupts                                         // call function
  B     loop                                               // branch infinite loop

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
  LDR   R0, =0x40023830                                    // load address of RCC_AHB1ENR register
  LDR   R1, [R0]                                           // load value inside RCC_AHB1ENR register
  ORR   R1, #(1<<0)                                        // set the GPIOAEN bit
  STR   R1, [R0]                                           // store value into RCC_AHB1ENR register
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
  LDR   R0, =0x40023830                                    // load address of RCC_AHB1ENR register
  LDR   R1, [R0]                                           // load value inside RCC_AHB1ENR register
  ORR   R1, #(1<<2)                                        // set the GPIOCEN bit
  STR   R1, [R0]                                           // store value into RCC_AHB1ENR register
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
  LDR   R0, =0x40020000                                    // load address of GPIOA_MODER register
  LDR   R1, [R0]                                           // load value inside GPIOA_MODER register
  ORR   R1, #(1<<19)                                       // set the MODER9 bit
  AND   R1, #~(1<<18)                                      // clear the MODER9 bit
  STR   R1, [R0]                                           // store value into GPIOA_MODER register
  LDR   R0, =0x40020024                                    // load address of GPIOA_AFRH register
  LDR   R1, [R0]                                           // load value inside GPIOA_AFRH register
  AND   R1, #~(1<<7)                                       // clear the AFRH9 bit
  ORR   R1, #(1<<6)                                        // set the AFRH9 bit
  ORR   R1, #(1<<5)                                        // set the AFRH9 bit
  ORR   R1, #(1<<4)                                        // set the AFRH9 bit
  STR   R1, [R0]                                           // store value into GPIOA_AFRH register
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
  LDR   R0, =0x40020000                                    // load address of GPIOA_MODER register
  LDR   R1, [R0]                                           // load value inside GPIOA_MODER register
  ORR   R1, #(1<<21)                                       // set the MODER10 bit
  AND   R1, #~(1<<20)                                      // clear the MODER10 bit
  STR   R1, [R0]                                           // store value into GPIOA_MODER register
  LDR   R0, =0x40020024                                    // load address of GPIOA_AFRH register
  LDR   R1, [R0]                                           // load value inside GPIOA_AFRH register
  AND   R1, #~(1<<11)                                      // clear the AFRH10 bit
  ORR   R1, #(1<<10)                                       // set the AFRH10 bit
  ORR   R1, #(1<<9)                                        // set the AFRH10 bit
  ORR   R1, #(1<<8)                                        // set the AFRH10 bit
  STR   R1, [R0]                                           // store value into GPIOA_AFRH register
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
  LDR   R0, =0x40020800                                    // load address of GPIOC_MODER register
  LDR   R1, [R0]                                           // load value inside GPIOC_MODER register
  AND   R1, #~(1<<1)                                       // clear the MODER0 bit
  ORR   R1, #(1<<0)                                        // set the MODER0 bit
  STR   R1, [R0]                                           // store value into GPIOC_MODER register
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
  LDR   R0, =0x40020800                                    // load address of GPIOC_MODER register
  LDR   R1, [R0]                                           // load value inside GPIOC_MODER register
  AND   R1, #~(1<<3)                                       // clear the MODER1 bit
  ORR   R1, #(1<<2)                                        // set the MODER1 bit
  STR   R1, [R0]                                           // store value into GPIOC_MODER register
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
  LDR   R0, =0x40020800                                    // load address of GPIOC_MODER register
  LDR   R1, [R0]                                           // load value inside GPIOC_MODER register
  AND   R1, #~(1<<5)                                       // clear the MODER2 bit
  ORR   R1, #(1<<4)                                        // set the MODER2 bit
  STR   R1, [R0]                                           // store value into GPIOC_MODER register
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
  LDR   R0, =0x40020800                                    // load address of GPIOC_MODER register
  LDR   R1, [R0]                                           // load value inside GPIOC_MODER register
  AND   R1, #~(1<<7)                                       // clear the MODER3 bit
  ORR   R1, #(1<<6)                                        // set the MODER3 bit
  STR   R1, [R0]                                           // store value into GPIOC_MODER register
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
  LDR   R0, =0x40023844                                    // load address of RCC_APB2ENR register
  LDR   R1, [R0]                                           // load value inside RCC_APB2ENR register
  ORR   R1, #(1<<4)                                        // set the USART1EN bit
  STR   R1, [R0]                                           // store value into RCC_AHB1ENR register
  LDR   R0, =0x40011008                                    // load address of USART1_BRR register 
  LDR   R1, [R0]                                           // load value inside USART1_BRR register
  MOV   R1, #0x683                                         // set register to 9600 baud
  STR   R1, [R0]                                           // store value into USART1_BRR register
  LDR   R0, =0x4001100C                                    // load address of USART1_CR1 register
  LDR   R1, [R0]                                           // load value inside USART1_CR1 register
  ORR   R1, #(1<<13)                                       // set the UE bit
  ORR   R1, #(1<<5)                                        // set the RXNEIE bit
  ORR   R1, #(1<<3)                                        // set the TE bit
  ORR   R1, #(1<<2)                                        // set the RE bit
  STR   R1, [R0]                                           // store value into USART1_CR1 register
  BX    LR                                                 // return to caller

/**
 * @brief   Sends a single character over USART1.
 *
 * @details This function sends a single character over USART1 by writing it to the USART1_DR
 *          register.  It first checks if the transmit buffer is empty (TXE bit) in the
 *          USART1_SR register.  If the buffer is not empty, it waits until it becomes empty
 *          before writing the character to USART1_DR.
 *
 * @param   R7: The character to be sent over USART1.
 * @retval  None
 */
USART1_Transmit_Character:
  LDR   R1, =0x40011000                                    // load address of USART1_SR register
.USART1_Transmit_Character_Loop:
  LDR   R2, [R1]                                           // load value inside USART1_SR register
  AND   R2, #(1<<7)                                        // read TXE bit
  CMP   R2, #0x00                                          // test TX FIFO is not full
  BEQ   .USART1_Transmit_Character_Loop                    // branch if equal
  LDR   R1, =0x40011004                                    // load value inside USART1_DR register
  STR   R7, [R1]                                           // store value into USART1_DR register
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
 * @retval  R7: The received character over USART1.
 */
USART1_Receive_Character:
  LDR   R0, =0x40011000                                    // load address of USART1_SR register
.USART1_Receive_Character_Loop:
  LDR   R1, [R0]                                           // load value inside USART1_SR register
  AND   R1, #(1<<5)                                        // read the RXNE bit
  CMP   R1, #0x00                                          // test TX FIFO is not full
  BEQ   .USART1_Receive_Character_Loop                     // branch if equal
  LDR   R2, =0x40011004                                    // load value inside USART1_DR register
  LDR   R7, [R2]                                           // read value inside USART1_DR register
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
  PUSH  {LR}                                               // push return address onto stack
  CPSID I                                                  // disable global interrupts
  LDR   R0, =0x40020800                                    // load address of GPIOC_MODER register
  LDR   R1, [R0]                                           // load value inside GPIOC_MODER register
  AND   R1, #~(1<<27)                                      // clear the MODER13 bit
  AND   R1, #~(1<<26)                                      // clear the MODER13 bit
  STR   R1, [R0]                                           // store value into GPIOC_MODER register
  LDR   R0, =0x4002080C                                    // load address of GPIOC_PUPDR register
  LDR   R1, [R0]                                           // load value inside GPIOC_PUPDR register
  AND   R1, #~(1<<27)                                      // clear the PUPDR13 bit
  ORR   R1, #(1<<26)                                       // set the PUPDR13 bit
  STR   R1, [R0]                                           // store value into GPIOC_PUPDR register
  LDR   R0, =0x40023844                                    // load address of RCC_ABP2ENR
  LDR   R1, [R0]                                           // load value inside RCC_ABP2ENR register
  ORR   R1, #(1<<14)                                       // set SYSCFGEN bit
  STR   R1, [R0]                                           // store value into RCC_APB2ENR register
  LDR   R0, =0x40013814                                    // load address of SYSCFG_EXTICR4
  LDR   R1, [R0]                                           // load value inside SYSCFG_EXTICR4 register
  ORR   R1, #(1<<5)                                        // set EXTI13 bit
  STR   R1, [R0]                                           // store value into SYSCFG_EXTICR4 register
  LDR   R0, =0x40013C00                                    // load address of EXTI_IMR register
  LDR   R1, [R0]                                           // load value inside EXTI_IMR register
  ORR   R1, #(1<<13)                                       // set MR13 bit
  STR   R1, [R0]                                           // store value into EXTI_IMR register
  LDR   R0, =0x40013C0C                                    // load address of EXTI_FTSR register
  LDR   R1, [R0]                                           // load value inside EXTI_FTSR register
  ORR   R1, #(1<<13)                                       // set TR13 bit
  STR   R1, [R0]                                           // store value into EXTI_IMR register
  BL    NVIC_EnableIRQ_EXTI15_10                           // call function
  CPSIE I                                                  // enable global interrupts
  POP   {LR}                                               // pop return address from stack
  BX    LR                                                 // return to caller

/**
 * @brief   EXTI callback function for button control.
 *
 * @details This EXTI callback function handles a button press on the main board
 *          of the MCU.
 *
 * @param   None
 * @retval  None
 */
EXTI_Callback:
  PUSH  {LR}                                               // push return address onto stack
  MOV   R7, #0x41                                          // 'A'
  BL    USART1_Transmit_Character                          // call function
  POP   {LR}                                               // pop return address from stack
  BX    LR                                                 // return to caller

/**
 * @brief   Initialize USART1 interrupt handling.
 *
 * @details This function initializes USART1 interrupt handling. It first disables global
 *          interrupts, then calls the `NVIC_EnableIRQ_USART1` function to enable USART1
 *          interrupts specifically, and finally re-enables global interrupts before
 *          returning to the caller.
 *
 * @param   None
 * @retval  None
 */
USART1_Int_Init:
  PUSH  {LR}                                               // push return address onto stack
  CPSID I                                                  // disable global interrupts
  BL    NVIC_EnableIRQ_USART1                              // call function
  CPSIE I                                                  // enable global interrupts
  POP   {LR}                                               // pop return address from stack
  BX    LR                                                 // return to caller

/**
 * @brief   USART1 callback function for UART control.
 *
 * @details This USART1 callback function handles an incomming char within the USART1
 *          of the MCU.
 *
 * @param   None
 * @retval  None
 */
USART1_Callback:
  PUSH  {LR}                                               // push return address onto stack
  BL    USART1_Receive_Character                           // call function
  BL    USART1_Transmit_Character                          // call function
  POP   {LR}                                               // pop return address from stack
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
  LDR    R0, =0xE000E104                                   // NVIC_ISER1, p 683 M7 Arch ref manual, ISER1 interrupts 32-63
  LDR    R1, [R0]                                          // load value inside NVIC_ISER1 register
  ORR    R1, #(1<<8)                                       // 0x100=EXTI15-10 (p 204 Ref Manual), p 210 M4 Programming manual, ISER1 8 is 40
  STR    R1, [R0]                                          // store value into R0
  BX     LR                                                // return to caller

/**
 * @brief   Enable NVIC (Nested Vectored Interrupt Controller) for USART1 interrupts.
 *
 * @details This function enables the NVIC for USART1 interrupts. It specifically targets
 *          the NVIC_ISER1 register, which controls interrupts 32 to 63, and sets the bit
 *          corresponding to USART1 (bit 5) to enable USART1 interrupt handling.
 *
 * @param   None
 * @retval  None
 */
NVIC_EnableIRQ_USART1:
  LDR    R0, =0xE000E104                                   // NVIC_ISER1, p 683 M7 Arch ref manual, ISER1 interrupts 32-63
  LDR    R1, [R0]                                          // load value inside NVIC_ISER1 register
  ORR    R1, #(1<<5)                                       // 0x20=USART1 (p 204 Ref Manual), p 210 M4 Programming manual, ISER1 5 is 37
  STR    R1, [R0]                                          // store value into R0
  BX     LR                                                // return to caller

// On p50 of the STM32F401CCUx datasheet, we see the memory map.
// We notice that starting at 0x00000000 we have flash aliased which 
// will be mapped to 0x08000000, flash memory during the linking
// process.  We also see SRAM starting at 0x20000000 and we see our 
// peripherals starting at 0x40000000. On p43 of the reference manual
// we see another table indicating similar info.  On page 45 of 
// the reference manual we see how the flash sectors are organized.
// We begin by creating our linker script, st_nucleo_f4.ld and define the 
// MEMORY and SECTIONS based on p50 of the STM32F401CCUx datasheet.  
// When we write Assembler instructions follow three steps.
// 1. assemble (creates a relocatable object file)
//    arm-none-eabi-as -g main.s -o main.o
// 2. link (map all the 0x0000000 addresses to physical addresses)
//    arm-none-eabi-ld main.o -o main.elf -T st_nucleo_f4.ld
// 3. flash
//    openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program main.elf 0x08000000 verify reset exit"
// We can dive into the object file with the below commands.
// * display the contents of the section headers
//   arm-none-eabi-objdump -h main.o
// * display assembler contents of all sections
//   arm-none-eabi-objdump -D main.o | less
// * display assembler contents of executable sections
//   arm-none-eabi-objdump -d main.o
// * display the full contents of all sections requested
//   arm-none-eabi-objdump -s main.o | less 
// * display information about the contents of ELF format files
//   arm-none-eabi-readelf -a main.elf | less
// * list symbols in file
//   arm-none-eabi-nm main.elf | less
// Keep in mind when you are using objdump there may be data in the binary
// which objdump will try to interpret as instructions so you can ignore those
// parts.
ARM_Cortex_M4_Linker_Script:
  BX     LR                                                // return to caller

// The ARM Cortex M4 has a vector table which the first thing loaded into memory
// on boot.  On page 202 of the reference manual, we see the vector table.
// These will also get mapped during the linking process.
// The the very base of memory is the MSP or master stack pointer followed
// by the Reset_Handler function.  The rest of the handlers follow the Reset_Handler
// followed by the IRQ or interrupt requests.
// * 85 IRQ's
// * 15 system exceptions
// * master stack pointer
// * 85+15+1 = 101 * 4 = 404 bytes or 0x194 bytes
ARM_Cortex_M4_Vector_Table:
  BX     LR                                                // return to caller

// The ARM Cortex M4 has 13 general-purpose registers (R0-R12), 1 stack pointer
// register (R13 or SP), 1 link register (R14 or LR), 1 program counter (R15 or
// PC) and a status register (xPSR) and 3 interrupt mask registers.  
// There are peripheral registers as well located on page 38 of the reference
// manual.  All registers are 32-bits wide or 4 bytes wide.
// * ASSEMBLE AND LINK w/ SYMBOLS:
// * 1. arm-none-eabi-as -g main.s -o main.o
// * 2. arm-none-eabi-ld main.o -o main.elf -T st_nucleo_f4.ld
// * 3. openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program main.elf 0x08000000 verify reset exit"
// * DEBUG w/ SYMBOLS:
// * 1. openocd -f board/st_nucleo_f4.cfg
// * 2. arm-none-eabi-gdb main.elf
// * 3. target remote :3333
// * 4. monitor reset halt
// * 5. b __start
// * 6. c
ARM_Cortex_M4_Registers:
  BX     LR                                                // return to caller

// Move an immedaite value into R0.
MOV_Immediate:
  MOV    R0, #0x42                                         // move 0x42 into R0
  BX     LR                                                // return to caller

// Move an immediate value into R0 and R1 and then move R1 into R0.
MOVS_Register:
  MOV    R0, #0x42                                         // move 0x42 into R0
  MOV    R1, #0x43                                         // move 0x43 into R1
  MOVS   R0, R1                                            // move R1 into R0
  BX     LR                                                // return to caller

// Load an address into R0, load value inside R0 into R1, set a bit and then store back the value into the address.
LDR_Immediate:
  LDR    R0, =0x40023830                                   // load address of RCC_AHB1ENR register (p118 RM)
  LDR    R1, [R0]                                          // load value inside RCC_AHB1ENR register
  ORR    R1, #(1<<1)                                       // set the GPIOBEN bit
  STR    R1, [R0]                                          // store value into RCC_AHB1ENR register
  BX     LR                                                // return to caller

// Load the SRAM base address into R0, load an offset address into R1 and left shift the offset in R1 by 1 which multiplies the value
// by 2 and then add it to the base value producing 0x20000020 to be stored back into R2. Set a 1 in the LSB and store back the value 
// into R0.
LDR_Offset:
  LDR    R0, =0x20000000                                   // SRAM base (p42 RM)
  LDR    R1, =0x10                                         // offset address
  LDR    R2, [R0, R1, LSL #1]                              // 0x20000000 + (0x10 * 2) = 0x20000020, then store what is inside 0x20000020 into R2
  ORR    R2, #(1<<0)                                       // put a 1 in LSB with the contents inside R2
  STR    R2, [R0]                                          // store the contents inside R2 into 0x20000000
  BX     LR                                                // return to caller

// Move half word into the MSB of R0 and then another half word into the LSB of R0 and load an address in RAM and store that value into it.
MOVW_MOVT:
  MOVW   R0, #0x5678                                       // move half word into MSB of R0
  MOVT   R0, #0x1234                                       // move half word into LSB of R0
  LDR    R1, =0x20002000                                   // load address in RAM
  STR    R0, [R1]                                          // store word into the value stored inside R1
  BX     LR                                                // return to caller

// Move a negative value into R0 and R1 and add them to set the N, C and T bits in the xPSR (0b10100001000000000000000000000000).  
// Load the max 32-bit signed value into R0 and load a value into R1 and add them to set the N, V and T bits inside the xPSR 
// (0b10010001000000000000000000000000). Load the max 32-bit unsigned value into R0 and load a value into R1 and add them to 
// set the Z, C and T bits inside the xPSR (0b01100001000000000000000000000000).
Flags:
  MOV    R0, #-1                                           // move a negative value into R0
  MOV    R1, #-2                                           // move a negative value into R1
  ADDS   R0, R1                                            // N, C, T bits set in xPSR (negative, carry and thumb) (p573 ARM)
  LDR    R0, =0x7FFFFFFF                                   // load the max 32-bit signed value into R0
  LDR    R1, =0x01                                         // load a value into R1 that when added to R0 will overflow
  ADDS   R0, R1                                            // N, V, T bits set in xPSR (negative, overflow and thumb)
  LDR    R0, =0xFFFFFFFF                                   // load the max 32-bit unsigned value into R0
  LDR    R1, =0x01                                         // load a value into R1 that when added to R0 will overflow
  ADDS   R0, R1                                            // Z, C, T bits set in xPSR (zero, carry, thumb)
  BX     LR                                                // return to caller

// Load the GPIOB_MODER register into R0 and move the value inside the register into R1, perofrm at bit clear and then a bit set and 
// store the value back into the register.
Bit_Set_Clear:
  LDR   R0, =0x40020400                                    // load address of GPIOB_MODER register (p158 RM)
  LDR   R1, [R0]                                           // load value inside GPIOB_MODER register
  AND   R1, #~(1<<1)                                       // clear the MODER0 bit
  ORR   R1, #(1<<1)                                        // set the MODER0 bit
  STR   R1, [R0]                                           // store value into GPIOB_MODER register
  BX    LR                                                 // return to caller

// Load the EXTI_PR register into R0 and the contents into R1 and test if bit 13 is a 0 (0b00100000000000000000010000000000)
// then move a random mock value into R0 and compare against antother random mock velue and branch appropriately.
Conditional_Branching:
  LDR   R0, =0x40013C14                                    // load the address of EXTI_PR register (p211 RM)
  LDR   R1, [R0]                                           // load value inside EXTI_PR register
  TST   R1, #(1<<13)                                       // read the PR13 bit, if 0, then BEQ
  BEQ   .Conditional_TST_Finish                            // branch if equal
  NOP                                                      // dummy logic
.Conditional_TST_Finish:
  MOV   R0, #0x40                                          // random mock value
  CMP   R0, #0x40                                          // compare random mock value to another value
  BNE   .Conditional_TST_Finish                            // branch if not equal
  BLE   .Conditional_CMP_Finish                            // branch if less than or equal
  NOP                                                      // dummy logic
.Conditional_CMP_Finish:  
  BX     LR                                                // return to caller

// Move a value into R0 and we have a NOP representing any logic you would want run then subtract 1 from the counter
// and compare against 0 otherwise loop.
Loops:
  MOV    R0, #0x10                                         // move a value into R0
.For_Loop:
  NOP                                                      // dummy logic
  SUBS   R0, #0x01                                         // subtract a value from R0
  CMP    R0, #0x00                                         // subtract R0 from a value without updating R0
  BNE    .For_Loop                                         // branch if not equal
  BX     LR                                                // return to caller

// Review EXTI15_10_IRQHandler and USART1_IRQHandler.  The vector table's weak reference must be 
// changed to a .word as we define the handler.  Review blue button press and observe 'A' displayed
// on UART and typing a char in the UART displays the char.  These actions are independent of the 
// main loop and will interrupt the main loop.
Interrupts:
  BX     LR                                                // return to caller
