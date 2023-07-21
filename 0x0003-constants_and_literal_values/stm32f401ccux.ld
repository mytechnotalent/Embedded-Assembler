/**
 * FILE: stm32f401ccux.ld
 *
 * DESCRIPTION:
 * This file contains the linker script
 * utilizing the STM32F401CC6 microcontroller.
 *
 * AUTHOR: Kevin Thomas
 * CREATION DATE: July 2, 2023
 * UPDATE Date: July 2, 2023
 *
 */

MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 256K
  SRAM  : ORIGIN = 0x20000000, LENGTH = 64K
}

SECTIONS
{
  .isr_vector :
  { 
    *(.isr_vector)
  } >FLASH
  .text :
  {
    *(.text)
  } >FLASH
  .data (NOLOAD) :
  {
    . = . + 0x400;
	_estack = .;
	*(.data)
  } >SRAM
}  
