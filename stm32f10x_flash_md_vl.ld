/*
Default linker script for STM32F10x_128K_20K
Original Copyright RAISONANCE S.A.S. 2008
Modified P Harrison May 2009
*/

/* 
 *	Default stack sizes. 
 * 
 *	These are used by the startup in order to allocate stacks for the different modes.
 *	PROVIDE" allows to easily override these values from an object file or the commmand line.
 */

__Stack_Size = 1024 ;
PROVIDE ( _Stack_Size = __Stack_Size ) ;
__Stack_Init = _estack  - __Stack_Size ;
PROVIDE ( _Stack_Init = __Stack_Init ) ;

/*
 *There will be a link error if there is not this amount of RAM free at the end.
 */
_Minimum_Stack_Size = 0x100 ;


MEMORY
{
  RAM (xrw)     : ORIGIN = 0x20000000, LENGTH =  8K
  FLASH (rx)    : ORIGIN = 0x08000000, LENGTH = 128K
}

/* higher address of the user mode stack */
_estack = 0x20002000;

SECTIONS
{
/* 
 * for Cortex devices, the beginning of the startup code is stored in the .isr_vector section, 
 * which goes to FLASH 
 */
	.isr_vector :
    {
			. = ALIGN(4);
      KEEP(*(.isr_vector))            /* Startup code */
			. = ALIGN(4);
    } >FLASH

/* 
 * for some STRx devices, the beginning of the startup code is stored in the .flashtext section, 
 * which goes to FLASH 
 */
	.flashtext :
    {
			. = ALIGN(4);
      KEEP (*(.flashtext))           /* Startup code */
			. = ALIGN(4);
    } >FLASH
    
 /* 
  * the program code is stored in the .text section, which goes to Flash 
  */
  .text :
		{
	    . = ALIGN(4);
	    *(.text)                   /* remaining code */
      *(.text.*)                   /* remaining code */
      *(.rodata)                 /* read-only data (constants) */
      *(.rodata*)
      *(.glue_7)
      *(.glue_7t)
 			. = ALIGN(4);
   	 	_etext = .;
	    _sidata = _etext; /* Uused by the startup in order to initialize the .data secion */
    } >FLASH
    
/* 
 * This is the initialized data section. It is stored in RAM but the initial values
 * are held in flash and copied to RAM by the startup code
 */   
  .data  : AT ( _sidata )  /* AT makes the LMA follow on in the binary image */
    {
	    . = ALIGN(4);
      _sdata = . ; /* Used by the startup in order to initialize the .data section */
      KEEP( *(.data) )
      KEEP( *(.data.*) )
			. = ALIGN(4);
	    _edata = . ; /* Used by the startup in order to initialize the .data section */
    } >RAM
    
    

/* 
 * This is the uninitialized data section. Date here is stored in RAM and will be
 * set to zero by the startup code.
 */
	.bss :
    {
	  	. = ALIGN(4);
      _sbss = .; /* Used by the startup in order to initialize the .bss section */
      *(.bss)
      *(COMMON)
      . = ALIGN(4);
	    _ebss = . ; /* Used by the startup in order to initialize the .bss section */
    } >RAM
    
PROVIDE ( end = _ebss );
PROVIDE ( _end = _ebss );
    
/* 
 * This is the user stack section 
 * This is just to check that there is enough RAM left for the User mode stack
 * It should generate an error if it's full.
 */
    ._usrstack :
    {
	    . = ALIGN(4);
      _susrstack = . ;
      . = . + _Minimum_Stack_Size ;
      . = ALIGN(4);
      _eusrstack = . ;
    } >RAM
    
/* 
 * after that it's only debugging information. 
 */
 
/* remove the debugging information from the standard libraries */
DISCARD :
	{
  	libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }
}
