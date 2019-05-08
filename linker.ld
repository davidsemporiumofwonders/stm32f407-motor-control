MEMORY
{
   FLASH  		(rx) 	: ORIGIN = 0x08000000, LENGTH = 512k
   RAM   		(rwx) 	: ORIGIN = 0x20000000, LENGTH = 112k
   RAM_BANK_2	(rwx)	: ORIGIN = 0x2001C000, LENGTH = 12k
   CCMRAM 		(rw) 	: ORIGIN = 0x10000000, LENGTH = 64k
}

STACK_SIZE = 8192;
_estack = ORIGIN(CCMRAM) + LENGTH(CCMRAM);
_sstack = _estack - STACK_SIZE;

SECTIONS
{
	/* flash */
    .text :
    {
        /* All unused bytes set to 0xFF. */
        FILL (0xff)
        _svtflash = .;   
        /*Vector Table */
        KEEP(*(.vector_table))
 		_evtflash = .; 
 		 
        . = ALIGN(4);
        *(.text)        /* Program code */
        *(.text.*)
        *(.rodata)      /* Constants */
        *(.fini)    
        *(.rodata*) 
        *(.glue_7)  
        *(.glue_7t) 
        . = ALIGN(4);
        _etext = .;
 
        _sidata = _etext;
         
    } > FLASH
    
    _eidata = .;
 
    /* Initialized global and static variables */
    .data :
    {
        FILL (0xff)
         
        . = ALIGN(4);
        _sdata = .;     
        *(.data)
        *(.data.*)
        . = ALIGN(4);
        _edata = .;
         
    } > RAM AT> FLASH
 
    /* Uninitialized global and static variables */
    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss.*)
        *(COMMON)
        . = ALIGN(4);
 
        _ebss = .;
         
    } > RAM
 
    /* bank 2 */
    .ram_bank_2 :
    {
        . = ALIGN(4);
        _sram_bank_2 = .;
        *(.ram_bank_2)
        *(.ram_bank_2.*)
        *(COMMON)
        . = ALIGN(4);
 
        _eram_bank_2 = .;
         
    } > RAM_BANK_2
 	
 	/*ccmram */
    .ccmram :
	{
	
    . = ALIGN(4);
    _sccmram = .;     
    *(.ccmram)
    *(.ccmram*)
    . = ALIGN(4);
    _eccmram = .;

 	} > CCMRAM
 	
 	
 	/* Stack */
    . = _sstack;
    .stack :
    {
        . = . + STACK_SIZE;
    } > CCMRAM
    
}