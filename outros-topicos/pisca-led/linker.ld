/* linker.ld - STM32F411CE */
ENTRY(reset_handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000,
                LENGTH = 512K
  RAM   (rwx) : ORIGIN = 0x20000000,
                LENGTH = 128K
}

/* pilha começa no topo da RAM */
_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  .isr_vector : {
    KEEP(*(.isr_vector))
  } > FLASH

  .text : {
    *(.text*)
    *(.rodata*)
    . = ALIGN(4);
  } > FLASH

  _sidata = LOADADDR(.data);

  .data : {
    _sdata = .;
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } > RAM AT > FLASH

  .bss : {
    _sbss = .;
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
  } > RAM
}
