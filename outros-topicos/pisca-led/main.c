/* main.c - pisca o LED da Blackpill (PC13) */
#include <stdint.h>

/* Registradores (endereços do RM0383) */
#define RCC_AHB1ENR \
  (*(volatile uint32_t *)0x40023830)
#define GPIOC_MODER \
  (*(volatile uint32_t *)0x40020800)
#define GPIOC_ODR \
  (*(volatile uint32_t *)0x40020814)

/* Símbolos definidos no linker.ld */
extern uint32_t _estack;
extern uint32_t _sidata, _sdata, _edata;
extern uint32_t _sbss, _ebss;

int main(void);

/* 1ª função executada após o reset */
void reset_handler(void)
{
  uint32_t *src = &_sidata;
  uint32_t *dst = &_sdata;

  /* copia .data da flash para a RAM */
  while (dst < &_edata)
    *dst++ = *src++;

  /* zera .bss */
  for (dst = &_sbss; dst < &_ebss; )
    *dst++ = 0;

  main();
  while (1) {}
}

/* Tabela de vetores: vai em 0x08000000 */
__attribute__((section(".isr_vector"), used))
const void *vetores[] = {
  &_estack,       /* SP inicial      */
  reset_handler,  /* endereço do PC  */
};

static void espera(uint32_t n)
{
  while (n--)
    __asm__ volatile ("nop");
}

int main(void)
{
  /* liga o clock da porta GPIOC (bit 2) */
  RCC_AHB1ENR |= (1u << 2);

  /* PC13 como saída: MODER13 = 01 */
  GPIOC_MODER &= ~(3u << 26);
  GPIOC_MODER |=  (1u << 26);

  while (1) {
    GPIOC_ODR ^= (1u << 13); /* inverte */
    espera(500000);
  }
}
