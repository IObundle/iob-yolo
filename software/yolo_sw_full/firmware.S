#include "system.h"

.section .init
.global main

/* set stack pointer */
lui sp, %hi(27<<(MAINRAM_ADDR_W-5))
addi sp, sp, %lo(27<<(MAINRAM_ADDR_W-5))

/* call main */
jal ra, main

/* break */
ebreak
