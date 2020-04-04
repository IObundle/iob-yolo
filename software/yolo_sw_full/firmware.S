#include "system.h"

.section .init
.global main

/* set stack pointer */
lui sp, %hi(3<<(MAINRAM_ADDR_W-2))
addi sp, sp, %lo(3<<(MAINRAM_ADDR_W-2))

/* call main */
jal ra, main

/* break */
ebreak
