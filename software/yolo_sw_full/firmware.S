#include "system.h"

.section .init
.global main

/* set stack pointer */
lui sp, %hi(13<<(MAINRAM_ADDR_W-4))
addi sp, sp, %lo(13<<(MAINRAM_ADDR_W-4))

/* call main */
jal ra, main

/* break */
ebreak
