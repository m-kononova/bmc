  .syntax unified
  .cpu cortex-m4
  .fpu softvfp
  .thumb

.equ FLASH_BASE, 0x08000000

.global ledEnable

/*
 * Some testing functions
 */
  ledEnable:
      mov r0, #1
      lsl r0, r0, #31
      mrs r0, psr
      add r0, #0x40000000
      msr psr_nzcvq, r0
      movs r0, #0xffffffff
      mrs r0, psr
      subs r0, #0x80000000
      ldr r0, =FLASH_BASE @Set to magic value
      ldr r1, [r0, #4]
      movs r1, #0x7fffffff
      movs r2, #0x80000000

      push {lr}
          bl locadd
      pop {lr}

      movs r0, #1
  loop:
      push {r0}
      subs r0, #1
      cmp r0, #0
      bgt loop

      push {r1}
      push {r2}
      push {r3}
      bx lr

locadd:
    push {r1}
    push {r2}
    add r0, r1, r2
    pop {r2}
    pop {r1}
    bx lr
