
/*
 * Entry function, placed at interrupt vector 0 (RESET).
 */

#pragma code _entry 0x1400

void _entry (void) __naked __interrupt 0
{
  __asm
    goto _main;
  __endasm;
}
