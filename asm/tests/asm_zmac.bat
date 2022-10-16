@REM z80asm.exe --verbose --list asm.lst --output asm.bin %1
zmac --od . --oo cim -o asm.cim -L -l %1 >asm.lst
