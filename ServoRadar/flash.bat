@echo off
cd /d %~dp0
E:\tools\openocd\xpack-openocd-0.12.0-7\bin\openocd.exe -f .vscode\openocd_stm32f103.cfg -c "program build/Debug/ServoRadar.elf 0x08000000 verify reset exit"
pause