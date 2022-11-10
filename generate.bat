ren %cd%\Core\Src\main.cpp main.c

@echo off
echo config load %cd%\AvionicsSoftware.ioc > %cd%\instructions.txt
echo project toolchain STM32CubeIDE >> %cd%\instructions.txt
echo project path %cd% >> %cd%\instructions.txt
echo project generate >> %cd%\instructions.txt
echo exit >> %cd%\instructions.txt

WHERE /R "C:\Program Files\STMicroelectronics" STM32CubeMX.exe > temp.txt
set /p VAR=<temp.txt
del temp.txt
echo %VAR%

pause
java -jar "%VAR%" -s instructions.txt
del instructions.txt

ren %cd%\Core\Src\main.c main.cpp
pause