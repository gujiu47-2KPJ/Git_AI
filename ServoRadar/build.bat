@echo off
cd /d %~dp0
cmake -B build/Debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build/Debug
pause