setlocal

@echo off
FOR %%G IN (2019) DO (call :subroutine "%%G")
GOTO end

:subroutine
set builddir=mayabuild.%1
if not exist %builddir% goto BUILDENV
rmdir %builddir% /S /Q
:BUILDENV
mkdir %builddir%
cd %builddir%
if %1 LSS "2020" (
    cmake -A x64 -T v140 -DMAYA_VERSION=%1 ../
) ELSE (
    cmake -A x64 -T v141 -DMAYA_VERSION=%1 ../
)
REM cmake --build . --target install --config Release
cd ..
goto :eof

:end
pause

