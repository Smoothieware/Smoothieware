@echo off
rem Copyright 2013 Adam Green (http://mbed.org/users/AdamGreen/)
rem
rem Licensed under the Apache License, Version 2.0 (the "License");
rem you may not use this file except in compliance with the License.
rem You may obtain a copy of the License at
rem
rem     http://www.apache.org/licenses/LICENSE-2.0
rem
rem Unless required by applicable law or agreed to in writing, software
rem distributed under the License is distributed on an "AS IS" BASIS,
rem WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
rem See the License for the specific language governing permissions and
rem limitations under the License.


rem Setup batch file specific environment variables.
setlocal
set ROOTDIR=%~dp0
set LOGFILE=%ROOTDIR%win_install.log
set ERRORFILE=%ROOTDIR%win_install.err
set GCC4ARM_VERSION=gcc-arm-none-eabi-4_8-2014q1
set GCC4ARM_FILENAME=gcc-arm-none-eabi-4_8-2014q1-20140314-win32.zip
set GCC4ARM_URL=https://launchpad.net/gcc-arm-embedded/4.8/4.8-2014-q1-update/+download/%GCC4ARM_FILENAME%
set GCC4ARM_TAR=%ROOTDIR%%GCC4ARM_FILENAME%
set GCC4ARM_MD5=09c19b3248863074f5498a88f31bee16
set GCC4ARM_MD5_FILENAME=%ROOTDIR%gcc-arm-none-eabi.md5
set GCC4ARM_DIR=%ROOTDIR%gcc-arm-none-eabi
set GCC4ARM_BINDIR=%GCC4ARM_DIR%\bin
set OUR_MAKE=%ROOTDIR%build\win32\make.exe
set BUILDENV_CMD=%GCC4ARM_BINDIR%\buildenv.cmd
set BUILDSHELL_CMD=%ROOTDIR%BuildShell.cmd
set BUILDSHELL_DEBUG_CMD=%ROOTDIR%BuildShellDebug.cmd


rem Make sure that we are running with current directory set to where this
rem batch file is located.
pushd %ROOTDIR%

rem Initialize install log files.
echo Logging install results to %LOGFILE%
echo %DATE% %TIME%  Starting %0 %*>%LOGFILE%

echo Downloading GNU Tools for ARM Embedded Processors...
echo %DATE% %TIME%  Executing build\win32\curl -kL0 %GCC4ARM_URL%>>%LOGFILE%
build\win32\curl -kL0 %GCC4ARM_URL% >%GCC4ARM_TAR%
if errorlevel 1 goto ExitOnError

echo Validating md5 signature of GNU Tools for ARM Embedded Processors...
echo %GCC4ARM_MD5% *%GCC4ARM_FILENAME%>%GCC4ARM_MD5_FILENAME%
call :RunAndLog build\win32\md5sum --check %GCC4ARM_MD5_FILENAME%
if errorlevel 1 goto ExitOnError
del "%GCC4ARM_MD5_FILENAME%"

echo Extracting GNU Tools for ARM Embedded Processors...
call :RunAndLog rd /s /q %GCC4ARM_DIR%
call :RunAndLog md %GCC4ARM_DIR%
if errorlevel 1 goto ExitOnError
call :RunAndLog cd %GCC4ARM_DIR%
call :RunAndLog ..\build\win32\bsdtar xf %GCC4ARM_TAR%
if errorlevel 1 goto ExitOnError
call :RunAndLog cd ..

echo Creating helper scripts...
echo @echo off>%BUILDENV_CMD%
echo REM Uncomment next line and set destination drive to match mbed device>>%BUILDENV_CMD%
echo REM SET LPC_DEPLOY=copy PROJECT.bin f:\>>%BUILDENV_CMD%
echo.>>%BUILDENV_CMD%
echo SET PATH=%%~dp0;%%~dp0..\..\build\win32;%%PATH%%>>%BUILDENV_CMD%
rem
echo @cmd.exe /K %%~dp0\gcc-arm-none-eabi\bin\buildenv.cmd>%BUILDSHELL_CMD%

rem Place GNU Tool for ARM Embedded Processors in the path before building gcc4mbed code.
set path=%GCC4ARM_BINDIR%;%ROOTDIR%build\win32;%PATH%

echo Performing a clean build of the gcc4mbed samples...
call :RunAndLog %OUR_MAKE% clean
if errorlevel 1 goto ExitOnError
call :RunAndLog %OUR_MAKE%
if errorlevel 1 goto ExitOnError

echo Cleaning up intermediate files...
call :RunAndLog del /f %GCC4ARM_TAR%

echo **************************************************************************
echo To build gcc4mbed samples, you will first need to run the following batch
echo file so that your environment variables are set correctly:
echo  %BUILDSHELL_CMD%
echo You will want to run this each time you start a new Command Prompt.  You
echo can simply double-click on this batch file from Explorer to launch a
echo Command Prompt that has been properly initialized for building gcc4mbed
echo based code.
echo **************************************************************************

rem Restore current directory and exit batch file on success.
echo %DATE% %TIME%  Finished successfully>>%LOGFILE%
echo Finished successfully
goto Exit



rem Logs the command to be run and then executes the command while logging the results.
:RunAndLog
echo %DATE% %TIME%  Executing %*>>%LOGFILE%
%* 1>>%LOGFILE% 2>%ERRORFILE%
goto :EOF



rem Exits the batch file due to error.
rem Make sure that any stderr text ends up in win_install.log and then restore
rem the current directory before forcing an early exit.
:ExitOnError
type %ERRORFILE% >>%LOGFILE%
echo %DATE% %TIME%  Failure forced early exit>>%LOGFILE%
type %LOGFILE%


:Exit
del %ERRORFILE%
popd
pause
