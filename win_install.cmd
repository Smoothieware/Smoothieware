@echo off
rem Copyright 2012 Adam Green (http://mbed.org/users/AdamGreen/)
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

set GCC4ARM_FILENAME=gcc-arm-none-eabi-4_7-2012q4-20121208-linux.tar.bz2
set GCC4ARM_URL=https://launchpad.net/gcc-arm-embedded/4.7/4.7-2012-q4-major/+download/%GCC4ARM_FILENAME%
set GCC4ARM_TAR=%ROOTDIR%%GCC4ARM_FILENAME%
set GCC4ARM_MD5=%ROOTDIR%build\win32\gcc-arm-none-eabi.md5
set GCC4ARM_VERSION=gcc-arm-none-eabi-4_7-2012q4
set GCC4ARM_EXTRACT=%ROOTDIR%%GCC4ARM_VERSION%
set GCC4ARM_DIR=%ROOTDIR%gcc-arm-none-eabi
set GCC4ARM_BINDIR=%GCC4ARM_DIR%\bin
set GCC4ARM_LIBEXEC=%GCC4ARM_DIR%\lib\gcc\arm-none-eabi\4.7.3
set WINBIN_URL=https://github.com/adamgreen/GCC-ARM-Embedded-20121208/tarball/master
set WINBIN_TAR=%ROOTDIR%GCC-ARM-Embedded-20121208.tar.gz

set WINBIN_MD5=%ROOTDIR%build\win32\GCC-ARM-Embedded.md5
set WINBIN_BASEDIR=%ROOTDIR%GCC-ARM-Embedded
set WINBIN_DIR=%WINBIN_BASEDIR%\win32
set OUR_MAKE=%ROOTDIR%build\win32\make.exe
set BUILDENV_CMD=%GCC4ARM_BINDIR%\buildenv.cmd
set BUILDSHELL_CMD=%ROOTDIR%BuildShell.cmd
set ERROR_ENCOUNTERED=0


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

echo Validating md5 signature of Code Sourcery G++ Lite...
call :RunAndLog build\win32\md5sum --check %GCC4ARM_MD5%
if errorlevel 1 goto ExitOnError

echo Downloading Windows GCC binaries from github...
echo %DATE% %TIME%  Executing build\win32\curl -kL0 %WINBIN_URL%>>%LOGFILE%
build\win32\curl -kL0 %WINBIN_URL% >%WINBIN_TAR%
if errorlevel 1 goto ExitOnError

echo Validating md5 signature of Windows GCC binaries...
call :RunAndLog build\win32\md5sum --check %WINBIN_MD5%
if errorlevel 1 goto ExitOnError

echo Extracting GNU Tools for ARM Embedded Processors...
call :RunAndLog rd /s /q %GCC4ARM_EXTRACT%
call :RunAndLog rd /s /q %GCC4ARM_DIR%
call :RunAndLog build\win32\bsdtar xf %GCC4ARM_TAR%
if errorlevel 1 goto ExitOnError
call :RunAndLog move %GCC4ARM_EXTRACT% %GCC4ARM_DIR%
if errorlevel 1 goto ExitOnError

echo Extracting Windows GCC binaries...
call :RunAndLog rd /s /q %WINBIN_BASEDIR%
call :RunAndLog build\win32\bsdtar xf %WINBIN_TAR%
for /d %%i in (adamgreen-GCC-ARM-Embedded-*) do call :RunAndLog move %%i %WINBIN_BASEDIR%
if errorlevel 1 goto ExitOnError

echo Installing Windows binaries...
call :RunAndLog del /q %GCC4ARM_BINDIR%\*
call :RunAndLog copy %WINBIN_DIR%\arm-none-eabi-* %GCC4ARM_BINDIR%\
if errorlevel 1 goto ExitOnError
for %%i in (as g++ ld objcopy ranlib ar c++ gcc nm objdump strip) do call :CopyGccFile %%i
if "%ERROR_ENCOUNTERED%"=="1" goto ExitOnError
call :RunAndLog xcopy /eiy %WINBIN_DIR%\libexec %GCC4ARM_LIBEXEC%
if errorlevel 1 goto ExitOnError

echo Creating helper scripts...
echo @echo off>%BUILDENV_CMD%
echo REM Uncomment next line and set destination drive to match mbed device>>%BUILDENV_CMD%
echo REM SET LPC_DEPLOY=copy PROJECT.bin f:\>>%BUILDENV_CMD%
echo.>>%BUILDENV_CMD%
echo SET PATH=%%~dp0;%%~dp0..\..\build\win32;%%PATH%%>>%BUILDENV_CMD%
rem
echo @cmd.exe /K %%~dp0\gcc-arm-none-eabi\bin\buildenv.cmd>%BUILDSHELL_CMD%

echo Cleaning up intermediate files...
call :RunAndLog rd /s /q %WINBIN_BASEDIR%
call :RunAndLog del /f %WINBIN_TAR%
call :RunAndLog del /f %GCC4ARM_TAR%

echo **************************************************************************
echo To build Smoothie, you will first need to run the following batch
echo file so that your environment variables are set correctly:
echo  %BUILDSHELL_CMD%
echo You will want to run this each time you start a new Command Prompt.  You
echo can simply double-click on this batch file from Explorer to launch a
echo Command Prompt that has been properly initialized for building.
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



rem Copies a file between GCC directories where one has arm-none-eabi prefix and the other doesn't
:CopyGccFile
call :RunAndLog del "%GCC4ARM_DIR%\arm-none-eabi\bin\%1"
call :RunAndLog copy /y "%GCC4ARM_BINDIR%\arm-none-eabi-%1.exe" "%GCC4ARM_DIR%\arm-none-eabi\bin\%1.exe"
if errorlevel 1 set ERROR_ENCOUNTERED=1
goto :EOF


rem Exits the batch file due to error.
rem Make sure that any stderr text ends up in win_install.log and the restore
rem the current dictory before forcing an early exit.
:ExitOnError
type %ERRORFILE% >>%LOGFILE%
echo %DATE% %TIME%  Failure forced early exit>>%LOGFILE%
type %LOGFILE%


:Exit
del %ERRORFILE%
popd
pause