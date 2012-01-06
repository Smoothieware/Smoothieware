@echo off
rem Copyright 2011 Adam Green (http://mbed.org/users/AdamGreen/)
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
set CODE_SOURCERY_FILENAME=arm-2011.03-42-arm-none-eabi-i686-mingw32.tar.bz2
set CODE_SOURCERY_URL=https://sourcery.mentor.com/sgpp/lite/arm/portal/package8735/public/arm-none-eabi/%CODE_SOURCERY_FILENAME%
set CODE_SOURCERY_TAR=%ROOTDIR%%CODE_SOURCERY_FILENAME%
set CODE_SOURCERY_MD5=%ROOTDIR%external\win32\%CODE_SOURCERY_FILENAME%.md5
set CODE_SOURCERY_VERSION=arm-2011.03
set CODE_SOURCERY_BINDIR=%ROOTDIR%%CODE_SOURCERY_VERSION%\bin
set BUILDENV_CMD=%CODE_SOURCERY_BINDIR%\buildenv.cmd
set MAKE_CMD=%CODE_SOURCERY_BINDIR%\make.cmd
set BUILDSHELL_CMD=%ROOTDIR%BuildShell.cmd


rem Make sure that we are running with current directory set to where this
rem batch file is located.
pushd %ROOTDIR%

rem Initialize install log files.
echo Logging install results to %LOGFILE%
echo %DATE% %TIME%  Starting %0 %*>%LOGFILE%

echo Downloading Code Sourcery G++ Lite...
echo %DATE% %TIME%  Executing external\win32\curl -kL0 %CODE_SOURCERY_URL%>>%LOGFILE%
external\win32\curl -kL0 %CODE_SOURCERY_URL% >%CODE_SOURCERY_TAR%
if errorlevel 1 goto ExitOnError

echo Validating md5 signature of Code Sourcery G++ Lite...
call :RunAndLog external\win32\md5sum --check %CODE_SOURCERY_MD5%
if errorlevel 1 goto ExitOnError

echo Extracting Code Sourcery G++ Lite files...
call :RunAndLog external\win32\bsdtar xf %CODE_SOURCERY_TAR%
if errorlevel 1 goto ExitOnError

echo Creating helper scripts...
echo @echo off>%BUILDENV_CMD%
echo REM Uncomment next line and set destination drive to match mbed device>>%BUILDENV_CMD%
echo REM SET LPC_DEPLOY=copy PROJECT.bin f:\>>%BUILDENV_CMD%
echo.>>%BUILDENV_CMD%
echo SET PATH=%%~dp0;%%PATH%%>>%BUILDENV_CMD%
rem
echo @cs-make %%*>%MAKE_CMD%
rem
echo @cmd.exe /K %%~dp0\%CODE_SOURCERY_VERSION%\bin\buildenv.cmd>>%BUILDSHELL_CMD%

rem Place Code Sourcery G++ Lite tools in the path before building gcc4mbed code.
set path=%CODE_SOURCERY_BINDIR%;%PATH%

echo Installing mbed libs and headers...
call :RunAndLog cs-make install_mbed
if errorlevel 1 goto ExitOnError

echo Performing a clean build of the gcc4mbed samples...
call :RunAndLog cs-make clean
if errorlevel 1 goto ExitOnError
call :RunAndLog cs-make
if errorlevel 1 goto ExitOnError

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