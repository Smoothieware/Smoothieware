@for /f "delims=" %%a in ('git symbolic-ref HEAD') do @set myvar=%%a
@for /f "delims=" %%b in ('git log "--pretty=format:%%h" -1') do @set myvar2=%%b
@echo %myvar:~11%-%myvar2%
@copy /b  version.cpp+,, >NUL:

