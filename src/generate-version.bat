@for /f "tokens=3* delims=/" %%a in ('git symbolic-ref HEAD') do @set myvar=%%a
@for /f "delims=" %%b in ('git log "--pretty=format:%%h" -1') do @set myvar2=%%b
@echo %myvar%-%myvar2%
@copy /b  version.cpp+,, >NUL:

