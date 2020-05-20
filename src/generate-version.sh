echo `git symbolic-ref HEAD 2> /dev/null | cut -b 12-`-`git rev-parse --short=7 HEAD`
touch version.cpp
