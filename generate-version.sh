echo "#include \"version.h\"" > src/version.cpp
echo "const char *Version::get_build(void) const {" >> src/version.cpp
echo "    return \"`git symbolic-ref HEAD 2> /dev/null | cut -b 12-`-`git log --pretty=format:\"%h\" -1`\";" >> src/version.cpp
echo "}" >> src/version.cpp
echo "const char *Version::get_build_date(void) const {" >> src/version.cpp
echo "    return __DATE__ \" \" __TIME__;" >> src/version.cpp
echo "}" >> src/version.cpp

