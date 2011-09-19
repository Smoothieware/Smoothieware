s/pushenvvar CSL_SCRIPTDIR/#/g
s:pushenvvar PATH /usr/local/tools/:#:g
s/pushenvvar LD_LIBRARY_PATH/#/g
s:#.*/init/source_package/binutils:if [ "" ] ; then:g
s:#.*/i686-pc-linux-gnu/host_cleanup:fi:g
s:/i686-pc-linux-gnu/:/macosx/:g
s/-i686-pc-linux-gnu/-macosx/g
s/i686-pc-linux-gnu-//g
s/i686-pc-linux-gnu//g
s: --css-ref=../cs.css::g
s:#.*/toolchain/binutils/postinstall:if [ "" ] ; then:g
s:#.*/macosx/toolchain/gcc_first/configure:fi:g
s/-Bstatic,//g
s/-Bdynamic //g
s/.* install-html/#/g
s/.* install-pdf/#/g
s:#.*/toolchain/gcc_first/postinstall:if [ "" ] ; then:g
s:#.*/toolchain/newlib/configure:fi:g
s:#.*/toolchain/newlib/postinstall:if [ "" ] ; then:g
s:#.*/toolchain/gcc_final/configure:fi:g
s:#.*/toolchain/gcc_final/postinstall:if [ "" ] ; then:g
s:#.*/toolchain/zlib/0/copy:fi:g
s:#.*/toolchain/gdb/0/postinstall:if [ "" ] ; then:g
s:#.*/finalize_libc_installation:fi:g
s:rm ./lib/libiberty.a:rm -f -r ./lib/x86_64:g
s:/usr/local/tools/gcc.*/bin/strip :strip :g
s/^strip.*arm-none-eabi-sprite/#/g
s:^# task.*/package_tbz2:exit 0:g
s:--with-newlib:--with-newlib --enable-target-optspace --disable-libunwind-exceptions --enable-cxx-flags="-g -Os -fno-unroll-loops -ffunction-sections -fdata-sections -fomit-frame-pointer -DPREFER_SIZE_OVER_SPEED -D__OPTIMIZE_SIZE__ -fshort-wchar -fno-exceptions" CFLAGS_FOR_TARGET="-g -Os -fno-unroll-loops -ffunction-sections -fdata-sections -fomit-frame-pointer -DPREFER_SIZE_OVER_SPEED -D__OPTIMIZE_SIZE__ -fshort-wchar -fno-exceptions":
s/'-g -O2 -fno-unroll-loops'/'-g -Os -fno-unroll-loops -ffunction-sections -fdata-sections -DPREFER_SIZE_OVER_SPEED -D__OPTIMIZE_SIZE__ -fomit-frame-pointer -D__BUFSIZ__=128 -DSMALL_MEMORY -fshort-wchar'/
s/--disable-newlib-supplied-syscalls/--disable-newlib-supplied-syscalls --enable-newlib-reent-small --disable-newlib-atexit-alloc/
s/# task/echo task/g
