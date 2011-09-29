#! /usr/bin/env bash
# Copyright 2011 Adam Green (http://mbed.org/users/AdamGreen/)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Simple script to setup Code Sourcery files to build on Mac OS X

# TODO: Will most likely need to change this for each build of tools.
# Expected location of files based on Code Sourcery script
src_path=/scratch/janisjo/arm-eabi-lite/src

# TODO: Will need to be customized for newer builds.
# Name of the tarball archive to be used for building the toolchain.
archive_name=arm-2011.03-42-arm-none-eabi.src.tar.bz2
archive_url=https://sourcery.mentor.com/sgpp/lite/arm/portal/package7812/public/arm-none-eabi/$archive_name
archive_md5=7c302162ec813d039b8388bd7d2b4176

# This script expects to be sitting in the same directory as the main archive
# from Code Sourcery so change into the directory before attempting to
# perform the extraction of the source code
SCRIPT_PATH=$0
SCRIPT_PATH=${SCRIPT_PATH%/*}
cd $SCRIPT_PATH

# Pull the tarball down from the Mentor Graphics/Code Sourcery website.
echo Downloading $archive_url
curl -LO $archive_url

echo Validating the MD5 signature on $archive_name
archive_match=`md5 -q $archive_name | grep -c $archive_md5`
if [ "$archive_match" != "1" ] ; then
    echo $archive_name failed MD5 signature check.
    exit 1
fi

# Extract the various tool sources from the provided archive
echo Extracting $archive_name
tar xf $archive_name
archive_out=${archive_name%%.src.tar.bz2}

# Now extract the sources for each tool from their respective archive
echo Extracting sources from $archive_out
pushd $archive_out >/dev/null
mkdir -p $src_path
for archive in `ls *.tar.bz2` ; do
    echo Extracting $archive
    tar -xf $archive -C $src_path
done

# Find the build bash script
scripts=(`ls arm-*-arm-none-eabi.sh`)
if [ "${#scripts[*]}" != "1" ] ; then
    echo This script only expects to find one build script in the package
    echo Actually found ${scripts[*]}
    exit 1
fi
script=${scripts[0]}

# Modify the script to skip portions not needed for Mac OS and
# replace the i686-pc-linux-gnu as appropriate for the Mac GCC toolchain
echo Creating modified bash build script appropriate for the Mac
sed -f ../build.sed $script >build.sh
chmod +x build.sh

# The build of GCC requires libgcc in /usr/lib without a version number so create a symbolic link.
if [[ ! (-a /usr/lib/libgcc_s.dylib) ]]; then 
    echo Creating symbolic link for /usr/lib/libgcc_s.dylib
    sudo ln -s /usr/lib/libgcc_s.10.5.dylib /usr/lib/libgcc_s.dylib
fi

echo Apply patches to the source code.
cp ../src.patch $src_path
cd $src_path
patch -p5 -N <src.patch

echo You should now be able to run the $archive_out/build.sh script to kick off the build.

popd >/dev/null