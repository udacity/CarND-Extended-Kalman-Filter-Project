#! /bin/bash
brew install openssl libuv cmake zlib
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
patch CMakeLists.txt < ../cmakepatch.txt
mkdir build
export PKG_CONFIG_PATH=/usr/local/opt/openssl/lib/pkgconfig 
cd build
# OPENSSL_VERSION=`openssl version -v | cut -d' ' -f2`
# in High Sierra, Apple uses LibreSSL so Homebrew doesn't install OpenSSL binary in /usr/local/bin
# this should work regardless of which OS version
OPENSSL_VERSION=`brew info openssl | head -1 | cut -d' ' -f3`
cmake -DOPENSSL_ROOT_DIR=$(brew --cellar openssl)/$OPENSSL_VERSION -DOPENSSL_LIBRARIES=$(brew --cellar openssl)/$OPENSSL_VERSION/lib -DOPENSSL_INCLUDE_DIR=$(brew --cellar openssl)/$OPENSSL_VERSION/include ..
make 
sudo make install
cd ..
cd ..
sudo rm -r uWebSockets
