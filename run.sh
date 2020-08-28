cd build

cmake  ../ -G "Unix Makefiles" -DENABLE_MONTAGE_DEVICES=ON \
-DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors \
-DCLANG:FILEPATH=/home/eawang/_worklib/bin/clang \
-DCLANGXX:FILEPATH=/home/eawang/_worklib/bin/clang++ \
-DLLVM_AS:FILEPATH=/home/eawang/_worklib/bin/llvm-as \
-DLLVM_CONFIG:FILEPATH=/home/eawang/_worklib/bin/llvm-config \
-DLLVM_LINK:FILEPATH=/home/eawang/_worklib/bin/llvm-link \
-DLLVM_LLC:FILEPATH=/home/eawang/_worklib/bin/llc \
-DLLVM_LLI:FILEPATH=/home/eawang/_worklib/bin/lli \
-DLLVM_OPT:FILEPATH=/home/eawang/_worklib/bin/opt 

make -j8

sudo make install
cd ..


