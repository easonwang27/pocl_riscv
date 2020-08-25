cd build

cmake  ../ -G "Unix Makefiles" -DENABLE_MONTAGE_DEVICES=ON \
-DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors \
-DCLANG:FILEPATH=/home/eawang/work/llvm_lib/bin/clang \
-DCLANGXX:FILEPATH=/home/eawang/work/llvm_lib/bin/clang++ \
-DLLVM_AS:FILEPATH=/home/eawang/work/llvm_lib/bin/llvm-as \
-DLLVM_CONFIG:FILEPATH=/home/eawang/work/llvm_lib/bin/llvm-config \
-DLLVM_LINK:FILEPATH=/home/eawang/work/llvm_lib/bin/llvm-link \
-DLLVM_LLC:FILEPATH=/home/eawang/work/llvm_lib/bin/llc \
-DLLVM_LLI:FILEPATH=/home/eawang/work/llvm_lib/bin/lli \
-DLLVM_OPT:FILEPATH=/home/eawang/work/llvm_lib/bin/opt 

make -j16

sudo make install
cd ..


