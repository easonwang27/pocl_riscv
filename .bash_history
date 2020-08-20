ls
ls -a
sudo vim /etc/sudoers
ls
sudo ls
ls /home
ls
tar -xf llvm-10.x.tar 
ls
cd llvm-10.x/
ls
cd build/
ls
rm -rf *
ld.lld -v
clang -v
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm10 -DLLVM_TARGETS_TO_BUILD=RISCV -DLLVM_ENABLE_PROJECTS=clang -DCMAKE_BUILD_TYPE=Release ../llvm 
cmake -LH
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm10 -DLLVM_TARGETS_TO_BUILD=RISCV -DLLVM_ENABLE_PROJECTS=clang\;lld -DCMAKE_BUILD_TYPE=Release ../llvm 
cd ..
git branch -a
cd build/
ls ~
ls
cd llvm-10.x/
cd build/
sudo make -j16
sudo make install
cd ~
ls
cd llvm10
ls bin
cd bin
ld.lld -v
./ld.lld --version
clang -v
cd ~
du -s *
ls
tar -xf pocl-1.5.tar 
ls
cd pocl-1.5/
ls
cd build/
ls
rm -rf *
cmake  ../ -G "Unix Makefiles" -DLLC_HOST_CPU=rv64gc -DLLC_TRIPLE=riscv64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/llvm10/bin/clang -DCLANGXX:FILEPATH=/home/fei/llvm10/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/llvm10/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/llvm10/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/llvm10/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/llvm10/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/llvm10/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/llvm10/bin/opt
cd ~
ls
cd llvm-10.x/
cd build/
ls
rm -rf *
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm10 -DLLVM_TARGETS_TO_BUILD=RISCV -DLLVM_ENABLE_PROJECTS=clang\;lld -DCMAKE_BUILD_TYPE=Release ../llvm 
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm10 -DLLVM_ENABLE_PROJECTS=clang\;lld -DCMAKE_BUILD_TYPE=Release ../llvm 
rm -rf *
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm10 -DLLVM_ENABLE_PROJECTS=clang\;lld -DCMAKE_BUILD_TYPE=Release ../llvm 
rm -rf *
sudo rm -rf *
ls
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm10 -DLLVM_TARGETS_TO_BUILD=RISCV\;X86\;NVPTX -DLLVM_ENABLE_PROJECTS=clang\;lld -DCMAKE_BUILD_TYPE=Release ../llvm 
sudo rm -rf  ~/llvm10
ls ~
sudo rm -rf  ~/pocl-1.5.tar 
ls ~
sudo make -j8
sudo make install
cd ~
ls
cd pocl-1.5/
cd build/
ls
rm -rf *
cmake  ../ -G "Unix Makefiles" -DLLC_HOST_CPU=rv64gc -DLLC_TRIPLE=riscv64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/llvm10/bin/clang -DCLANGXX:FILEPATH=/home/fei/llvm10/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/llvm10/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/llvm10/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/llvm10/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/llvm10/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/llvm10/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/llvm10/bin/opt
make -j8
~/llvm10/bin/clang -v
~/llvm10/bin/ld.lld --version
exit
cd pocl-1.5/build/
ls
sudo make install
export POCL_DEBUG=all
export POCL_DEVICES="basic"
./examples/vecadd/vecadd 
cd ~/ww
ls
ls ~
mkdir ~/ww
./examples/vecadd/vecadd 
ls ~/ww
~/llvm10/bin/llvm-objdump -d ~/ww/objfile.o 
exit
cd ~
ls
mv llvm-10.x llvm-10.x-code
ls
mv llvm-10.x.tar llvm-10.x-code.tar
ls
rm examples.desktop 
ls
du -s -g *
du -s -m *
du -h
sudo fdisk -l
free
df -l
df -l -m
df -l -g
df -l -m
ls /
ls
du -s
du -s -m
ls
tar -czf llvm10.tar.gz llvm10
ls
tar -xf llvm-rvv-code.tar 
cd llvm-rvv/
git tag
git branch -a
cd ~/ww
ls
../llvm10/bin/ld.lld -o obj.so objfile.o -shared
ls
../llvm10/bin/llvm-objdump -d obj.so
../llvm10/bin/llvm-readelf -a obj.so
exit
ls
rm llvm-rvv-code.tar 
./llvm10/bin/clang -v
cd llvm-rvv/
ls
mkdir mk
cd mk
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm8-rvv -DLLVM_TARGETS_TO_BUILD=RISCV\;X86 -DLLVM_ENABLE_PROJECTS=clang\;lld -DCMAKE_BUILD_TYPE=Release ../llvm 
sudo make -j10
make install
cd ~
ls
cd llvm8-rvv/
./bin/clang -v
cd ~
ls
mv llvm8-rvv/ llvm10-rvv-x86
ls
./llvm10-rvv-x86/bin/clang -v
du -s -m *
tar -czf llvm10-rvv-x86.tar.gz llvm10-rvv-x86/
ls
du -s -m *
exit
top
ls
rm -rf llvm-rvv/
sudo rm -rf llvm-rvv/
exit
whereis tar
exit
top
cd /
ls
sudo tar -cpzf boot.tar.gz boot
ls
du -s -m *
sudo du -s -m *
sudo tar -cpzf bin.tar.gz bin
sudo tar -cpzf usr.tar.gz usr
ls
ls var
sudo tar -cpzf var.tar.gz var
ls
sudo tar -cpzf etc.tar.gz etc
sudo tar -cpzf sbin.tar.gz sbin
sudo tar -cpzf lib.tar.gz lib
sudo tar -cpzf lib32.tar.gz lib32
sudo tar -cpzf lib64.tar.gz lib64
sudo tar -cpzf srv.tar.gz srv
sudo tar -cpzf proc.tar.gz proc
ls
ls proc
ls /proc/1995
ls
du -s -m *
sudo du -s -m *
ls
sudo rm proc.tar.gz 
sudo tar -cpzf opt.tar.gz opt
sudo tar -cpzf root.tar.gz root
sudo tar -cpzf run.tar.gz run
ls
ls -a
ls -a ~
ls cdrom/
sudo mkdir backup
ls
sudo mv *.tar.gz backup/
ls
cd backup/
ls
sudo du -s
cd ..
sudo du -s
cd home/
sudo du -s
cd ~
ls ~
ls -a
ls ./.*
sudo tar -cpzf .bash_history.tar.gz .bash_history 
sudo tar -cpzf .bash_logout.tar.gz .bash_logout
sudo tar -cpzf .bashrc.tar.gz .bashrc
sudo tar -cpzf .profile.tar.gz .profile
sudo tar -cpzf .wget-hsts.tar.gz .wget-hsts
ls
ls -a
ls
exit
cd ww
ls
dtc -I dtb -O dts -o bcm2710-rpi-3-b.dts bcm2710-rpi-3-b.dtb
sudo apt install device-tree-compiler 
dtc -I dtb -O dts -o bcm2710-rpi-3-b.dts bcm2710-rpi-3-b.dtb
ls
wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-a/9.2-2019.12/binrel/gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu.tar.xz
top
sudo apt install git bc bison flex libssl-dev make libc6-dev libncurses5-dev
exit
top
ls
du -s -m *
dd if=/dev/zero of=v-ext4.img bs=1M count=1024
mkfs.ext4 v-ext4.img
mount -o loop v-fat32.img ~/mnt/fat32
sudo mount -o loop v-fat32.img ~/mnt/fat32
sudo mount -o loop v-ext4.img ~/mnt/ext4
ls /mnt
ls /dev
sudo unmount v-ext4.img
sudo umount v-ext4.img
sudo umount /mnt/ext4
ls
rm *.img
rm -rf mnt
sudo rm -rf mnt
sudo umount /mnt/ext4
sudo umount -l /mnt/ext4
exit
top
sudo umount -l /mnt/ext4
sudo rm -rf mnt
top
du -s -m *
tar -cf rpi-4.11.y.tar rpi-4.11.y/
ls
cd tools/
ls
cd arm-bcm2708/
ls
tar -cf arm-rpi-4.9.3-linux-gnueabihf.tar arm-rpi-4.9.3-linux-gnueabihf/
ls
cd ..
ls
ls mkimage/
ls
cd ~/rpi-4.11.y/
ls
ls scripts
ls arch/arm/boot
ls -l arch/arm/boot/zImage 
echo $(nproc)
cd ..
ls
rm -rf rpi-4.11.y
tar -xf rpi-4.11.y.tar 
cd rpi-4.11.y/
cd ..
rm -rf rpi-4.11.y
ls
ls -l
tar -xf rpi-4.11.y.tar 
cd rpi-4.11.y/
ls
cd ../tools/mkimage/
ls
./imagetool-uncompressed.py ~/rpi-4.11.y/arch/arm/boot/zImage
ls
cd ~
ls
exit
ls
cd rpi-4.11.y/
ls
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- menuconfig
ls arch/arm/boot
scripts/mkknlimg --help
scripts/mkknlimg -h
scripts/mkknlimg 
cd ..
ls
cd tools
ls
cd mkimage/
ls
./imagetool-uncompressed.py ../../rpi-4.11.y/arch/arm/boot/zImage
ls
ls ../../rpi-4.11.y/arch/arm/boot/zImage
ls ../../rpi-4.11.y/arch/arm/boot
ls -l
date
cd ~/r
ls ~/rpi-4.11.y/drivers/video/logo/
ls
ls -s
du -s -m
cd rpi-4.11.y/
sudo make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean 
export PATH=$PATH:$HOME/tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf/bin
sudo make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean 
arm-linux-gnueabihf-gcc -v
sudo make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean 
source ~/.bashrc
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean 
du -s -m
find ./ -name "bcm2709_defconfig"
ls /arch/arm/configs
ls ./arch/arm/configs
ls ./arch/
ls ./arch/arm64/
ls ./arch/arm64/configs/
make ARCH=arm64 CROSS_COMPILE=arm-linux-gnueabihf- bcmrpi3_defconfig
make -j10 ARCH=arm64 CROSS_COMPILE=arm-linux-gnueabihf- zImage dtbs
exit
top
exit
echo PATH=\$PATH:~/tools/arm-bcm2708/arm-linux-gnueabihf/bin >> ~/.bashrc
source ~/.bashrc
arm-linux-gnueabihf-gcc -v
cat tools/arm-bcm2708/arm-rpi-4.9.3-linux-gnueabihf
cat tools/arm-bcm2708/arm-linux-gnueabihf
vim tools/arm-bcm2708/arm-linux-gnueabihf
ls -l tools/arm-bcm2708/arm-linux-gnueabihf
cd tools/arm-bcm2708/
ln -snf arm-bcm2708hardfp-linux-gnueabi/ arm-linux-gnueabihf
ls -l arm-linux-gnueabihf
cd ~
arm-bcm2708hardfp-linux-gnueabi-gcc -v
cd rpi-4.11.y/
make clean
make ARCH=arm64 CROSS_COMPILE=arm-bcm2708hardfp-linux-gnueabi- clean
make ARCH=arm64 CROSS_COMPILE=arm-bcm2708hardfp-linux-gnueabi- bcmrpi3_defconfig
make -j10 ARCH=arm64 CROSS_COMPILE=arm-bcm2708hardfp-linux-gnueabi- zImage dtbs
ln -snf ~/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/ ~/tools/arm-bcm2708/arm-linux-gnueabihf
whereis arm-linux-gnueabihf-gcc
sudo make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean 
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean 
make ARCH=arm64 CROSS_COMPILE=arm-linux-gnueabihf- bcmrpi3_defconfig
make ARCH=arm64 CROSS_COMPILE=arm-linux-gnueabihf- menuconfig
KERNEL=kernel8
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean 
make ARCH=arm64 CROSS_COMPILE=arm-linux-gnueabihf- bcmrpi3_defconfig
make ARCH=arm64 CROSS_COMPILE=arm-linux-gnueabihf- menuconfig
make -j10 ARCH=arm64 CROSS_COMPILE=arm-linux-gnueabihf- zImage dtbs
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- clean 
make ARCH=arm64 CROSS_COMPILE=arm-linux-gnueabihf- bcm2709_defconfig
ls arch/arm64/configs/
exit
top
ls
cd ww
ls
rm gcc-arm-9.2-2019.12-x86_64-aarch64-none-linux-gnu.tar.xz 
ls
rm bcm*
ls
clang --target=riscv64 -march=rv64gfdc -emit-llvm -c fun.c
../llvm10/bin/clang --target=riscv64 -march=rv64gfdc -emit-llvm -c fun.c
../llvm10/bin/clang --target=riscv64 -march=rv64gdc -emit-llvm -c fun.c
../llvm10/bin/clang --target=riscv64 -march=rv64gc -emit-llvm -c fun.c
ls
llc -filetype=obj fun.bc -o fun.o
../llvm10/bin/llc -filetype=obj fun.bc -o fun.o
ls
../llvm10/bin/llvm-objdump -d fun.o
../llvm10/bin/llvm-objdump -d -mattr=+f fun.o
top
ls
cd rpi-4.11.y/
ls
cd drivers/
ls
ls spi
cd ..
ls
/ *.o
cd ..
ls
rm -rf rpi-4.11.y
tar -xf rpi-4.11.y.tar 
ls
cd rpi-4.11.y/
ls drivers/spi
ls drivers/uart
ls drivers
ls drivers/tty/
ls drivers/usb
ls drivers/gpio
exit
free
lspci
lspci -n
lspci -t
lscpu
free -lh
ls pci -vh 10ee
lspci -vh 10ee
lspci -vt 10ee
lspci -vs 10ee
lspci -vd 10ee
lspci -vb 10ee
lspci -vk 10ee
lspci -vd 10ee:
exit
lspci -vb 10ee:
lspci -vd 10ee:
ls
lspci -vd 10ee:
lspci
lspci -vd 10ee:
lspci grep | xilinx
lspci | grep xilinx
lspci | grep "xilinx"
lspci | grep Xilinx
ls
rm Videos/
rm -r Videos/
rm -r Pictures/
rm -r Public/
rm -r Documents/
rm -r Downloads/
ls
ls Templates/
rm -r Templates/
rm -r Music/
ls
rm rpi-4.11.y.tar 
rm -rf mnt
ls
ls Desktop/
rm -rf Desktop/
ls
exit
lspci | grep Xilinx
lspci -vd 10ee:
exit
cd build/
make
sudo make install
make
top
ls
cd llvm10-rvv-x86/
ls
cd ..
find -name IR
ls llvm10-rvv-x86/include/llvm/IR
cat llvm10-rvv-x86/include/llvm/IR/IntrinsicsRISCV.td 
cat llvm10-rvv-x86/include/llvm/IR/IntrinsicsARM.td 
cd llvm-10.x-code/
find -name Intrinsic
find -name Intrin
find -name IR
cd ..
cat llvm10-rvv-x86/include/llvm/IR/IntrinsicsMips.td 
ls /home/eawang/work/
cp /home/eawang/work/riscv-llvm-riscv-release_50.zip ./
ls
unzip riscv-llvm-riscv-release_50.zip 
ls
cd riscv-llvm-riscv-release_50/
ls
mkdir mk
cd mk
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm5 -DLLVM_TARGETS_TO_BUILD=RISCV -DLLVM_ENABLE_PROJECTS=clang -DCMAKE_BUILD_TYPE=Release ../llvm 

cmake ../llvm
cd ..
ls
./configure --help
ls -a
cmake
./configure --prefix=~/llvm5
make
cd ..
ls 
rm -rf riscv-llvm-riscv-release_50*
ls
exit
ls
cat README
cat README | grep pocl
ls
./llvm10-rvv-x86/bin/llvm-mc -mcpu=help
./llvm10-rvv-x86/bin/llvm-mc -mcpu=help ./ww/fun.c
lspci -vd 10ee:
exit
TOP
top
ls
cd llvm-10.x-code/
ls
cd build/
ls
rm -rf *
sudo rm -rf *
ls
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm-arm -DLLVM_TARGETS_TO_BUILD=ARM -DLLVM_ENABLE_PROJECTS=clang -DCMAKE_BUILD_TYPE=Release ../llvm 
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm-arm -DLLVM_ENABLE_PROJECTS=clang -DCMAKE_BUILD_TYPE=Release ../llvm 
sudo rm -rf *
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm-arm -DLLVM_ENABLE_PROJECTS=clang -DCMAKE_BUILD_TYPE=Release ../llvm 
sudo rm -rf *
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm-AArch64 -DLLVM_TARGETS_TO_BUILD=AArch64 -DLLVM_ENABLE_PROJECTS=clang -DCMAKE_BUILD_TYPE=Release ../llvm 
make -j10
make install
cd ~
ls
cd llvm-AArch64/
du -s
./bin/llvm-mc -triple=help ~/ww/fun.c
./bin/llvm-mc --triple=help ~/ww/fun.c
./bin/llvm-mc --triple=h ~/ww/fun.c
./bin/llvm-mc --triple=arm ~/ww/fun.c
./bin/llvm-mc --triple
./bin/llvm-mc --help
./bin/llvm-mc --version
./bin/llvm-mc -mcpu=help
./bin/llvm-mc --mcpu=help
./bin/llvm-mc --triple=aarch64 -mcpu=help
./bin/llvm-mc --triple=aarch64 -mcpu=help ~/ww/fun.c

ls
./llvm10-rvv-x86/bin/llvm-mc --triple=riscv64 -mcpu=help ~/ww/fun.c
exit
ls
exit
ls
cd ww
ls
~/llvm10/bin/llvm-objdump -d objfile.o 
exit
top
ls
cd ww
las
ls
~/llvm10-rvv-x86/bin/llvm-objdump -d -mattr=+v fun.o
~/llvm10-rvv-x86/bin/llvm-objdump -d -mattr=+f fun.o
rm fun.o
ls
rm fun.bc 
ls
~/llvm10-rvv-x86/bin/clang --target=riscv64 -march=rv64gfdc -emit-llvm -c fun.c
~/llvm10-rvv-x86/bin/clang --target=riscv64 -march=rv64gdc -emit-llvm -c fun.c
~/llvm10-rvv-x86/bin/clang --target=riscv64 -march=rv64gc -emit-llvm -c fun.c
ls
~/llvm10-rvv-x86/bin/llc -filetype=obj fun.bc -o fun.o
~/llvm10-rvv-x86/bin/llvm-objdump -d -mattr=+f fun.o
~/llvm10-rvv-x86/bin/llvm-mc -triple=riscv64 -filetype=obj -mattr=+f fun.c -o fun.o
../llvm10/bin/clang --target=riscv64 -march=rv64gc -emit-llvm -c fun.c
~/llvm10/bin/llc -filetype=obj fun.bc -o fun.o
~/llvm10-rvv-x86/bin/llvm-objdump -d -mattr=+f fun.o
cd ..
ls
cd pocl-1.5/build/
ls
rm -rf *
ls
cmake  ../ -G "Unix Makefiles" -DLLC_TRIPLE=aarch64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/llvm-AArch64/bin/clang -DCLANGXX:FILEPATH=/home/fei/llvm-AArch64/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/llvm-AArch64/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/llvm-AArch64/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/llvm-AArch64/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/llvm-AArch64/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/llvm-AArch64/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/llvm-AArch64/bin/opt
cmake  ../ -G "Unix Makefiles" -DLLC_HOST_CPU=cortex-a53 -DLLC_TRIPLE=aarch64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/llvm-AArch64/bin/clang -DCLANGXX:FILEPATH=/home/fei/llvm-AArch64/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/llvm-AArch64/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/llvm-AArch64/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/llvm-AArch64/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/llvm-AArch64/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/llvm-AArch64/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/llvm-AArch64/bin/opt
rm -rf *
cmake  ../ -G "Unix Makefiles" -DLLC_HOST_CPU=cortex-a53 -DLLC_TRIPLE=aarch64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/llvm-AArch64/bin/clang -DCLANGXX:FILEPATH=/home/fei/llvm-AArch64/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/llvm-AArch64/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/llvm-AArch64/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/llvm-AArch64/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/llvm-AArch64/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/llvm-AArch64/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/llvm-AArch64/bin/opt
cd ~
ls
cd llvm-10.x-code/
LS
ls
cd build/
ls
rm -rf *
cmake
cmake ../llvm
rm -rf *
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm-AArch64-x86 -DLLVM_TARGETS_TO_BUILD=AArch64\;X86 -DLLVM_ENABLE_PROJECTS=clang -DCMAKE_BUILD_TYPE=Release ../llvm 
exit
llvm10/bin/llc --version
llvm10/bin/clang --print-supported-archs
llvm10/bin/clang --print-supported-targets
exit
top
cd mk
rm -rf *
cmake  ../ -G "Unix Makefiles" -DLLC_HOST_CPU=rocket-rv64 -DLLC_TRIPLE=riscv64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/riscv-x86-llvm/bin/clang -DCLANGXX:FILEPATH=/home/fei/riscv-x86-llvm/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/riscv-x86-llvm/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/riscv-x86-llvm/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/riscv-x86-llvm/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/riscv-x86-llvm/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/riscv-x86-llvm/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/riscv-x86-llvm/bin/opt
make -j8
rm -rf *
cmake  ../ -G "Unix Makefiles" -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/riscv-x86-llvm/bin/clang -DCLANGXX:FILEPATH=/home/fei/riscv-x86-llvm/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/riscv-x86-llvm/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/riscv-x86-llvm/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/riscv-x86-llvm/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/riscv-x86-llvm/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/riscv-x86-llvm/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/riscv-x86-llvm/bin/opt
make -j12
apt install -y build-essential ocl-icd-libopencl1 cmake git pkg-config libclang-dev clang llvm make ninja-build ocl-icd-libopencl1 ocl-icd-dev ocl-icd-opencl-dev libhwloc-dev zlib1g zlib1g-dev clinfo dialog apt-utils
sudo apt install -y build-essential ocl-icd-libopencl1 cmake git pkg-config libclang-dev clang llvm make ninja-build ocl-icd-libopencl1 ocl-icd-dev ocl-icd-opencl-dev libhwloc-dev zlib1g zlib1g-dev clinfo dialog apt-utils
rm -rf *
cmake  ../ -G "Unix Makefiles" -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors
llc --version
sudo apt remove llvm-6.0
sudo apt remove clang-6.0
rm -rf *
cmake  ../ -G "Unix Makefiles" -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors
sudo apt install llvm-10
sudo apt install clang-10
rm -rf *
cmake  ../ -G "Unix Makefiles" -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors
clang++ --version
clang++-10 --version
cmake -LH | grep clang++
sudo apt install clang
cmake -LH | grep clang++
rm -rf *
cmake  ../ -G "Unix Makefiles" -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors
sudo apt install llvm
rm -rf *
cmake  ../ -G "Unix Makefiles" -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors
sudo apt install -y build-essential ocl-icd-libopencl1 cmake git pkg-config libclang-dev clang llvm make ninja-build ocl-icd-libopencl1 ocl-icd-dev ocl-icd-opencl-dev libhwloc-dev zlib1g zlib1g-dev clinfo dialog apt-utils
exit
ls
cd llvm-10.x-code/
cd build/
ls
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm-AArch64-x86 -DLLVM_TARGETS_TO_BUILD=AArch64\;X86 -DLLVM_ENABLE_PROJECTS=clang -DCMAKE_BUILD_TYPE=Release ../llvm 
make -j10
make install
cd ~
ls
rm -rf pocl-1.5/
llvm-AArch64-x86/bin/llc --version
rm -rf llvm-AArch64
ls
cd pocl/
cd mk/
~/llvm-AArch64-x86/bin/llvm-mc -triple=help -mcpu=help -mattr=+f ～/ww/fun.c
~/llvm-AArch64-x86/bin/llvm-mc --triple
~/llvm-AArch64-x86/bin/llvm-mc --version
~/llvm-AArch64-x86/bin/llvm-mc -triple=help -mcpu=help -mattr=+f ~/ww/fun.c
~/llvm-AArch64-x86/bin/llvm-mc -triple=aarch64 -mcpu=help -mattr=+f ~/ww/fun.c
~/llvm-AArch64-x86/bin/clang --target=aarch64 -march=help -emit-llvm -c ~/ww/fun.c
~/llvm-AArch64-x86/bin/clang --target=aarch64 -march= -emit-llvm -c ~/ww/fun.c
~/llvm-AArch64-x86/bin/clang --target=aarch64 -march=cortex-a53 -emit-llvm -c ~/ww/fun.c
~/llvm-AArch64-x86/bin/clang --target=aarch64 -march=cortex -emit-llvm -c ~/ww/fun.c
~/llvm-AArch64-x86/bin/clang --target=aarch64 --march -emit-llvm -c ~/ww/fun.c
~/llvm-AArch64-x86/bin/clang --target=aarch64 -emit-llvm -c ~/ww/fun.c

~/llvm-AArch64-x86/bin/llc -filetype=obj fun.bc -o fun.o
~/llvm-AArch64-x86/bin/llvm-objdump -d fun.o
cmake  ../ -G "Unix Makefiles" -DLLC_TRIPLE=aarch64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/llvm-AArch64-x86/bin/clang -DCLANGXX:FILEPATH=/home/fei/llvm-AArch64-x86/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/llvm-AArch64-x86/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/llvm-AArch64-x86/bin/opt
cmake  ../ -G "Unix Makefiles" -DLLC_HOST_CPU=cortex-a53 -DLLC_TRIPLE=aarch64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/llvm-AArch64-x86/bin/clang -DCLANGXX:FILEPATH=/home/fei/llvm-AArch64-x86/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/llvm-AArch64-x86/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/llvm-AArch64-x86/bin/opt
cmake  ../ -G "Unix Makefiles" -DLLC_TRIPLE=aarch64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/llvm-AArch64-x86/bin/clang -DCLANGXX:FILEPATH=/home/fei/llvm-AArch64-x86/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/llvm-AArch64-x86/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/llvm-AArch64-x86/bin/opt
cmake  ../ -G "Unix Makefiles" -DLLC_HOST_CPU=cortex-a53 -DLLC_TRIPLE=aarch64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/llvm-AArch64-x86/bin/clang -DCLANGXX:FILEPATH=/home/fei/llvm-AArch64-x86/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/llvm-AArch64-x86/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/llvm-AArch64-x86/bin/opt
make
cd ~
ls
cd llvm-10.x-code/build/
ls
cmake -LH
cmake  ../ -G "Unix Makefiles" -DLLC_HOST_CPU=cortex-a53 -DLLC_TRIPLE=aarch64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/llvm-AArch64-x86/bin/clang -DCLANGXX:FILEPATH=/home/fei/llvm-AArch64-x86/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/llvm-AArch64-x86/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/llvm-AArch64-x86/bin/lli \
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/llvm-AArch64-x86 -DLLVM_TARGETS_TO_BUILD=AArch64\;X86 -DLLVM_ENABLE_PROJECTS=clang -DCMAKE_BUILD_TYPE=Release ../llvm 
ls ~
LS
ls
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/aarch64-x86-llvm -DLLVM_TARGETS_TO_BUILD=AARCH64\;X86 -DLLVM_ENABLE_PROJECTS=clang\;libc\;libclc\;libcxx\;libcxxabi -DCMAKE_BUILD_TYPE=Release ../llvm 
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/aarch64-x86-llvm -DLLVM_TARGETS_TO_BUILD=AArch64\;X86 -DLLVM_ENABLE_PROJECTS=clang\;libc\;libclc\;libcxx\;libcxxabi -DCMAKE_BUILD_TYPE=Release ../llvm 
make -j12
rm -rf *
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/riscv-x86-llvm -DLLVM_TARGETS_TO_BUILD=RISCV\;X86 -DLLVM_ENABLE_PROJECTS=clang\;libc\;libclc\;libcxx\;libcxxabi -DCMAKE_BUILD_TYPE=Release ../llvm 
make -j12
rm -rf *
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/aarch64-x86-llvm -DLLVM_TARGETS_TO_BUILD=AArch64\;X86 -DLLVM_ENABLE_PROJECTS=clang\;clang-tools-extra\;compiler-rt\;libc\;libclc\;libcxx\;libcxxabi -DCMAKE_BUILD_TYPE=Release ../llvm 
make -j12
rm -rf *
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/riscv-x86-llvm -DLLVM_TARGETS_TO_BUILD=RISCV\;X86 -DLLVM_ENABLE_PROJECTS=clang\;clang-tools-extra\;compiler-rt\;libc\;libclc\;libcxx\;libcxxabi -DCMAKE_BUILD_TYPE=Release ../llvm 
make -j12
make -j8
make clean
make -j8
rm -rf *
cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX=/home/fei/riscv-x86-llvm -DLLVM_TARGETS_TO_BUILD=RISCV\;X86 -DLLVM_ENABLE_PROJECTS=clang\;libclc-DCMAKE_BUILD_TYPE=Release ../llvm 
make -j12
make install
ls ~
exit
ls
cd pocl/
cd mk/
ls
rm rf *
rm -rf *
cake ..
cmake ..
cmake  ../ -G "Unix Makefiles" -DLLC_HOST_CPU=skylake -DLLC_TRIPLE=riscv64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/riscv-x86-llvm/bin/clang -DCLANGXX:FILEPATH=/home/fei/riscv-x86-llvm/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/riscv-x86-llvm/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/riscv-x86-llvm/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/riscv-x86-llvm/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/riscv-x86-llvm/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/riscv-x86-llvm/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/riscv-x86-llvm/bin/opt
rm -rf *
cmake  ../ -G "Unix Makefiles" -DLLC_HOST_CPU=skylake -DLLC_TRIPLE=riscv64 -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DCLANG:FILEPATH=/home/fei/riscv-x86-llvm/bin/clang -DCLANGXX:FILEPATH=/home/fei/riscv-x86-llvm/bin/clang++ -DLLVM_AS:FILEPATH=/home/fei/riscv-x86-llvm/bin/llvm-as -DLLVM_CONFIG:FILEPATH=/home/fei/riscv-x86-llvm/bin/llvm-config -DLLVM_LINK:FILEPATH=/home/fei/riscv-x86-llvm/bin/llvm-link -DLLVM_LLC:FILEPATH=/home/fei/riscv-x86-llvm/bin/llc -DLLVM_LLI:FILEPATH=/home/fei/riscv-x86-llvm/bin/lli -DLLVM_OPT:FILEPATH=/home/fei/riscv-x86-llvm/bin/opt
sudo find /usr/lib  -name libclang-cpp.so
sudo find /usr/lib  -name "libclang-cpp.so"
sudo find /usr/lib  -name "libclang-cpp.so*

"
ls /usr/lib
ls /usr/lib/llvm-10/
ls /usr/lib/llvm-10/lib
rm -rf *
cmake  ../ -G "Unix Makefiles" -DPOCL_INSTALL_ICD_VENDORDIR=/etc/OpenCL/vendors -DC_LIBFILE_clang-cpp:FILEPATH=/usr/lib/llvm-10/lib/libclang-cpp.so.10
rm -rf *
cd ~/ww
ls -l
date
exit
