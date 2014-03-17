#! /bin/bash
#******************************************************************************************************************
#     COMPILE KERNEL
#******************************************************************************************************************
# as normal user on linux pc terminal:

echo
echo -------------------------------------------------------------------------------
echo BUILDING KERNEL
echo -------------------------------------------------------------------------------
echo
sleep 1

script=`readlink -f "$0"`
# Absolute path this script is in
project=`dirname "$script"`
echo $project
source "$project"/env_setup
PATH=${AM1808_COMPILER}:$PATH
PATH=${AM1808_UBOOT_DIR}/tools:$PATH

cd ${AM1808_KERNEL}

make distclean ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi-
cp pru-firmware-05-31-2011-1423-v3.0/PRU_SUART_Emulation.bin PRU/
cp pru-firmware-05-31-2011-1423-v3.0/PRU_SUART_Emulation.bin firmware/omapl_pru/

# build the noobs kernel
cp ${project}/LEGOBoardNoob.config .config
make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi-
make -j4 uImage ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi-
cp arch/arm/boot/uImage ${project}/uImageNoobs

# and the main kernel
cp ${project}/LEGOBoard.config .config
make ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi-
make -j4 uImage ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi-
cp arch/arm/boot/uImage ${project}/uImage


mkdir ${project}/modules 2> /dev/null
make modules_install INSTALL_MOD_PATH=${project}/modules ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi-

