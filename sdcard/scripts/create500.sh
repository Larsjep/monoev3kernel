#! /bin/bash

echo
echo "  ...."creating sd500.img file
sudo umount sdmount/SD500 2>/dev/null
rm sd500.img 2>/dev/null
dd if=/dev/zero of=sd500.img bs=516096c count=1000

echo
echo "  ...."creating partitions
sudo fdisk -u -C1000 -S63 -H16 sd500.img <fdisk500.cmd &>> sdimage.err

sleep 2

echo
echo "  ...."creating loop devices
sudo losetup -d /dev/loop0 2>/dev/null
sudo losetup -o1048576 /dev/loop0 sd500.img

echo
echo "  ...."making.kernel.partition
sudo mkfs.msdos -F 32 -n SD500 /dev/loop0 &>> sdimage.err

sleep 2

echo
echo "  ...."mount file systems
sync

rm -r sdmount 2>/dev/null
mkdir sdmount
mkdir sdmount/SD500

sudo mount /dev/loop0 sdmount/SD500

echo
echo "  ...."zip the image file
zip sd500.zip sd500.img



