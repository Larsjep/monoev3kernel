#! /bin/bash

echo
echo "  ...."creating sd.img file
sudo umount sdmount/LMS2012
sudo umount sdmount/LMS2012_EXT
rm sd.img
dd if=/dev/zero of=sd.img bs=516096c count=2000

echo
echo "  ...."creating patitions
sudo fdisk -u -C2000 -S63 -H16 sd.img < fdisk.cmd &>> sdimage.err

sleep 2

echo
echo "  ...."creating loop devices
sudo losetup -d /dev/loop0
sudo losetup -d /dev/loop1
sudo losetup -o1048576 --sizelimit 52428800 /dev/loop0 sd.img
sudo losetup -o53477376 /dev/loop1 sd.img

echo
echo "  ...."making.kernel.partition
sudo mkfs.msdos -n LMS2012 /dev/loop0 &>> sdimage.err

sleep 2

echo
echo "  ...."making.filesystem.partition
sudo mkfs.ext3 -L LMS2012_EXT /dev/loop1 &>> sdimage.err

sleep 2

echo
echo "  ...."mount file systems
sync

mkdir sdmount
mkdir sdmount/LMS2012
mkdir sdmount/LMS2012_EXT

sudo mount /dev/loop0 sdmount/LMS2012
sudo mount /dev/loop1 sdmount/LMS2012_EXT

df -Th

echo
echo "  ...."update the sdcard image
./update_sdcard.sh sdmount $1

echo
echo "  ...."zip the image file
zip sd.zip sd.img



