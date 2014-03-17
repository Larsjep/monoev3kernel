#! /bin/bash
# build the base rootfs used as a base for both the noobs rootfs and the actual
# lejos system
basefs=basefs
sudo rm -rf $basefs 2> /dev/null
sudo mkdir $basefs 2> /dev/null
# extract the base file system
sudo tar -C "$basefs"/ -jxf external/lmsfs.tar.bz2
sudo chown root basefs
# delete lego files
sudo rm -r "$basefs"/home/root/lms2012
sudo rm -r "$basefs"/usr/sbin
# add parted and required packages
sudo dpkg-deb -x external/parted* $basefs
sudo dpkg-deb -x external/libparted* $basefs
sudo dpkg-deb -x external/libreadline* $basefs
sudo dpkg-deb -x external/libselinux* $basefs
sudo dpkg-deb -x external/libdevmapper* $basefs
sudo cp scripts/init $basefs
sudo rm rootfs.cpio 2> /dev/null
cd "$basefs"
sudo bash -c 'find . | cpio -o -H newc > ../rootfs.cpio'
cd ..
rm rootfs.cpio.gz 2> /dev/null
gzip rootfs.cpio
sudo rm -rf "$basefs"
