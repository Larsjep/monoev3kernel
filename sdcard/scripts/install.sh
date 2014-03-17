#!/bin/sh
# install lejos on the new file system
. ./funcs.sh
rootfs=/media/rootfs
bootfs=/media/bootfs
mkdir $bootfs 2> /dev/null
mkdir $rootfs 2> /dev/null
mount $1 $bootfs
mount $2 $rootfs
LEJOS_HOME=$rootfs/home/root/lejos
log "Prepare install"
if [ ! -e $bootfs/lejosimage.bz2 ]
then
  error "Missing lejos install files"
fi
if [ ! -e $bootfs/ejre* ]
then
  error "Missing jre install files"
fi
log "Deleting old files"
rm -rf $rootfs/*
log "Expand image"
tar -C $rootfs -jxf $bootfs/lejosimage.bz2
if [ ! -e $rootfs/lejosimage ]
then
  error "Missing lejos image"
fi
current=${PWD}
cd $rootfs/lejosimage
log "Start install"
./update_sdcard.sh $bootfs $rootfs $current
log "Remove temp files"
cd $rootfs
rm -rf lejosimage
cd /
log "Installing kernel"
mv $bootfs/uImage $bootfs/uImageNoobs
mv $bootfs/uImageStandard $bootfs/uImage
log "Sync disks"
sync
sync
log "Unmount disks"
umount $rootfs
umount $bootfs
