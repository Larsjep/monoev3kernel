#!/bin/sh
# check the SD card basic layout and contents
. ./funcs.sh
. ./partfuncs.sh
device=$1
getDiskInfo $device
getNumberParts
numParts=$REPLY
# do basic sanity check we need to have 1 or 2 partitions
if [ $numParts -le  0 ]
then
  error "No partitions found"
fi

if [ $numParts -gt 2 ]
then
  error "Too many partitions"
fi
# check partition contents
rootfs=/media/rootfs
bootfs=/media/bootfs
mkdir $bootfs 2> /dev/null
mkdir $rootfs 2> /dev/null
mount $2 $bootfs 2> /dev/null
log "Check install"
if [ ! -e $bootfs/lejosimage.bz2 ]
then
  error "Missing lejos files"
fi
if [ ! -e $bootfs/ejre* ]
then
  error "Missing jre"
fi
if [ $numParts -eq 2 ]
then
  # save config files from existing setup
  log "Preserve config"
  mount $3 $rootfs 2> /dev/null
  if [ ! -f $bootfs/hostname ]
  then
    cp $rootfs/etc/hostname $bootfs > /dev/null
  fi
  if [ ! -f $bootfs/wpa_supplicant.conf ]
  then
    cp $rootfs/etc/wpa_supplicant.conf $bootfs 2> /dev/null
  fi
  cp $rootfs/home/root/lejos/bin/utils/wpa_supplicant.conf $bootfs/wpa_supplicant.lejos 2> /dev/null
  if [ ! -f $bootfs/netaddress ]
  then
    cp $rootfs/home/root/lejos/bin/netaddress $bootfs 2> /dev/null
  fi
fi
sync
sync
log "Unmount disks"
umount $rootfs 2> /dev/null
umount $bootfs 2> /dev/null
