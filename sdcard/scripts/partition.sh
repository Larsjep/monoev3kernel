#!/bin/sh
. ./funcs.sh
. ./partfuncs.sh

log "Checking SD layout"
device=$1
getDiskInfo $device 
getNumberParts
numParts=$REPLY
getTotalSize
totalSize=$REPLY
echo $totalSize
# do basic sanity check we need to have 1 or 2 partitions and disk needs
# to be 2GB or more
if [ $numParts -le  0 ]
then
  error "no partitions found"
fi

if [ $numParts -gt 2 ]
then
  error "too many partitions"
fi

log "Check size"
if [ $totalSize -lt 1800 ]
then
  error "disk is too small"
fi

log "Check layout"
if [ $numParts -eq 1 ]
then
  # single partition, must be fat32
  getPartType 1
  partType=$REPLY
  if [ "$partType" != "fat32" ]
  then
    error "partion must be fat32"
  fi
  # we will resize the first partition to be 500Mb
  getPartStart 1
  start=$REPLY
  end=`echo $start 500 | awk '{print $1 + $2;}'`
  echo start $start end $end
  log "Resize FAT32 fs"
  parted -s -a optimal $device unit MB resize 1 $start $end 
  log "Resize complete"
  getDiskInfo $device 
  getPartEnd 1
  end=$REPLY
  echo end $end
  # now create the linux partition
  start=`expr $end + 1`
  start=`echo $end 1 | awk '{print $1 + $2;}'`
  end=`expr $totalSize - 1`
  end=`echo $totalSize 1 | awk '{print $1 - $2;}'`
  echo start $start end $end
  log "Create Linux fs"
  parted -s -a optimal $device unit MB mkpartfs primary ext2 $start $end 
  log "fs created"
fi 

