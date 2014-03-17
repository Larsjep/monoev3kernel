#! /bin/sh
. $3/funcs.sh
bootfs=$1
rootfs=$2
jvm=`echo $bootfs/ejre*`
ipaddress=10.0.1.1
LJHOME=home/root/lejos
echo Java is $jvm
current=${PWD}
log "Installing rootfs"
tar -C "$rootfs" -jxf lmsfs.tar.bz2 
rm "$rootfs"/etc/mtab
rm "$rootfs"/etc/wpa_supplicant.conf
sync
log "Installing modules"
rm -rf $rootfs/lib/modules/*
cp -r modules/* $rootfs
cp -r netmods/* "$rootfs/"/lib/modules/*/kernel/drivers/net/wireless/
cp -r firmware/* "$rootfs"/
rm "$rootfs"/lib/modules/*/modules.dep
sync
log "Installing leJOS"
rm -rf "$rootfs"/home/root/lms2012
cp -r lejosfs/* "$rootfs"
cp ev3classes.jar "$rootfs"/$LJHOME/lib
cp dbusjava.jar "$rootfs"/$LJHOME/lib
cp mod/*.ko "$rootfs"/$LJHOME/mod
cp version "$rootfs"/$LJHOME
sync
log "Configure network"
sh -c "echo $ipaddress > '$rootfs'/$LJHOME/bin/netaddress"
log "Install links"
cd $rootfs/bin
ln -s ../$LJHOME/bin/jrun jrun
cd ../etc/rc0.d
ln -s ../init.d/lejos K09lejos
ln -s ../init.d/lejosunload S89lejosunload
cd ../rc5.d
ln -s ../init.d/dropbear S81dropbear
ln -s ../init.d/lejos S98lejos
cd $current
rm "$rootfs"/var/lib/bluetooth
mkdir "$rootfs"/var/lib/bluetooth
log "Install libjna"
cp -r libjna "$rootfs"/$LJHOME
log "Copy config files"
cp $bootfs/wpa_supplicant.conf "$rootfs"/etc 2> /dev/null
cp $bootfs/wpa_supplicant.lejos $rootfs/$LJHOME/bin/utils/wpa_supplicant.conf 2> /dev/null
cp $bootfs/hostname $rootfs/etc 2> /dev/null
cp $bootfs/netaddress $rootfs/$LJHOME/bin 2> /dev/null
ls $bootfs
cd $bootfs
ls
pwd
newjre=$(echo ejre*)
echo "jre is $newjre"
if [ -f $newjre ];
then
  log "Install jre"
  rm -rf $rootfs/$LJHOME/ejre* 2> /dev/null
  log "extracting jre"
  tar -C $rootfs/$LJHOME -zxf ${newjre}
  echo ${newjre} > $rootfs/$LJHOME/jrever
  rm $rootfs/$LJHOME/jreopt
fi
cd $rootfs/$LJHOME
if [ ! -f jreopt ];
then
  log "Optimize java"
  $rootfs/$LJHOME/ejre*/bin/java -client -Xshare:dump
  touch jreopt
fi

sync
