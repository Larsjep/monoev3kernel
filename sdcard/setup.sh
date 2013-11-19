#! /bin/bash
# Create a set of files to allow the easy creation of leJOS SD card images
# Created by Andy Shaw
LJGIT=${LEJOSGIT?"Lejos git repository location LEJOSGIT not set"}
LJHOME=lejosfs/home/root/lejos
img=lejosimage
rm -rf $img 2> /dev/null
mkdir $img 2> /dev/null
mkdir lejosfs/home/lejos/programs 2> /dev/null
mkdir $LJHOME/lib 2> /dev/null
mkdir $LJHOME/libjna 2> /dev/null
mkdir $LJHOME/mod 2> /dev/null
mkdir $LJHOME/btcache 2> /dev/null
mkdir $LJHOME/samples 2> /dev/null
mkdir $LJHOME/bin/utils 2> /dev/null
cp scripts/* $img
cp -r lejosfs $img
cp ../kernel/uImage $img
cp -r ../kernel/modules $img
cp external/lmsfs.tar.bz2 $img 
mkdir $img/netmods
cp ../modules/net/bin/* $img/netmods
mkdir $img/mod
cp external/Linux_AM1808/sys/mod/*.ko $img/mod 
cp ../modules/lms2012/bin/* $img/mod
cp external/Linux_AM1808/sys/wpa_supplicant $img/$LJHOME/bin
dpkg-deb -x external/libjna* $img/libjna
dpkg-deb -x external/libffi* $img/libjna
dpkg-deb -x external/firmware-brcm80211* $img/firmware
dpkg-deb -x external/firmware-libertas* $img/firmware
dpkg-deb -x external/firmware-ralink* $img/firmware
dpkg-deb -x external/firmware-realtek* $img/firmware
cd $img/lejosfs
dpkg-deb --fsys-tarfile ../../external/bridge-utils* | tar x ./usr/sbin/brctl
cd ../..
cp ${LJGIT}/ev3classes/ev3classes.jar $img
cp ${LJGIT}/DBusJava/dbusjava.jar $img
cp ${LJGIT}/EV3HelloWorld/bin/EV3HelloWorld.class $img/$LJHOME/samples
#cp ${LJGIT}/EV3Splash/bin/Splash.class $img/$LJHOME/bin/utils
cp ${LJGIT}/EV3Menu/dist/EV3Menu.jar $img/$LJHOME/bin/utils
cp ${LJGIT}/EV3Menu/src/wpa_supplicant.txt $img/$LJHOME/bin/utils
#cp ${LJGIT}/EV3PowerOff/bin/PowerOff.class $img/$LJHOME/bin/utils
git describe > $img/version
cp readme $img
tar cfj lejosimage.bz2 lejosimage
