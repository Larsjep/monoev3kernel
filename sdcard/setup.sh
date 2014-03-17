#! /bin/bash
# Create a set of files to allow the easy creation of leJOS SD card images
# Created by Andy Shaw
LJGIT=${LEJOSGIT?"Lejos git repository location LEJOSGIT not set"}
LJHOME=lejosfs/home/root/lejos
img=lejosimage
rm -rf $img 2> /dev/null
mkdir $img 2> /dev/null
mkdir lejosfs/home/lejos 2> /dev/null
mkdir lejosfs/home/lejos/programs 2> /dev/null
mkdir $LJHOME/lib 2> /dev/null
mkdir $LJHOME/libjna 2> /dev/null
mkdir $LJHOME/mod 2> /dev/null
mkdir $LJHOME/btcache 2> /dev/null
mkdir $LJHOME/samples 2> /dev/null
mkdir $LJHOME/bin/utils 2> /dev/null
cp scripts/update_sdcard.sh $img
cp -r lejosfs $img
cp scripts/funcs.sh $img/lejosfs/home/root/lejos/bin
cp scripts/spinner.sh $img/lejosfs/home/root/lejos/bin
cp -r ../kernel/modules $img
cp external/lmsfs.tar.bz2 $img 
mkdir $img/netmods
cp ../modules/net/bin/* $img/netmods
mkdir $img/mod
cp ../modules/lms2012/bin/* $img/mod
cp external/wpa_supplicant $img/$LJHOME/bin
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
cp ${LJGIT}/EV3Menu/dist/EV3Menu.jar $img/$LJHOME/bin/utils
cp ${LJGIT}/EV3Menu/src/wpa_supplicant.txt $img/$LJHOME/bin/utils
cp ${LJGIT}/EV3BumperCar/dist/BumperCar.jar $img/$LJHOME/samples
cp ${LJGIT}/EV3GraphicsTest/dist/GraphicsTest.jar $img/$LJHOME/samples
cp ${LJGIT}/EV3SensorTest/dist/SensorTest.jar $img/$LJHOME/samples
git describe > $img/version 2> /dev/null
cp readme $img
rm -rf zipimage 2> /dev/null
mkdir zipimage
tar cfj zipimage/lejosimage.bz2 lejosimage
cp $img/version zipimage
cp ../kernel/uImage zipimage/uImageStandard
cp ../kernel/uImageNoobs zipimage/uImage
mkdir zipimage/lejos
mkdir zipimage/lejos/images
mkdir zipimage/lejos/bin
cp scripts/partition.sh zipimage/lejos/bin
cp scripts/funcs.sh zipimage/lejos/bin
cp scripts/install.sh zipimage/lejos/bin
cp scripts/spinner.sh zipimage/lejos/bin
cp scripts/partfuncs.sh zipimage/lejos/bin
cp scripts/check.sh zipimage/lejos/bin
cp scripts/lejoslogo.ev3i zipimage/lejos/images

rm lejosimage.zip 2> /dev/null
cd zipimage
zip -r ../lejosimage.zip *
cd ..
