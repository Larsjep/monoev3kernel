#! /bin/bash
# Based on an original script created by Lego.
# leJOS additions Andy Shaw/Sven Kohler

echo
echo -------------------------------------------------------------------------------
echo UPDATE SDCARD WITH NEWEST KERNEL, FILESYSTEM AND APPLICATION          TCP120709
echo -------------------------------------------------------------------------------
echo
sudo -v
mount=${1:-/media/`id -u -r -n`}
jvm=${2:-`echo ejre*`}
ipaddress=${3:-10.0.1.1}
LJHOME=home/root/lejos
echo "Mount point is $mount"
echo Java is $jvm
echo "IP for BT and USB is $ipaddress"
echo
echo "  ...."checking.sdcard
sleep 10
set -e
current=${PWD}

if [[ -d "$mount/LMS2012" && -d "$mount/LMS2012_EXT" ]]
then

        echo "  ...."erasing.sdcard
        sudo rm -rf "$mount/LMS2012"/*
        sudo rm -rf "$mount/LMS2012_EXT"/*
        sync

        echo "  ...."copying.kernel.to.sdcard
        sudo cp uImage "$mount/LMS2012/uImage"
        sync

        echo "  ...."copying.filesystem.to.sdcard
	sudo tar -C "$mount/LMS2012_EXT" -jxf lmsfs.tar.bz2 
        sync

        echo "  ...."copying.application.to.sdcard
        sudo cp -r Linux_AM1808/* "$mount/LMS2012_EXT"/home/root/lms2012

        echo "  ...."copying.extra.modules.to.sdcard
	sudo rm -rf $mount/LMS2012_EXT/lib/modules/*
	sudo cp -r modules/* $mount/LMS2012_EXT/
        sudo cp -r netmods/* "$mount/LMS2012_EXT"/lib/modules/*/kernel/drivers/net/wireless/
        echo "  ...."force.depmod.on.first.boot
	sudo rm "$mount/LMS2012_EXT"/lib/modules/*/modules.dep

        echo "  ...."copying.lejos.to.sdcard
        sudo cp -r lejosfs/* "$mount/LMS2012_EXT"
	sudo cp wpa_supplicant.conf "$mount/LMS2012_EXT"/etc
	sudo cp ev3classes.jar "$mount/LMS2012_EXT"/$LJHOME/lib
	sudo cp Linux_AM1808/sys/mod/*.ko "$mount/LMS2012_EXT"/$LJHOME/mod
	sudo cp mod/*.ko "$mount/LMS2012_EXT"/$LJHOME/mod
	sudo cp version "$mount/LMS2012_EXT"/$LJHOME
	sudo sh -c "echo $ipaddress > '$mount/LMS2012_EXT'/$LJHOME/bin/netaddress"
	pushd "$mount/LMS2012_EXT" > /dev/null
	cd bin
	sudo ln -s ../$LJHOME/bin/jrun jrun
	cd ../etc/rc0.d
	sudo ln -s ../init.d/lejos K09lejos
	sudo ln -s ../init.d/lejosunload S89lejosunload
	cd ../rc5.d
	sudo ln -s ../init.d/dropbear S81dropbear
	sudo ln -s ../init.d/lejos S98lejos
	popd > /dev/null
	sudo rm "$mount/LMS2012_EXT"/var/lib/bluetooth
	sudo mkdir "$mount/LMS2012_EXT"/var/lib/bluetooth
    	if [ -e $jvm ]
    	then
        	echo "  ....  "Java
		sudo tar -C "$mount/LMS2012_EXT/$LJHOME" -axf $jvm 
        else
                echo "   WARNING: file $jvm does not exist"
	fi
	if [ -e libjna ]
	then
        	echo "  ....  "Jna
		sudo cp -r libjna "$mount/LMS2012_EXT"/$LJHOME
	fi

        echo "  ...."writing.to.sdcard
        sync

        echo
        echo REMOVE sdcard

else

    echo
    echo SDCARD NOT PROPERLY FORMATTED !!!

fi
echo
echo -------------------------------------------------------------------------------
echo

