#! /bin/bash
# refresh the external components used to build a leJOS SD card image.
# Standard Lego files come from the standard Lego source layout
# Created by Andy Shaw
mkdir external 2> /dev/null
cd external
cp ~/projects/lms2012/open_first/lmsfs.tar.bz2 .
cp -r ~/projects/lms2012/lms2012/Linux_AM1808 .
wget -N -q http://ftp.debian.org/debian/pool/main/libf/libffi/libffi5_3.0.10-3_armel.deb
wget -N -q http://ftp.debian.org/debian/pool/main/libj/libjna-java/libjna-java_3.2.7-4_armel.deb
wget -N -q http://ftp.debian.org/debian/pool/main/b/bridge-utils/bridge-utils_1.4-5_armel.deb
wget -N -q http://ftp.debian.org/debian/pool/non-free/f/firmware-nonfree/firmware-realtek_0.40_all.deb
wget -N -q http://ftp.debian.org/debian/pool/non-free/f/firmware-nonfree/firmware-libertas_0.40_all.deb
wget -N -q http://ftp.uk.debian.org/debian/pool/non-free/f/firmware-nonfree/firmware-brcm80211_0.40_all.deb
wget -N -q http://ftp.debian.org/debian/pool/non-free/f/firmware-nonfree/firmware-ralink_0.40_all.deb
cd ..
