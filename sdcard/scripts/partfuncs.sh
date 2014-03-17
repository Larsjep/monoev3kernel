getDiskInfo()
{
  parted -m $1 unit MB print > /tmp/disk
}

getTotalSize()
{
  REPLY=`awk -F: '{if (NR == (2)) {print substr($2, 1, index($2, "M")-1);}}' < /tmp/disk`
}

getNumberParts()
{
  REPLY=`wc /tmp/disk | awk '{print $1-2;}'`
}

getPartSize()
{
  REPLY=`awk -F: '{if (NR == ('$1'+2)) {print substr($4, 1, index($4, "M")-1);}}' < /tmp/disk`
  if [ -z "$REPLY" ]
  then
    REPLY=0
  fi
}

getPartType()
{
  REPLY=`awk -F: '{if (NR == ('$1'+2)) {print $5;}}' < /tmp/disk`
}

getPartStart()
{
  REPLY=`awk -F: '{if (NR == ('$1'+2)) {print substr($2, 1, index($2, "M")-1);}}' < /tmp/disk`
}

getPartEnd()
{
  REPLY=`awk -F: '{if (NR == ('$1'+2)) {print substr($3, 1, index($3, "M")-1);}}' < /tmp/disk`
}
