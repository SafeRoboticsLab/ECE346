#!/bin/sh
set -ex
GPT_SIZE=40
UDA_NEW_SIZE=32
move_part() {
  name=$(sgdisk -i $1 /dev/nvme0n1 | grep "Partition name" | cut -d"'" -f2)
  typecode=$(sgdisk -i $1 /dev/nvme0n1 | grep "Partition GUID code:" | cut -d' ' -f4)
  guid=$(sgdisk -i $1 /dev/nvme0n1 | grep "Partition unique GUID:" | cut -d' ' -f4)
  sgdisk -d $1 -n $1:$2:$3 -c $1:"$name" -t $1:"$typecode" -u $1:"$guid" /dev/nvme0n1
  partprobe /dev/nvme0n1
}
read DISK_SIZE </sys/block/nvme0n1/size
START=$((DISK_SIZE-GPT_SIZE-UDA_NEW_SIZE))
move_part 11 $START $((START+UDA_NEW_SIZE-1))
for i in $(seq 10 -1 2); do
  dd if=/dev/nvme0n1p$i of=part$i.img
  read size </sys/block/nvme0n1/nvme0n1p$i/size
  START=$((START-size))
  move_part $i $START $((START+size-1))
  dd of=/dev/nvme0n1p$i if=part$i.img
  rm -f part$i.img
done
move_part 1 $GPT_SIZE 0
sgdisk --move-second-header /dev/nvme0n1
resize2fs /dev/nvme0n1p1
