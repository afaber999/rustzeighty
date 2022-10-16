# Build a filesystem from the cpm22 distro files, retro.asm BIOS, and some
# sample downloads like adventure.

all: sda.img

force:
	make -C ../asm/cpm22/ cpm_all.bin


cpm_all.img: ../asm/cpm22/cpm_all.bin ./cpm22-b ./cpm-advent
	rm -f cpm_all.img
	mkfs.cpm -f z80-blkdev-2k-8m -b ../asm/cpm22/cpm_all.bin cpm_all.img
	cpmcp -f z80-blkdev-2k-8m cpm_all.img ./cpm22-b/* 0:
	cpmcp -f z80-blkdev-2k-8m cpm_all.img cpm-advent/adv-B03/* 0:

sda.img: cpm_all.img
	#dd bs=512 count=300000 if=/dev/zero of=sda.img
	#dd bs=512 count=19000 if=/dev/zero of=sda.img
	dd if=./cpm_all.img of=./sda.img bs=512 seek=2048
	cp ./sda.img ./sda_org.img

lsimg:
	cpmls -f z80-blkdev-2k-8m cpm_all.img 

cpm-advent.zip:
	wget --no-check-certificate https://ifarchive.org/if-archive/games/cpm/cpm-advent.zip 

cpm-advent: cpm-advent.zip
	unzip -d cpm-advent cpm-advent.zip

cpm22-b: cpm22-b.zip
	unzip -d cpm22-b cpm22-b.zip

cpm22-b.zip:
	wget http://www.cpm.z80.de/download/cpm22-b.zip

clean:
	rm -f cpm_all.img
	rm -f sda.img
	rm -rf cpm22_filesystem
	rm -rf cpm-advent

