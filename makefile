ARMCC=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-gcc
ARMSTRIP=/home/savan/Desktop/gcc-linaro-5.3.1-2016.05-x86_64_arm-linux-gnueabi/bin/arm-linux-gnueabi-strip

ARMCC64=/home/savan/Desktop/qemu/busybox-1.23.2/busybox-1.23.2/busybox-w32/buildroot-2017.02.2/output/host/usr/bin/aarch64-buildroot-linux-uclibc-gcc
ARMSTRIP64=/home/savan/Desktop/qemu/busybox-1.23.2/busybox-1.23.2/busybox-w32/buildroot-2017.02.2/output/host/usr/bin/aarch64-buildroot-linux-uclibc-strip

CCWIN=i686-w64-mingw32-gcc
CCWINSTRIP=i686-w64-mingw32-strip
WINDRES=i686-w64-mingw32-windres

CC=gcc
STRIP=strip

CFLAGS = -Wall -O3 -pedantic -static -I. -I ./expat/include -L ./expat/lib

default:xflasher.arm32 xflasher.arm64 xflasher.x64 xflasher.i386 xflasher.exe

xflasher.arm32:

	${ARMCC} ${CFLAGS} -fPIC -c xflasher_v23.c -o xflasher.o
	${ARMCC} ${CFLAGS} -fPIC -c sha256.c -o sha256.o
	${ARMCC} ${CFLAGS} -fPIC sha256.o xflasher.o -o xflasher.arm32 -L. -lexpat.arm32
	${ARMSTRIP} xflasher.arm32

xflasher.arm64:

	${ARMCC64} ${CFLAGS} -fPIC -c xflasher_v23.c -o xflasher.o
	${ARMCC64} ${CFLAGS} -fPIC -c sha256.c -o sha256.o
	${ARMCC64} ${CFLAGS} -fPIC sha256.o xflasher.o -o xflasher.arm64 -L. -lexpat.arm64
	${ARMSTRIP64} xflasher.arm64

xflasher.i386:

	${CC} ${CFLAGS} -m32 -c xflasher_v23.c -o xflasher.o
	${CC} ${CFLAGS} -m32 -c sha256.c -o sha256.o
	${CC} ${CFLAGS} -m32 sha256.o xflasher.o -o xflasher.i386 -L. -lexpat.i386
	${STRIP} xflasher.i386

xflasher.x64:

	${CC} ${CFLAGS} -c xflasher_v23.c -o xflasher.o
	${CC} ${CFLAGS} -c sha256.c -o sha256.o
	${CC} ${CFLAGS} sha256.o xflasher.o -o xflasher.x64 -L. -lexpat.x64
	${STRIP} xflasher.x64

xflasher.exe:

	${CCWIN} ${CFLAGS} -c xflasher_v23.c -o xflasher.o
	${CCWIN} ${CFLAGS} -c sha256.c -o sha256.o
	${WINDRES} xflasher.rc -O coff -o xflasher.res
	${CCWIN} ${CFLAGS} sha256.o xflasher.o xflasher.res -o xflasher.exe -L. -lexpat.win -lsetupapi
	${CCWINSTRIP} xflasher.exe

clean:
	rm -f *.o *.res

distclean:
	rm -f *.o *.res xflasher.exe xflasher.arm32 xflasher.arm64 xflasher.i386 xflasher.x64



