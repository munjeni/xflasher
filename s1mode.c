#include <linux/usbdevice_fs.h>
#include <linux/usb/ch9.h>
#include <asm/byteorder.h>

#include <string.h>
#include <errno.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#include <dirent.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <ctype.h>

#define TIMEOUT 5000

#define USBVID 0x0FCE
#define USBPID 0xADDE
#define EPIN 0x81
#define EPOUT 0x01

#define MAX_USBFS_BULK_SIZE 400000

static void display_buffer_hex(char *message, char *buffer, unsigned int size)
{
	unsigned int i, j, k;

	printf("%s[%d]:\n", message, (int)size);

	for (i=0; i<size; i+=16) {
		printf("\n  %08X  ", i);
		for(j=0,k=0; k<16; j++,k++) {
			if (i+j < size) {
				printf("%02X", buffer[i+j] & 0xff);
			} else {
				printf("  ");
			}
			printf(" ");
		}
		printf(" ");
		for(j=0,k=0; k<16; j++,k++) {
			if (i+j < size) {
				if ((buffer[i+j] < 32) || (buffer[i+j] > 126)) {
					printf(".");
				} else {
					printf("%c", buffer[i+j]);
				}
			}
		}
	}
	printf("\n\n" );
}

struct usb_handle
{
	char fname[64];
	int desc;
	unsigned char ep_in;
	unsigned char ep_out;
};

static inline int badname(const char *name)
{
	while (*name) {
		if (!isdigit(*name++))
			return 1;
	}
	return 0;
}

static int get_vidpid(int fd)
{
	struct usb_device_descriptor *dev;
	//struct usb_interface_descriptor *ifc;
	char desc[1024];
	int n;

	if ((n = read(fd, desc, sizeof(desc))) == 0)
		return 0;
	dev = (void *)desc;
	//fprintf(stderr, "found vid: %04x\n", dev->idVendor);
	//fprintf(stderr, "found pid: %04x\n", dev->idProduct);
	if (dev->idVendor != USBVID || dev->idProduct != USBPID)
		return 0;

	return 1;
}

static unsigned long transfer_bulk_async(struct usb_handle *h, int ep, char *bytes, unsigned long size, int timeout, int exact)
{
	unsigned long count = 0;
	struct usbdevfs_bulktransfer bulk;
	int n;

	if (ep == EPIN)
	{
		if (h->ep_in == 0)
			return 0;

		while (size > 0)
		{
			int xfer = (size > MAX_USBFS_BULK_SIZE) ? MAX_USBFS_BULK_SIZE : size;

			bulk.ep = h->ep_in;
			bulk.len = xfer;
			bulk.data = bytes;
			bulk.timeout = timeout;

			do
			{
				n = ioctl(h->desc, USBDEVFS_BULK, &bulk);
				if (n < 0) {
					printf("ERROR: n = %d, errno = %d (%s)\n",n, errno, strerror(errno));
					return 0;
				}
			}
			while(n < 0);

			count += n;
			size -= n;

			if (n < xfer)
				break;
		}
	}

	if (ep == EPOUT)
	{
		if (h->ep_out == 0)
			return 0;

		if (size == 0) {
			bulk.ep = h->ep_out;
			bulk.len = 0;
			bulk.data = bytes;
			bulk.timeout = 0;

			n = ioctl(h->desc, USBDEVFS_BULK, &bulk);
			if (n != 0) {
				fprintf(stderr,"ERROR: n = %d, errno = %d (%s)\n", n, errno, strerror(errno));
				return 0;
			}
			return 0;
		}

		while (size > 0) {
			int xfer;
			xfer = (size > MAX_USBFS_BULK_SIZE) ? MAX_USBFS_BULK_SIZE : size;
        
			bulk.ep = h->ep_out;
			bulk.len = xfer;
			bulk.data = bytes;
			bulk.timeout = timeout;

			n = ioctl(h->desc, USBDEVFS_BULK, &bulk);
			if (n != xfer) {
				printf("ERROR: n = %d, errno = %d (%s)\n", n, errno, strerror(errno));
				return 0;
			}

			count += xfer;
			size -= xfer;
		}
	}

	if (exact) {
		if (count != size) {
			printf("Error read/write!!! Need nBytes: 0x%lx but done: 0x%lx\n", size, count);
			return 0;
		}
	}

	if (ep == EPIN) {
		printf("Successfully read 0x%lx bytes from handle.\n", count);
		display_buffer_hex("Raw input ", bytes, count);
	}

	if (ep == EPOUT) {
		printf("Successfully write 0x%lx bytes to handle.\n", count);
		//display_buffer_hex("Raw output ", bytes, count);
	}
    
	return count;
}

struct usb_handle *get_fastboot(void)
{
	char busname[64], devname[64];
	DIR *busdir, *devdir;
	struct dirent *de;

	int fd;
	int found_usb = 0;
	int n;
	int ifc;

	struct usb_handle *usb = NULL;

	busdir = opendir("/dev/bus/usb");
	//fprintf(stderr, "busdir: %p\n", busdir);

	while ((de = readdir(busdir)) && (found_usb == 0)) {
		//fprintf(stderr, "dirent: %p\n", de);
		if (badname(de->d_name))
			continue;
		sprintf(busname, "%s/%s", "/dev/bus/usb", de->d_name);
		//fprintf(stderr, "busname: %s\n", busname);

		devdir = opendir(busname);

		while ((de = readdir(devdir)) && (found_usb == 0)) {
			if (badname(de->d_name))
				continue;
			sprintf(devname, "%s/%s", busname, de->d_name);
			//fprintf(stderr, "devname: %s\n", devname);

			if ((fd = open(devname, O_RDWR)) < 1) {
				fprintf(stderr, "cannot open %s for writing\n", devname);
				continue;	
			}

			if (get_vidpid(fd)) {
				fprintf(stderr, "found device with vid:%04x pid:%04x.\n", USBVID, USBPID);

				usb = calloc(1, sizeof(struct usb_handle));

				usb->ep_in = EPIN;
				usb->ep_out = EPOUT;
				usb->desc = fd;

				ifc = 0;
				if ((n = ioctl(fd, USBDEVFS_CLAIMINTERFACE, &ifc)) != 0) {
					fprintf(stderr,"ERROR: n = %d, errno = %d (%s)\n",
						 n, errno, strerror(errno));
					closedir(devdir);
					closedir(busdir);
					return NULL;
				}
				found_usb = 1;                
			}
		}
		closedir(devdir);
	}
	closedir(busdir);
	return usb;
}

int usb_close(struct usb_handle *h)
{
	int fd;
    
	fd = h->desc;
	h->desc = -1;
	if (fd >= 0) {
		close(fd);
		//printf("usb closed %d\n", fd);
	}

	return 0;
}

int main(void)
{
	struct usb_handle *usb;

	usb = get_fastboot();

	if (usb == NULL) {
		printf("\nNo usb device with vid:%04x pid:%04x !\n", USBVID, USBPID);
		exit(1);
	}

	char test1[4096];
	transfer_bulk_async(usb, EPIN, test1, sizeof(test1), TIMEOUT, 0);
	usb_close(usb);

	return 0;
}




