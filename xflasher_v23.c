/*
 * Copyright (C) 2014 Munjeni
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#if (!defined(_WIN32)) && (!defined(WIN32)) && (!defined(__APPLE__))
	#ifndef __USE_FILE_OFFSET64
		#define __USE_FILE_OFFSET64 1
	#endif
	#ifndef __USE_LARGEFILE64
		#define __USE_LARGEFILE64 1
	#endif
	#ifndef _LARGEFILE64_SOURCE
		#define _LARGEFILE64_SOURCE 1
	#endif
	#ifndef _FILE_OFFSET_BITS
		#define _FILE_OFFSET_BITS 64
	#endif
	#ifndef _FILE_OFFSET_BIT
		#define _FILE_OFFSET_BIT 64
	#endif
#endif

#ifdef _WIN32
	#define __USE_MINGW_ANSI_STDIO 1

#include <windows.h>
#include <setupapi.h>
#include <initguid.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#ifdef HAS_STDINT_H
	#include <stdint.h>
#endif
#ifdef unix
	#include <unistd.h>
	#include <sys/types.h>
#else
	#include <direct.h>
	#include <io.h>
#endif

#if defined(USE_FILE32API)
	#define fopen64 fopen
	#define ftello64 ftell
	#define fseeko64 fseek
#else
	#ifdef __FreeBSD__
		#define fopen64 fopen
		#define ftello64 ftello
		#define fseeko64 fseeko
	#endif
	/*#ifdef __ANDROID__
		#define fopen64 fopen
		#define ftello64 ftello
		#define fseeko64 fseeko
	#endif*/
	#ifdef _MSC_VER
		#define fopen64 fopen
		#if (_MSC_VER >= 1400) && (!(defined(NO_MSCVER_FILE64_FUNC)))
			#define ftello64 _ftelli64
			#define fseeko64 _fseeki64
		#else  /* old msc */
			#define ftello64 ftell
			#define fseeko64 fseek
		#endif
	#endif
#endif

#include <ctype.h>
#include <sys/stat.h>
#include <limits.h>
#include <time.h>
#include <dirent.h>

#ifndef _WIN32
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
#endif

#include "sha256.h"
#include "expat.h"

#ifdef XML_LARGE_SIZE
#if defined(XML_USE_MSC_EXTENSIONS) && _MSC_VER < 1400
#define XML_FMT_INT_MOD "I64"
#else
#define XML_FMT_INT_MOD "ll"
#endif
#else
#define XML_FMT_INT_MOD "l"
#endif

#ifdef _WIN32
#define sleep Sleep
#define ONESEC 1000
#else
#define ONESEC 1
#endif

#define ENABLE_DEBUG 1

#if ENABLE_DEBUG
#define LOG printf
#else
#define LOG(...)
#endif

#define USB_TIMEOUT 60000
#define USB_VERIFY_TIMEOUT 600000

#define CMD_LOADER_INFO		"\x00\x00\x00\x01"            /* loader info (hook phone in flashmode) */
#define CMD_FLASHMODE_OFF		"\x00\x00\x00\x04"            /* Kick device off flashmode */
#define CMD_WRITE_SIN_HEADER	"\x00\x00\x00\x05"            /* write SIN header */
#define CMD_WRITE_SIN		"\x00\x00\x00\x06"            /* write SIN */
#define CMD_GET_LAST_ERROR		"\x00\x00\x00\x07"            /* Get last error */
#define CMD_OPEN_TA			"\x00\x00\x00\x09"            /* open TA (takes the partition number as parameter) */
#define CMD_CLOSE_TA			"\x00\x00\x00\x0A"            /* close TA */
#define CMD_READ_TA			"\x00\x00\x00\x0C"            /* read TA */
#define CMD_WRITE_TA			"\x00\x00\x00\x0D"            /* write TA */
#define CMD_DISABLE_VERIFICATION	"\x00\x00\x00\x19"            /* Disable Final Verification check ? */

#define FLAG1				"\x00\x00\x00\x01"
#define FLAG3				"\x00\x00\x00\x03"            /* command have no more data than max packet size */
#define FLAG7				"\x00\x00\x00\x07"            /* command have more data than max packet size */

static char device_otp_lock[33];
static char device_otpdata[22];
static char device_idcode[20];
static char device_plfroot[78];
static char bootdelivery_attr[33+22+20+78];
static int bootdelivery_found = 0;
static char device_hwconfig[64];

static char tmp[4096];
static char *tmp2;
static char command[13];
static char temp[9];
static char rest[4];
static char part[1];
static char *sum;
static char *crc32;
#if 0
static char huk_cmd[23] = "\x00\x00\x00\x19\x00\x00\x00\x03\x00\x00\x00\x06\x23\x00\x01\x00\x00\x00\x01\x4A\x67\x19\x26";
#endif
static int unit_buff_sz = 0;

static char *basenamee(char *in) {
	char *ssc;
	int p = 0;
	ssc = strstr(in, "/");
	if(ssc == NULL) {
	  ssc = strstr(in, "\\");
	  if(ssc == NULL) {
	  	return in;
		}
	}
	do {
	    p = strlen(ssc) + 1;
	    in = &in[strlen(in)-p+2];
	    ssc = strstr(in, "/");
			if (ssc == NULL)
	      ssc = strstr(in, "\\");
	} while(ssc);

	return in;
}

#ifndef _WIN32
static char *TEXT(char *what) {
	return what;
}

void DisplayError(char *title)
{
	printf("%s\n%s\n", title, strerror(errno));
}
#else
#define StringCchPrintf(str, n, format, ...) snprintf((char*)str, n, (char const*)format, __VA_ARGS__)
void DisplayError(LPTSTR lpszFunction)
{
	LPVOID lpMsgBuf;
	LPVOID lpDisplayBuf;
	DWORD dw = GetLastError();

	FormatMessage(
		FORMAT_MESSAGE_ALLOCATE_BUFFER |
		FORMAT_MESSAGE_FROM_SYSTEM |
		FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL,
		dw,
		MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
		(LPTSTR) &lpMsgBuf,
		0,
		NULL );

	lpDisplayBuf =
			(LPVOID)LocalAlloc( LMEM_ZEROINIT,
			          ( lstrlen((LPCTSTR)lpMsgBuf)
			            + lstrlen((LPCTSTR)lpszFunction)
			            + 40) /* account for format string */
			          * sizeof(TCHAR) );

	if (FAILED( StringCchPrintf((LPTSTR)lpDisplayBuf,
			LocalSize(lpDisplayBuf) / sizeof(TCHAR),
			TEXT("%s failed with error code %lu as follows:\n%s"),
			lpszFunction,
			dw,
			(char *)lpMsgBuf)))
	{
		printf("FATAL ERROR: Unable to output error code.\n");
	}

	printf("ERROR: %s\n", (LPCTSTR)lpDisplayBuf);

	LocalFree(lpMsgBuf);
	LocalFree(lpDisplayBuf);
}
#endif

#ifdef _WIN32
static char *uint16_to_vidpidstring(unsigned short VID, unsigned short PID)
{
	static char temp[18];

	snprintf(temp, sizeof(temp), "vid_%x%x%x%x&pid_%x%x%x%x",
		 (VID>>12)&0xf, (VID>>8)&0xf, (VID>>4)&0xf, VID&0xf,
		 (PID>>12)&0xf, (PID>>8)&0xf, (PID>>4)&0xf, PID&0xf);

	return temp;
}
#endif

static void to_ascii(char *dest, const char *text) {
	unsigned long int ch;
	for(; sscanf((const char *)text, "%02lx", &ch)==1; text+=2)
		*dest++ = ch;
	*dest = 0;
}

static void to_uppercase(char *ptr) {
	for ( ; *ptr; ++ptr) *ptr = toupper(*ptr);
}

static void display_buffer_hex(char *message, char *buffer, unsigned long size) {
	unsigned long i, j, k;

	LOG("%s[0x%lX]:\n", message, size);

	for (i=0; i<size; i+=16) {
		LOG("\n  %08lX  ", i);
		for(j=0,k=0; k<16; j++,k++) {
			if (i+j < size) {
				LOG("%02X", buffer[i+j] & 0xff);
			} else {
				LOG("  ");
			}
			LOG(" ");
		}
		LOG(" ");
		for(j=0,k=0; k<16; j++,k++) {
			if (i+j < size) {
				if ((buffer[i+j] < 32) || (buffer[i+j] > 126)) {
					LOG(".");
				} else {
					LOG("%c", buffer[i+j]);
				}
			}
		}
	}
	LOG("\n\n" );
}

#define EP_IN 0
#define EP_OUT 1

#ifdef _WIN32
static GUID GUID_DEVINTERFACE_USB_DEVICE = {0xA5DCBF10L, 0x6530, 0x11D2, {0x90, 0x1F, 0x00, 0xC0, 0x4F, 0xB9, 0x51, 0xED}};

HDEVINFO	hDevInfo;
#else
#define SetupDiDestroyDeviceInfoList(...)

/* The max bulk size for linux is 16384 which is defined
 * in drivers/usb/core/devio.c.
 */
#define MAX_USBFS_BULK_SIZE 4096
/*(16 * 1024)*/

struct usb_handle
{
	char fname[64];
	int desc;
	unsigned char ep_in;
	unsigned char ep_out;
};

typedef struct usb_handle *HANDLE;

static inline int badname(const char *name)
{
	while (*name) {
		if (!isdigit(*name++))
			return 1;
	}
	return 0;
}

static int get_vidpid(int fd, unsigned short VID, unsigned short PID)
{
	struct usb_device_descriptor *dev;
	char desc[1024];
	int n;

	if ((n = read(fd, desc, sizeof(desc))) == 0)
		return 0;
	dev = (void *)desc;
	/*printf("found vid: %04x\n", dev->idVendor);
	printf("found pid: %04x\n", dev->idProduct);*/
	if (dev->idVendor != VID || dev->idProduct != PID)
		return 0;

	return 1;
}

struct usb_handle *get_flashmode(unsigned short VID, unsigned short PID)
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
	if (busdir == NULL) {
		printf("Error, no /dev/bus/usb ! Please connect device first in flash mode!\n");
		return usb;
	}

	/*printf("busdir: %p\n", busdir);*/

	while ((de = readdir(busdir)) && (found_usb == 0)) {
		/*printf("dirent: %p\n", de);*/
		if (badname(de->d_name))
			continue;
		sprintf(busname, "%s/%s", "/dev/bus/usb", de->d_name);
		/*printf("busname: %s\n", busname);*/

		devdir = opendir(busname);

		while ((de = readdir(devdir)) && (found_usb == 0)) {
			if (badname(de->d_name))
				continue;
			sprintf(devname, "%s/%s", busname, de->d_name);
			/*printf("devname: %s\n", devname);*/

			if ((fd = open(devname, O_RDWR)) < 1) {
				printf("cannot open %s for writing\n", devname);
				continue;	
			}

			if (get_vidpid(fd, VID, PID)) {
				printf("found device with vid:0x%04x pid:0x%04x.\n", VID, PID);

				usb = calloc(1, sizeof(struct usb_handle));

				usb->ep_in = 0x81;
				usb->ep_out = 0x01;
				usb->desc = fd;

				ifc = 0;
				if ((n = ioctl(fd, USBDEVFS_CLAIMINTERFACE, &ifc)) != 0) {
					printf("ERROR: n = %d, errno = %d (%s)\n",
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
		/*printf("usb closed %d\n", fd);*/
	}

	return 0;
}

#define CloseHandle usb_close
#endif

#ifdef _WIN32
static char *open_dev(unsigned short VID, unsigned short PID)
{
	SP_DEVICE_INTERFACE_DATA         DevIntfData;
	PSP_DEVICE_INTERFACE_DETAIL_DATA DevIntfDetailData;
	SP_DEVINFO_DATA                  DevData;

	unsigned long dwSize, dwMemberIdx;
	static char devicePath[MAX_PATH];
	char szDescription[MAX_PATH];

	int ret = 1;
	char *vidpid = uint16_to_vidpidstring(VID, PID);

	memset(devicePath, 0, sizeof(devicePath));

	hDevInfo = SetupDiGetClassDevs(&GUID_DEVINTERFACE_USB_DEVICE, NULL, 0, DIGCF_DEVICEINTERFACE | DIGCF_PRESENT);

	if (hDevInfo != INVALID_HANDLE_VALUE)
	{
		DevIntfData.cbSize = sizeof(SP_DEVICE_INTERFACE_DATA);
		dwMemberIdx = 0;

		SetupDiEnumDeviceInterfaces(hDevInfo, NULL, &GUID_DEVINTERFACE_USB_DEVICE, dwMemberIdx, &DevIntfData);

		while(GetLastError() != ERROR_NO_MORE_ITEMS)
		{
			DevData.cbSize = sizeof(DevData);

			SetupDiGetDeviceInterfaceDetail(hDevInfo, &DevIntfData, NULL, 0, &dwSize, NULL);

			DevIntfDetailData = HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, dwSize);
			DevIntfDetailData->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA);

			if (SetupDiGetDeviceInterfaceDetail(hDevInfo, &DevIntfData, DevIntfDetailData, dwSize, &dwSize, &DevData))
			{
				if (strstr(DevIntfDetailData->DevicePath, vidpid) != NULL)
				{
					strncpy(devicePath, DevIntfDetailData->DevicePath, strlen(DevIntfDetailData->DevicePath));
					printf("Device path: %s\n", devicePath);
					memset(szDescription, 0, MAX_PATH);
					SetupDiGetClassDescription(&DevData.ClassGuid, szDescription, MAX_PATH, &dwSize);
					printf("Class Description: %s\n", szDescription);

					memset(szDescription, 0, MAX_PATH);
					SetupDiGetDeviceInstanceId(hDevInfo, &DevData, szDescription, MAX_PATH, 0);
					printf("Device Instance Id: %s\n\n", szDescription);

					ret = 0;
				}
			}

			HeapFree(GetProcessHeap(), 0, DevIntfDetailData);

			/* Continue looping */
			SetupDiEnumDeviceInterfaces(hDevInfo, NULL, &GUID_DEVINTERFACE_USB_DEVICE, ++dwMemberIdx, &DevIntfData);

			if (ret == 0)
				break;
		}

		if (ret)
			SetupDiDestroyDeviceInfoList(hDevInfo);
	}

	return devicePath;
}

static unsigned long transfer_bulk_async(HANDLE dev, int ep, char *bytes, unsigned long size, int timeout, int exact)
{
	static unsigned long nBytesRead = 0;
	BOOL bResult;

	OVERLAPPED gOverLapped_in = {
		.Internal     = 0,
		.InternalHigh = 0,
		.Offset       = 0,
		.OffsetHigh   = 0
	};

	OVERLAPPED gOverLapped_out = {
		.Internal     = 0,
		.InternalHigh = 0,
		.Offset       = 0,
		.OffsetHigh   = 0
	};

	if (ep == EP_IN)
	{
		nBytesRead = 0;
		gOverLapped_in.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		if (NULL == gOverLapped_in.hEvent) {
			DisplayError(TEXT("Error creating overlaped_in hEvent!"));
			return 0;
		}
		else
		{
			bResult = ReadFile(dev, bytes, size, NULL, &gOverLapped_in);

			if(!bResult)
			{
				switch (GetLastError())
				{
					case ERROR_HANDLE_EOF:
					{
						/* we have reached the end of the file during the call to ReadFile */
						DisplayError(TEXT("HANDLE_EOF:"));
						break;
					}
					case ERROR_IO_PENDING:
					{
						/* asynchronous i/o is still in progress */
						switch(WaitForSingleObject(gOverLapped_in.hEvent, timeout))
						{
							case WAIT_OBJECT_0:
								/* check on the results of the asynchronous read and update the nBytesRead... */
								bResult = GetOverlappedResult(dev, &gOverLapped_in, &nBytesRead, TRUE);
								if (!bResult)
									DisplayError(TEXT("GetOverlapped_in_Result:"));
								break;

							case WAIT_TIMEOUT:
								DisplayError(TEXT("TIMEOUT:"));
								CancelIo(dev);
								break;

							default:
								DisplayError(TEXT("ERROR_IO_PENDING OTHER:"));
								CancelIo(dev);
								break;
						}
					}
					default:
					{
						CancelIo(dev);
						break;
					}
				}
			}

			ResetEvent(gOverLapped_in.hEvent);
			CloseHandle(gOverLapped_in.hEvent);
		}
	}

	if (ep == EP_OUT)
	{
		nBytesRead = 0;
		gOverLapped_out.hEvent = CreateEvent(NULL, TRUE, FALSE, NULL);
		if (NULL == gOverLapped_out.hEvent) {
			DisplayError(TEXT("Error creating overlaped_in hEvent!"));
			return 0;
		}
		else
		{
			bResult = WriteFile(dev, bytes, size, NULL, &gOverLapped_out);

			if(!bResult)
			{
				switch (GetLastError())
				{
					case ERROR_HANDLE_EOF:
					{
						/* we have reached the end of the file during the call to ReadFile */
						DisplayError(TEXT("HANDLE_EOF:"));
						break;
					}
					case ERROR_IO_PENDING:
					{
						/* asynchronous i/o is still in progress */
						switch(WaitForSingleObject(gOverLapped_out.hEvent, timeout))
						{
							case WAIT_OBJECT_0:
								/* check on the results of the asynchronous read and update the nBytesRead... */
								bResult = GetOverlappedResult(dev, &gOverLapped_out, &nBytesRead, TRUE);
								if (!bResult)
									DisplayError(TEXT("GetOverLapped_out_Result:"));
								break;

							case WAIT_TIMEOUT:
								DisplayError(TEXT("TIMEOUT:"));
								CancelIo(dev);
								break;

							default:
								DisplayError(TEXT("ERROR_IO_PENDING OTHER:"));
								CancelIo(dev);
								break;
						}
					}
					default:
					{
						CancelIo(dev);
						break;
					}
				}
			}

			ResetEvent(gOverLapped_out.hEvent);
			CloseHandle(gOverLapped_out.hEvent);
		}
	}

	if (exact) {
		if (nBytesRead != size) {
			printf(" - Error %s! Need nBytes: 0x%lx but done: 0x%lx\n", (ep == EP_IN) ? "read" : "write", size, nBytesRead);
			display_buffer_hex("nBytes", bytes, nBytesRead);
			return 0;
		}
	}

	if (ep == EP_IN && nBytesRead) {
		printf(" - Successfully read 0x%lx bytes from handle.\n", nBytesRead);
		display_buffer_hex("Raw input ", bytes, nBytesRead);
	}

	if (ep == EP_OUT && nBytesRead) {
		printf(" - Successfully write 0x%lx bytes to handle.\n", nBytesRead);
		/*display_buffer_hex("Raw output ", bytes, nBytesRead);*/
	}

	return nBytesRead;
}
#else
static unsigned long transfer_bulk_async(struct usb_handle *h, int ep, const void *_bytes, unsigned long size, int timeout, int exact)
{
	char *bytes = (char *)_bytes;
	unsigned long count = 0;
	unsigned long size_tot = size;
	struct usbdevfs_bulktransfer bulk;
	int n;

	if (ep == EP_IN)
	{
		if (h->ep_in == 0) {
			printf(" - ep_in is not 0x81!!!\n");
			return 0;
		}

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
					printf(" - (ep_in) ERROR: n = %d, errno = %d (%s)\n",n, errno, strerror(errno));
					return 0;
				}
			}
			while(n < 0);

			count += n;
			size -= n;
			bytes += n;

			if (n < xfer)
				break;
		}
	}

	if (ep == EP_OUT)
	{
		if (h->ep_out == 0) {
			printf(" - ep_out is not 0x01!!!\n");
			return 0;
		}

		if (size == 0) {
			bulk.ep = h->ep_out;
			bulk.len = 0;
			bulk.data = bytes;
			bulk.timeout = 0;

			n = ioctl(h->desc, USBDEVFS_BULK, &bulk);
			if (n != 0) {
				printf(" - (ep_out size=0)ERROR: n = %d, errno = %d (%s)\n", n, errno, strerror(errno));
				return 0;
			}
			return 0;
		}

		while (size > 0)
		{
			int xfer = (size > MAX_USBFS_BULK_SIZE) ? MAX_USBFS_BULK_SIZE : size;

			bulk.ep = h->ep_out;
			bulk.len = xfer;
			bulk.data = bytes;
			bulk.timeout = timeout;

			n = ioctl(h->desc, USBDEVFS_BULK, &bulk);
			if (n != xfer) {
				printf(" - (ep_out size=%d)ERROR: n = %d, errno = %d (%s)\n", xfer, n, errno, strerror(errno));
				return 0;
			}

			count += xfer;
			size -= xfer;
			bytes += xfer;
		}
	}

	if (exact) {
		if (count != size_tot) {
			printf(" - Error %s! Need nBytes: 0x%lx but done: 0x%lx\n", (ep == EP_IN) ? "read" : "write", size_tot, count);
			display_buffer_hex("nBytes", bytes, count);
			return 0;
		}
	}

	if (ep == EP_IN) {
		printf(" - Successfully read 0x%lx bytes from handle.\n", count);
		display_buffer_hex("Raw input ", bytes, count);
	}

	if (ep == EP_OUT) {
		printf(" - Successfully write 0x%lx bytes to handle.\n", count);
		/*display_buffer_hex("Raw output ", bytes, count);*/
	}

	return count;
}
#endif

static unsigned int crctable[256] = {
	0x00000000, 0x77073096, 0xee0e612c, 0x990951ba, 0x076dc419, 0x706af48f, 0xe963a535, 0x9e6495a3,
	0x0edb8832, 0x79dcb8a4, 0xe0d5e91e, 0x97d2d988, 0x09b64c2b, 0x7eb17cbd, 0xe7b82d07, 0x90bf1d91,
	0x1db71064, 0x6ab020f2, 0xf3b97148, 0x84be41de, 0x1adad47d, 0x6ddde4eb, 0xf4d4b551, 0x83d385c7,
	0x136c9856, 0x646ba8c0, 0xfd62f97a, 0x8a65c9ec, 0x14015c4f, 0x63066cd9, 0xfa0f3d63, 0x8d080df5,
	0x3b6e20c8, 0x4c69105e, 0xd56041e4, 0xa2677172, 0x3c03e4d1, 0x4b04d447, 0xd20d85fd, 0xa50ab56b,
	0x35b5a8fa, 0x42b2986c, 0xdbbbc9d6, 0xacbcf940, 0x32d86ce3, 0x45df5c75, 0xdcd60dcf, 0xabd13d59,
	0x26d930ac, 0x51de003a, 0xc8d75180, 0xbfd06116, 0x21b4f4b5, 0x56b3c423, 0xcfba9599, 0xb8bda50f,
	0x2802b89e, 0x5f058808, 0xc60cd9b2, 0xb10be924, 0x2f6f7c87, 0x58684c11, 0xc1611dab, 0xb6662d3d,
	0x76dc4190, 0x01db7106, 0x98d220bc, 0xefd5102a, 0x71b18589, 0x06b6b51f, 0x9fbfe4a5, 0xe8b8d433,
	0x7807c9a2, 0x0f00f934, 0x9609a88e, 0xe10e9818, 0x7f6a0dbb, 0x086d3d2d, 0x91646c97, 0xe6635c01,
	0x6b6b51f4, 0x1c6c6162, 0x856530d8, 0xf262004e, 0x6c0695ed, 0x1b01a57b, 0x8208f4c1, 0xf50fc457,
	0x65b0d9c6, 0x12b7e950, 0x8bbeb8ea, 0xfcb9887c, 0x62dd1ddf, 0x15da2d49, 0x8cd37cf3, 0xfbd44c65,
	0x4db26158, 0x3ab551ce, 0xa3bc0074, 0xd4bb30e2, 0x4adfa541, 0x3dd895d7, 0xa4d1c46d, 0xd3d6f4fb,
	0x4369e96a, 0x346ed9fc, 0xad678846, 0xda60b8d0, 0x44042d73, 0x33031de5, 0xaa0a4c5f, 0xdd0d7cc9,
	0x5005713c, 0x270241aa, 0xbe0b1010, 0xc90c2086, 0x5768b525, 0x206f85b3, 0xb966d409, 0xce61e49f,
	0x5edef90e, 0x29d9c998, 0xb0d09822, 0xc7d7a8b4, 0x59b33d17, 0x2eb40d81, 0xb7bd5c3b, 0xc0ba6cad,
	0xedb88320, 0x9abfb3b6, 0x03b6e20c, 0x74b1d29a, 0xead54739, 0x9dd277af, 0x04db2615, 0x73dc1683,
	0xe3630b12, 0x94643b84, 0x0d6d6a3e, 0x7a6a5aa8, 0xe40ecf0b, 0x9309ff9d, 0x0a00ae27, 0x7d079eb1,
	0xf00f9344, 0x8708a3d2, 0x1e01f268, 0x6906c2fe, 0xf762575d, 0x806567cb, 0x196c3671, 0x6e6b06e7,
	0xfed41b76, 0x89d32be0, 0x10da7a5a, 0x67dd4acc, 0xf9b9df6f, 0x8ebeeff9, 0x17b7be43, 0x60b08ed5,
	0xd6d6a3e8, 0xa1d1937e, 0x38d8c2c4, 0x4fdff252, 0xd1bb67f1, 0xa6bc5767, 0x3fb506dd, 0x48b2364b,
	0xd80d2bda, 0xaf0a1b4c, 0x36034af6, 0x41047a60, 0xdf60efc3, 0xa867df55, 0x316e8eef, 0x4669be79,
	0xcb61b38c, 0xbc66831a, 0x256fd2a0, 0x5268e236, 0xcc0c7795, 0xbb0b4703, 0x220216b9, 0x5505262f,
	0xc5ba3bbe, 0xb2bd0b28, 0x2bb45a92, 0x5cb36a04, 0xc2d7ffa7, 0xb5d0cf31, 0x2cd99e8b, 0x5bdeae1d,
	0x9b64c2b0, 0xec63f226, 0x756aa39c, 0x026d930a, 0x9c0906a9, 0xeb0e363f, 0x72076785, 0x05005713,
	0x95bf4a82, 0xe2b87a14, 0x7bb12bae, 0x0cb61b38, 0x92d28e9b, 0xe5d5be0d, 0x7cdcefb7, 0x0bdbdf21,
	0x86d3d2d4, 0xf1d4e242, 0x68ddb3f8, 0x1fda836e, 0x81be16cd, 0xf6b9265b, 0x6fb077e1, 0x18b74777,
	0x88085ae6, 0xff0f6a70, 0x66063bca, 0x11010b5c, 0x8f659eff, 0xf862ae69, 0x616bffd3, 0x166ccf45,
	0xa00ae278, 0xd70dd2ee, 0x4e048354, 0x3903b3c2, 0xa7672661, 0xd06016f7, 0x4969474d, 0x3e6e77db,
	0xaed16a4a, 0xd9d65adc, 0x40df0b66, 0x37d83bf0, 0xa9bcae53, 0xdebb9ec5, 0x47b2cf7f, 0x30b5ffe9,
	0xbdbdf21c, 0xcabac28a, 0x53b39330, 0x24b4a3a6, 0xbad03605, 0xcdd70693, 0x54de5729, 0x23d967bf,
	0xb3667a2e, 0xc4614ab8, 0x5d681b02, 0x2a6f2b94, 0xb40bbe37, 0xc30c8ea1, 0x5a05df1b, 0x2d02ef8d
};

static char *getCRC32(char *in, unsigned int size) {
	unsigned int i, L, L1, permutation;
	static char out[4];

	permutation = 0;

	for(i=0; i < size; ++i) {
		L = (unsigned int)in[i];
		L1 = (permutation ^ L);
		permutation = ((permutation >> 8) ^ crctable[L1 & 0xff]);
	}

	out[0] = (permutation >> 24) & 0xff;
	out[1] = (permutation >> 16) & 0xff;
	out[2] = (permutation >> 8) & 0xff;
	out[3] = (permutation >> 0) & 0xff;

	return out;
}

static char *getSUM(char *in, unsigned int size) {
	unsigned int i;
	int m = 0;
	static char out[1];

	if (size < 1 || size > 12)
		return NULL;

	for(i=0; i < size; ++i) {
		m ^= (int)in[i];
	}
	m += 7;

	out[0] = m & 0xff;

	return out;
}

static int getSHA256(char *key, char *sum) {
	int i;
	char output[65];
	sha256_context ctx;
	unsigned char sha256sum[32];

	printf("Unlock key SHA-256 validation test: ");

	sha256_starts(&ctx);
	sha256_update(&ctx, (uint8 *)key, strlen(key));
	sha256_finish(&ctx, (uint8 *)sha256sum);

	for(i=0; i<32; i++)
		snprintf(output+i*2, sizeof(output), "%02X", sha256sum[i]&0xff);

	if(memcmp(output, sum, 64)) {
		printf("failed!\n");
		return 0;
	}

	printf("passed.\n\n");

	return 1;
}

static unsigned long get_reaply(HANDLE dev, int ep, char *bytes, unsigned long size, int timeout) {
	return  transfer_bulk_async(dev, ep, bytes, size, timeout, 0);
}

static char *verify(HANDLE dev, int ep, int timeout) {
	unsigned long ret, i;
	char *buff = NULL;

	ret = transfer_bulk_async(dev, ep, tmp, sizeof(tmp), timeout, 0);

	if (ret < 1)
	{
		goto out_error;
	}
	else
	{
		if ((buff = (char *)malloc((ret*2)+1)) == NULL) {
			printf("Error allocating buff!\n");
			return NULL;
		}

		for (i=0; i<ret; ++i) {
			snprintf(buff+i*2, (ret*2)+1, "%02X", tmp[i]&0xff);
		}
		goto out_success;
	}

	memset(tmp, 0, sizeof(tmp));

out_error:
	return NULL;

out_success:
	return buff;
}

unsigned long long file_size(char *filename) {
	unsigned long long size;
	FILE *fp = fopen64(filename, "rb");

	if (fp == NULL) {
		return 0;
	}

	fseeko64(fp, 0, SEEK_END);
	size = ftello64(fp);
	fseeko64(fp, 0, SEEK_SET);
	fclose(fp);

	return size;
}

unsigned long swap_uint32(unsigned long val) {
	val = ((val << 8) & 0xFF00FF00 ) | ((val >> 8) & 0xFF00FF);
	return ((val << 16) | (val >> 16)) & 0xffffffff;
}

unsigned long long swap_uint64(unsigned long long val) {
	val = ((val << 8) & 0xFF00FF00FF00FF00ULL) | ((val >> 8) & 0x00FF00FF00FF00FFULL);
	val = ((val << 16) & 0xFFFF0000FFFF0000ULL) | ((val >> 16) & 0x0000FFFF0000FFFFULL);
	return ((val << 32) | (val >> 32)) & 0xffffffffffffffffULL;
}

void fread_unus_res(void *ptr, size_t size, size_t nmemb, FILE *stream) {
	size_t in;
	in = fread(ptr, size, nmemb, stream);
	if (in) {
		/* satisfy warn unused result */
	}
}

#define MAX_UNIT_LINE_LEN 0x20000

static ssize_t g_getline(char **lineptr, size_t *n, FILE *stream) {
	char *cur_pos, *new_lineptr;
	int c;
	size_t new_lineptr_len;

	if (lineptr == NULL || n == NULL || stream == NULL) {
		errno = EINVAL;
		printf("Error: EINVAL!\n");
		return -1;
	}

	if (*lineptr == NULL) {
		*n = MAX_UNIT_LINE_LEN;
		if ((*lineptr = (char *)malloc(*n)) == NULL) {
			errno = ENOMEM;
			printf("Error: MAX_UNIT_LINE_LEN reached!\n");
			return -1;
		}
	}

	cur_pos = *lineptr;
	for (;;) {
		c = getc(stream);

		if (ferror(stream) || (c == EOF && cur_pos == *lineptr))
			return -1;

		if (c == EOF)
			break;

		if ((*lineptr + *n - cur_pos) < 2) {
			if (SSIZE_MAX / 2 < *n) {
#ifdef EOVERFLOW
				errno = EOVERFLOW;
#else
				errno = ERANGE; /* no EOVERFLOW defined */
#endif
			printf("Error: EOVERFLOW!\n");
			return -1;
		}
		new_lineptr_len = *n * 2;

		if ((new_lineptr = (char *)realloc(*lineptr, new_lineptr_len)) == NULL) {
			errno = ENOMEM;
			printf("Error: ENOMEM for realloc!\n");
			return -1;
		}
		*lineptr = new_lineptr;
		*n = new_lineptr_len;
	}

	*cur_pos++ = c;

	if (c == '\r' || c == '\n')
		break;
	}

	*cur_pos = '\0';
	return (ssize_t)(cur_pos - *lineptr);
}

static void trim(char *ptr) {
	int i = 0;
	int j = 0;

	while(ptr[j] != '\0') {
		if(ptr[j] == 0x20 || ptr[j] == 0x09 || ptr[j] == '\n' || ptr[j] == '\r') {
			++j;
			ptr[i] = ptr[j];
		} else {
			ptr[i] = ptr[j];
			++i;
			++j;
		}
	}
	ptr[i] = '\0';
}

static int check_valid_unit(char *in) {
	int i, ret=0;

	if (strlen(in) < 8)
		return ret;

	for (i=0; i<8; ++i) {
		if ((in[i] >= '0' && in[i] <= '9') || (in[i] >= 'A' && in[i] <= 'Z') || (in[i] >= 'a' && in[i] <= 'z'))
			ret += 1;
	}

	if (ret == 8)
		return 1;
	else
		return 0;
}

static int xflasher_gen(char *prog)
{
	DIR *dp = NULL;
	FILE *fp = NULL;
	struct dirent *ep = NULL;
	char *ret = NULL;
	char *progname = basenamee(prog);

#ifdef _WIN32
	static const char blabla[] = "@setlocal enableextensions enabledelayedexpansion\r\n\r\n:::NOTE:\r\n"
			":::      first command must write loader to the device! Or in case newer sony device which doesn't have loader\r\n"
			":::      first parameter must be noloader instead of loader.sin!\r\n"
			":::      All next commands set argument 0 as a loader since loader or noloader is need only in first command!\r\n"
			":::      Next command in this example make s1 dump to file called tadump.ta.\r\n"
			":::\r\n"
			":::      Remember, loader or noloader must be first parameter and no more than one time, and must be first parameter!\r\n"
			":::      0 mean skip command from table!\r\n"
			":::      unlock bootloader parameter is 0 for skip unlocking, or unlock bootloader KEY for unlocking!\r\n"
			":::      dump ta parameter 0 is for skip s1 dumping, or 1 for dumping s1, dump will be stored in file caled tadump.ta\r\n"
			":::      Enjoy tool!\r\n"
			":::\r\n"
			":::Table of commands with example steps for using xflasher tool:\r\n"
			":::-----------------------------------------------------------------------------------------------------------------------------------------------------\r\n"
			":::xflasher   | send loader?  |   dump ta?  |  unlock bootloader? KEY | USB_VID | USB_PID |  FLASH BOOT DELIVERY? | flash ta file? | write log to file\r\n"
			":::-----------------------------------------------------------------------------------------------------------------------------------------------------\r\n";
#else
	static const char blabla[] = "#NOTE:\n"
			"#      first command must write loader to the device! Or in case newer sony device which doesn't have loader\n"
			"#      first parameter must be noloader instead of loader.sin!\n"
			"#      All next commands set argument 0 as a loader since loader or noloader is need only in first command!\n"
			"#      Next command in this example make s1 dump to file called tadump.ta.\n"
			"#\n"
			"#      Remember, loader or noloader must be first parameter and no more than one time, and must be first parameter!\n"
			"#      0 mean skip command from table!\n"
			"#      unlock bootloader parameter is 0 for skip unlocking, or unlock bootloader KEY for unlocking!\n"
			"#      dump ta parameter 0 is for skip s1 dumping, or 1 for dumping s1, dump will be stored in file caled tadump.ta\n"
			"#      Enjoy tool!\n"
			"#\n"
			"#Table of commands with example steps for using xflasher tool:\n"
			"#-----------------------------------------------------------------------------------------------------------------------------------------------------\n"
			"#xflasher   | send loader?  |   dump ta?  |  unlock bootloader? KEY | USB_VID | USB_PID |  FLASH BOOT DELIVERY? | flash ta file? | write log to file\n"
			"#-----------------------------------------------------------------------------------------------------------------------------------------------------\n";
#endif

#ifdef _WIN32
	if ((fp = fopen("xflasher.bat", "wb")) != NULL)
#else
	if ((fp = fopen("xflasher.sh", "wb")) != NULL)
#endif
	{
		FILE *chk = NULL;
		int comment;

		fprintf(fp, "%s", blabla);

#ifdef _WIN32
		fprintf(fp, "\r\n:::upload loader.sin (please change this to loader.sin in case your device uses loader.sin)\r\n"
				":::                  (please change to noloader in case your device don't use loader!     )\r\n");

		if ((chk = fopen("loader.sin", "rb")) != NULL) {
			fclose(chk);
			fprintf(fp, "%s  loader.sin  0  0  0FCE  ADDE  0  0  >xflasher.log & if %%ERRORLEVEL%% neq 0 ( exit %%ERRORLEVEL%% )\r\n\r\n", progname);
		}
		else
		{
			fprintf(fp, "%s  noloader  0  0  0FCE  ADDE  0  0  >xflasher.log & if %%ERRORLEVEL%% neq 0 ( exit %%ERRORLEVEL%% )\r\n\r\n", progname);
		}
#else
		fprintf(fp, "\n#upload loader.sin (please change this to loader.sin in case your device uses loader.sin)\n"
				"#                  (please change to noloader in case your device don't use loader!     )\n");

		if ((chk = fopen("loader.sin", "rb")) != NULL) {
			fclose(chk);
			fprintf(fp, "./%s  loader.sin  0  0  0FCE  ADDE  0  0  >xflasher.log ; rc=$?; if [[ $rc != 0 ]]; then exit $rc; fi\n\n", progname);
		}
		else
		{
			fprintf(fp, "./%s  noloader  0  0  0FCE  ADDE  0  0  >xflasher.log ; rc=$?; if [[ $rc != 0 ]]; then exit $rc; fi\n\n", progname);
		}
#endif

		comment = 1;

		if ((dp = opendir(".")) != NULL)
		{
			while ((ep = readdir(dp)) != NULL)
			{
				/*if (ep->d_type == DT_REG)*/
				{
					if (strcmp(ep->d_name, ".") != 0 && strcmp(ep->d_name, "..") != 0)
					{
						if ((ret = strrchr(ep->d_name, '.')) != NULL)
						{
							if (strcmp(ret, ".sin") == 0) {
								if (comment && strcmp(ep->d_name, "loader.sin") != 0) {
#ifdef _WIN32
									fprintf(fp, ":::flash sin files\r\n");
#else
									fprintf(fp, "#flash sin files\n");
#endif
									comment = 0;
								}
								if (strcmp(ep->d_name, "loader.sin") != 0 && strstr(ep->d_name, "partition") != NULL) {
#ifdef _WIN32
									fprintf(fp, "%s  %s  0  0  0FCE  ADDE  0  0  >>xflasher.log & if %%ERRORLEVEL%% neq 0 ( exit %%ERRORLEVEL%% )\r\n", progname, ep->d_name);
#else
									fprintf(fp, "./%s  %s  0  0  0FCE  ADDE  0  0  >>xflasher.log ; rc=$?; if [[ $rc != 0 ]]; then exit $rc; fi\n", progname, ep->d_name);
#endif
								}
							}
						}
					}
				}
			}
			closedir(dp);
		}

		if ((dp = opendir(".")) != NULL)
		{
			while ((ep = readdir(dp)) != NULL)
			{
				/*if (ep->d_type == DT_REG)*/
				{
					if (strcmp(ep->d_name, ".") != 0 && strcmp(ep->d_name, "..") != 0)
					{
						if ((ret = strrchr(ep->d_name, '.')) != NULL)
						{
							if (strcmp(ret, ".sin") == 0) {
								if (strcmp(ep->d_name, "loader.sin") != 0 && strstr(ep->d_name, "partition") == NULL) {
#ifdef _WIN32
									fprintf(fp, "%s  %s  0  0  0FCE  ADDE  0  0  >>xflasher.log & if %%ERRORLEVEL%% neq 0 ( exit %%ERRORLEVEL%% )\r\n", progname, ep->d_name);
#else
									fprintf(fp, "./%s  %s  0  0  0FCE  ADDE  0  0  >>xflasher.log ; rc=$?; if [[ $rc != 0 ]]; then exit $rc; fi\n", progname, ep->d_name);
#endif
								}
							}
						}
					}
				}
			}
			closedir(dp);
		}

		comment = 1;

		if ((dp = opendir(".")) != NULL)
		{
			while ((ep = readdir(dp)) != NULL)
			{
				/*if (ep->d_type == DT_REG)*/
				{
					if (strcmp(ep->d_name, ".") != 0 && strcmp(ep->d_name, "..") != 0)
					{
						if ((ret = strrchr(ep->d_name, '.')) != NULL)
						{
							if (strcmp(ret, ".ta") == 0) {
								if (comment) {
#ifdef _WIN32
									fprintf(fp, "\r\n:::flash ta files\r\n");
#else
									fprintf(fp, "\n#flash ta files\n");
#endif
									comment = 0;
								}
#ifdef _WIN32
								fprintf(fp, "%s  0  0  0  0FCE  ADDE  0  %s  >>xflasher.log & if %%ERRORLEVEL%% neq 0 ( exit %%ERRORLEVEL%% )\r\n", progname, ep->d_name);
#else
								fprintf(fp, "./%s  0  0  0  0FCE  ADDE  0  %s  >>xflasher.log ; rc=$?; if [[ $rc != 0 ]]; then exit $rc; fi\n", progname, ep->d_name);
#endif
							}
						}
					}
				}
			}
			closedir(dp);
		}

		if ((dp = opendir("boot")) != NULL)
		{
#ifdef _WIN32
			fprintf(fp, "\r\n:::flash boot delivery\r\n%s  0  0  0  0FCE  ADDE  1  0  >>xflasher.log & if %%ERRORLEVEL%% neq 0 ( exit %%ERRORLEVEL%% )\r\n\r\n", progname);
#else
			fprintf(fp, "\n#flash boot delivery\n./%s  0  0  0  0FCE  ADDE  1  0  >>xflasher.log ; rc=$?; if [[ $rc != 0 ]]; then exit $rc; fi\n\n", progname);
#endif
			closedir(dp);
		}

		fclose(fp);
#ifndef _WIN32
		printf("xflasher.sh script is created, see it!\n");
		printf("Setting permission for xflasher.sh returned: %s\n\n",
			 (system("chmod 755 ./xflasher.sh") == 0) ? "ok." : "error!");
#endif
	}

	return 0;
}

static int write_sin(char *filename, HANDLE dev)
{
	FILE *sin = NULL;
	unsigned long long sin_sz;

	char *out = NULL;
	unsigned char *hs = NULL;
	unsigned long sin_header_sz;
	unsigned long long sin_data_sz;

	unsigned int chunk;
	unsigned int chunk_i = 0;
	unsigned int i = 0;

	unsigned int MAX_PACKET;

	clock_t start = 0;

	if ((sin = fopen64(filename, "rb")) == 0) {
		printf("Error: unable to open %s!\n", filename);
		return 0;
	}

	fseeko64(sin, 0, SEEK_END);
	sin_sz = ftello64(sin);
	fseeko64(sin, 0, SEEK_SET);

	if (sin_sz < 1024) {
		printf("Error: is %s valid?\n", filename);
		if (sin) fclose(sin);
		return 0;
	}

	fread_unus_res(tmp, 4, 1, sin);

	if (memcmp(tmp, "\x03\x53\x49\x4E", 4) != 0) {
		printf("Error: file is not a sin!?\n");
		if (sin) fclose(sin);
		return 0;
	}

	printf("Proccessing %s...\n", filename);
	printf("File size: 0x%llX\n", sin_sz);

	fread_unus_res(&sin_header_sz, sizeof(unsigned long), 1, sin);

	sin_header_sz = swap_uint32(sin_header_sz);

	printf("Sin header size: 0x%lX\n", sin_header_sz);

	sin_data_sz = sin_sz - sin_header_sz;
	printf("Sin data size: 0x%llX\n", sin_data_sz);

	if (sin_header_sz < 1 || sin_data_sz < 1) {
		printf("Error: sin header size or sin data size is 0!\n");
		if (sin) fclose(sin);
		return 0;
	}

	{
		unsigned int max_pkt_read_sz = 0;
		char bla[9];
		FILE *blaw = fopen("max_p", "rb");
		if (blaw != NULL) {
			fread_unus_res(bla, 8, 1, blaw);
			bla[8] = '\0';
			fclose(blaw);
			sscanf(bla, "%x", &max_pkt_read_sz);
		}
		else
		{
			DisplayError(TEXT("Unable to open max_p for read!"));
			if (sin) fclose(sin);
			return 0;
		}

		if (max_pkt_read_sz)
			MAX_PACKET = max_pkt_read_sz;
		else {
			printf("Error, max_pkt_read_sz!\n");
			if (sin) fclose(sin);
			return 0;
		}

		printf("Setting max packet size to: 0x%X\n", MAX_PACKET);
	}

	hs = (unsigned char *)malloc(4);
	hs[0] = (sin_header_sz >> 24) & 0xff;
	hs[1] = (sin_header_sz >> 16) & 0xff;
	hs[2] = (sin_header_sz >> 8) & 0xff;
	hs[3] = sin_header_sz & 0xff;

	memcpy(command, CMD_WRITE_SIN_HEADER, 4);
	memcpy(command+4, FLAG3, 4);
	memcpy(command+8, hs, 4);
	sum = getSUM(command, 12);
	memcpy(command+12, sum, 1);
	if (hs) free(hs);

	display_buffer_hex("Sending command...\nRaw command", command, sizeof(command));

	if (transfer_bulk_async(dev, EP_OUT, command, sizeof(command), USB_TIMEOUT, 1) < 1) {
		printf("Error writing command CMD_WRITE_SIN_HEADER!\n");
		if (sin) fclose(sin);
		return 0;
	}

	fseeko64(sin, 0, SEEK_SET);

	out = (char *)malloc(sin_header_sz);
	if (out == NULL) {
		printf("Error: malloc sin header failed, out of memory!");
		if (sin) fclose(sin);
		return 0;
	}
	fread_unus_res(out, sin_header_sz, 1, sin);

	printf("Writing sin header with size of 0x%lx bytes\n", sin_header_sz);

	if (transfer_bulk_async(dev, EP_OUT, out, sin_header_sz, USB_TIMEOUT, 1) < 1) {
		printf("Error writing sin header!\n");
		if (sin) fclose(sin);
		if (out) free(out);
		return 0;
	}

	printf("Sin header writen.\n");

	crc32 = getCRC32(out, sin_header_sz);
	display_buffer_hex("CRC32", crc32, 4);

	if (out) free(out);

	printf("Writing crc32 for sin header...\n");

	if (transfer_bulk_async(dev, EP_OUT, crc32, 4, USB_TIMEOUT, 1) < 1) {
		printf("error writing crc32:\n");
		if (sin) fclose(sin);
		return 0;
	}

	tmp2 = verify(dev, EP_IN, USB_TIMEOUT);
	if (tmp2 != NULL) {
		printf("Verifying crc32...\n");

		if (strstr(tmp2, "0000000500000001") == NULL) {
			printf("Error: device reported that sin header is not ok!\n");
			if (sin) fclose(sin);
			free(tmp2);
			return 0;
		} else
			printf("success: device replied with %s which mean ok.\n", tmp2);

		printf("\n");
		free(tmp2);

	}
	else
	{
		printf("Unable to get reaply from device related to sin header verification!\n\n");
		if (sin) fclose(sin);
		return 0;
	}

	if (get_reaply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT) < 1) {
		if (sin) fclose(sin);
		return 0;
	}

	fseeko64(sin, sin_header_sz, SEEK_SET);

	for (chunk_i=0; chunk_i < sin_data_sz;)
	{
		int timeout = USB_TIMEOUT;
		i += 1;
		chunk_i += MAX_PACKET;

		if (chunk_i > sin_data_sz)
			chunk = (unsigned int)(sin_data_sz - (chunk_i - MAX_PACKET));
		else
			chunk = MAX_PACKET;

		rest[0] = (chunk >> 24) & 0xff;
		rest[1] = (chunk >> 16) & 0xff;
		rest[2] = (chunk >> 8) & 0xff;
		rest[3] = chunk & 0xff;
		memcpy(command, CMD_WRITE_SIN, 4);
		if (chunk_i > sin_data_sz)
			memcpy(command+4, FLAG3, 4);
		else
			memcpy(command+4, FLAG7, 4);
		memcpy(command+8, rest, 4);
		sum = getSUM(command, 12);
		memcpy(command+12, sum, 1);

		display_buffer_hex("Sending command...\nRaw command", command, sizeof(command));

		if (transfer_bulk_async(dev, EP_OUT, command, sizeof(command), USB_TIMEOUT, 1) < 1) {
				printf("Error writing command CMD_WRITE_SIN!\n");
				if (sin) fclose(sin);
				return 0;
		}

		printf("Writing chunk part: %d with size of 0x%x bytes\n", i, chunk);

		out = (char *)malloc(chunk);

		if (out == NULL) {
			printf("Error: malloc chunk data failed, out of memory!");
			if (sin) fclose(sin);
			return 0;
		}

		fread_unus_res(out, chunk, 1, sin);

		if (transfer_bulk_async(dev, EP_OUT, out, chunk, USB_TIMEOUT, 1) < 1) {
			printf("Error writing chunk part: %d\n", i);
			if (sin) fclose(sin);
			if (out) free(out);
			return 0;
		}
		/* display_buffer_hex("raw", out, chunk); */

		printf("Chunk part: %d writen.\n", i);

		crc32 = getCRC32(out, chunk);
		display_buffer_hex("CRC32", crc32, 4);

		if (out) free(out);

		printf("Writing crc32 for chunk part: %d\n", i);

		if (transfer_bulk_async(dev, EP_OUT, crc32, 4, USB_TIMEOUT, 1) < 1) {
			printf("Error writing crc32 for chunk part: %d\n", i);
			if (sin) fclose(sin);
			return 0;
		}

		start = clock();

		if (chunk_i > sin_data_sz) {
			printf("WRITING LAST PACKET. SETTING TIMEOUT TO %d (ms).\n", USB_VERIFY_TIMEOUT);
			timeout = USB_VERIFY_TIMEOUT;
		} else {
			timeout = USB_TIMEOUT;
		}

		/* wait timeout max 10 minutes until device proccesses sin and send reaply */
		tmp2 = verify(dev, EP_IN, timeout);

		if (tmp2 != NULL)
		{
			printf("Verifying crc32...\n");

			if (chunk_i > sin_data_sz)    /* case for last packet verify */
			{
				if (strstr(tmp2, "0000000600000001") == NULL && strstr(tmp2, "0000000600000000") == NULL) {
					printf("Error: device reported that crc32 for chunk part: %d is not ok!\n", i);
					if (sin) fclose(sin);
					free(tmp2);
					return 0;
				} else
					printf("Success: device replied with %s which mean ok.\n", tmp2);
			}
			else
			{
				if (strstr(tmp2, "0000000600000001") == NULL) {
					printf("Error: device reported that crc32 for chunk part: %d is not ok!\n", i);
					if (sin) fclose(sin);
					free(tmp2);
					return 0;
				} else
					printf("Success: device replied with %s which mean ok.\n", tmp2);
			}

			printf("\n");
			free(tmp2);
			if (chunk_i > sin_data_sz) {
				clock_t stop = clock();
				double elapsed = (double)(stop - start) * 1000.0 / CLOCKS_PER_SEC;
				printf("LAST PACKET REAPLY AFTER ms: %f\n", elapsed);
			}
		}
		else
		{
			printf("Unable to get reaply from device related to crc32 chunk part: %d verification!\n\n", i);
			if (sin) fclose(sin);
			if (chunk_i > sin_data_sz) {
				clock_t stop = clock();
				double elapsed = (double)(stop - start) * 1000.0 / CLOCKS_PER_SEC;
				printf("LAST PACKET REAPLY AFTER ms: %f , SEEMS TIMEOUT OCOURED!\n", elapsed);
			}
			return 0;
		}

		if (get_reaply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT) < 1) {
			if (sin) fclose(sin);
			return 0;
		}
	}

	printf("Finished.\n\n\n");

	if (sin)
		fclose(sin);

	return 1;
}

static int open_ta(int unit_partition, HANDLE dev) {
	memset(part, '\0', 1);

	if (unit_partition > 16) {
		printf("Error: ta unit partition argument is max 16!\n");
		return 0;
	}

	memcpy(command, CMD_OPEN_TA, 4);
	memcpy(command+4, FLAG3, 4);
	memcpy(command+8, "\x00\x00\x00\x01", 4);
	sum = getSUM(command, 12);
	memcpy(command+12, sum, 1);

	printf("Opening ta...\n");

	display_buffer_hex("Sending command...\nRaw command", command, sizeof(command));

	if (transfer_bulk_async(dev, EP_OUT, command, sizeof(command), USB_TIMEOUT, 1) < 1) {
		printf("Error writing command CMD_OPEN_TA!\n");
		return 0;
	}

	printf("Want open ta unit partition: %d\n", unit_partition);

	part[0] = unit_partition & 0xff;

	if (transfer_bulk_async(dev, EP_OUT, part, 1, USB_TIMEOUT, 1) < 1) {
		printf("Error writing ta unit partition argument!\n");
		return 0;
	}

	printf("Ta unit partition argument writen.\n");

	crc32 = getCRC32(part, 1); /* was 2 hmm, why I set it to 2? See it later */
	display_buffer_hex("CRC32", crc32, 4);

	printf("Writing crc32 for ta unit partition argument...\n");

	if (transfer_bulk_async(dev, EP_OUT, crc32, 4, USB_TIMEOUT, 1) < 1) {
		printf("Error writing crc32!\n");
		return 0;
	}

	tmp2 = verify(dev, EP_IN, USB_TIMEOUT);
	if (tmp2 != NULL) {
		printf("Verifying crc32...\n");

		if (strstr(tmp2, "0000000900000001") == NULL) {
			printf("Error: device reported that ta unit partition: %d not exist!\n", unit_partition);
			free(tmp2);
			return 0;
		} else
			printf("success: device replied with %s which mean ta unit partition: %d is opened.\n", tmp2, unit_partition);

		printf("\n");
		free(tmp2);

	} else {
		printf("Unable to get reaply from device related to ta unit partition argument!\n\n");
		return 0;
	}

	if (get_reaply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT) < 1)
		return 0;

	return 1;
}

static int dump_ta(const char *unit, HANDLE dev, FILE *fp) {
	int i, read, ret=1;

	memset(command, '\0', 13);

	memcpy(command, CMD_READ_TA, 4);
	memcpy(command+4, FLAG3, 4);
	memcpy(command+8, "\x00\x00\x00\x04", 4);
	sum = getSUM(command, 12);
	memcpy(command+12, sum, 1);

	printf("Want read ta unit %02X%02X%02X%02X...\n\n", unit[0]&0xff, unit[1]&0xff, unit[2]&0xff, unit[3]&0xff);

	display_buffer_hex("Sending command...\nCommand raw", command, sizeof(command));

	if (transfer_bulk_async(dev, EP_OUT, command, sizeof(command), USB_TIMEOUT, 1) < 1) {
		printf("Error writing command CMD_READ_TA!\n");
		return 0;
	}

	display_buffer_hex("Sending command...\nWant unit raw", (char *)unit, 4);

	if (transfer_bulk_async(dev, EP_OUT, (char *)unit, 4, USB_TIMEOUT, 1) < 1) {
		printf("Error writing command CMD_READ_TA!\n");
		return 0;
	}

	crc32 = getCRC32((char *)unit, 4);
	display_buffer_hex("CRC32", crc32, 4);

	printf("Writing crc32 for want read ta unit...\n");

	if (transfer_bulk_async(dev, EP_OUT, crc32, 4, USB_TIMEOUT, 1) < 1) {
		printf("Error writing crc32!\n");
		return 0;
	}

	tmp2 = verify(dev, EP_IN, USB_TIMEOUT);
	if (tmp2 != NULL) {
		printf("Verifying crc32...\n");

		if (strstr(tmp2, "0000000C00000001") == NULL) {
			printf("Error: device reported that wanted ta unit is not found or can not be read!\n");
			free(tmp2);
			ret = 0;
		} else
			printf("success: device replied with %s which mean wanted ta unit is read.\n", tmp2);

		printf("\n");
		free(tmp2);

	} else {
		printf("Unable to get reaply from device related to wanted ta unit!\n\n");
		return 0;
	}

	read = get_reaply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT);
	if (read < 1) {
		return 0;
	} else {
		if (ret) {
			printf("Raw unit data:\n");

			printf("%02X%02X%02X%02X %04X ", unit[0]&0xff, unit[1]&0xff, unit[2]&0xff, unit[3]&0xff, read);

			if (fp != NULL)
			{
				tmp2 = (char *)malloc(9);

				for (i=0; i<4; ++i) {
					snprintf(tmp2+i*2, 9, "%02X", unit[i]&0xff);
				}
				fwrite(tmp2, 1, 8, fp);
				fwrite("\x20", 1, 1, fp);

				snprintf(tmp2, 3, "%02X", (read>>8)&0xff);
				snprintf(tmp2+2, 3, "%02X", read&0xff);
				fwrite(tmp2, 1, 4, fp);
				fwrite("\x20", 1, 1, fp);
			}

			for (i=0; i<read; ++i)
			{
				printf("%02X", tmp[i]&0xff);

				if (fp != NULL) {
					fwrite("\x20", 1, 1, fp);
					snprintf(tmp2, 3, "%02X", tmp[i]&0xff);
					fwrite(tmp2, 1, 2, fp);
				}
			}
			printf("\n\n");

			if (fp != NULL) {
				fwrite("\n", 1, 1, fp);
				free(tmp2);
			}
		}
	}

	if (ret) {
		if (get_reaply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT) < 1)
			return 0;
	}

	return ret;
}

static int read_ta_howmany = 0;

static char *read_ta(const char *unit, HANDLE dev) {
	int read, ret=1;

	char *unit_buff = NULL;

	memset(command, '\0', 13);

	memcpy(command, CMD_READ_TA, 4);
	memcpy(command+4, FLAG3, 4);
	memcpy(command+8, "\x00\x00\x00\x04", 4);
	sum = getSUM(command, 12);
	memcpy(command+12, sum, 1);

	printf("Want read ta unit %02X%02X%02X%02X...\n\n", unit[0]&0xff, unit[1]&0xff, unit[2]&0xff, unit[3]&0xff);

	display_buffer_hex("Sending command...\nCommand raw", command, sizeof(command));

	if (transfer_bulk_async(dev, EP_OUT, command, sizeof(command), USB_TIMEOUT, 1) < 1) {
		printf("Error writing command CMD_READ_TA!\n");
          read_ta_howmany = 0;
		return NULL;
	}

	display_buffer_hex("Sending command...\nWant unit raw", (char *)unit, 4);

	if (transfer_bulk_async(dev, EP_OUT, (char *)unit, 4, USB_TIMEOUT, 1) < 1) {
		printf("Error writing command CMD_READ_TA!\n");
          read_ta_howmany = 0;
		return NULL;
	}

	crc32 = getCRC32((char *)unit, 4);
	display_buffer_hex("CRC32", crc32, 4);

	printf("Writing crc32 for want read ta unit...\n");

	if (transfer_bulk_async(dev, EP_OUT, crc32, 4, USB_TIMEOUT, 1) < 1) {
		printf("Error writing crc32!\n");
          read_ta_howmany = 0;
		return NULL;
	}

	tmp2 = verify(dev, EP_IN, USB_TIMEOUT);
	if (tmp2 != NULL) {
		printf("Verifying crc32...\n");

		if (strstr(tmp2, "0000000C00000001") == NULL) {
			printf("Error: device reported that wanted ta unit is not found or can not be read!\n");
			free(tmp2);
			ret = 0;
		} else
			printf("success: device replied with %s which mean wanted ta unit is read.\n", tmp2);

		printf("\n");
		free(tmp2);

	} else {
		printf("Unable to get reaply from device related to wanted ta unit!\n\n");
          read_ta_howmany = 0;
		return NULL;
	}

	read = get_reaply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT);
	if (read < 1) {
          read_ta_howmany = 0;
		return NULL;
	} else {
		unit_buff = (char *)malloc(read+1);
		memset(unit_buff, 0, read+1);
		memcpy(unit_buff, tmp, read);
	}

	if (ret) {
		if (get_reaply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT) < 1) {
			free(unit_buff);
               read_ta_howmany = 0;
			return NULL;
		}
	}

	unit_buff_sz = read;
	read_ta_howmany = read;

	return unit_buff;
}

static int write_ta(const char *unit, HANDLE dev, char *data, unsigned long size) {
	memset(command, '\0', 13);

	memcpy(command, CMD_WRITE_TA, 4);
	memcpy(command+4, FLAG3, 4);
	rest[0] = (size >> 24) & 0xff;
	rest[1] = (size >> 16) & 0xff;
	rest[2] = (size >> 8) & 0xff;
	rest[3] = size & 0xff;
	memcpy(command+8, rest, 4);
	sum = getSUM(command, 12);
	memcpy(command+12, sum, 1);

	printf("Want write ta unit %02X%02X%02X%02X...\n\n", unit[0]&0xff, unit[1]&0xff, unit[2]&0xff, unit[3]&0xff);

	display_buffer_hex("Sending command...\nCommand raw", command, sizeof(command));

	if (transfer_bulk_async(dev, EP_OUT, command, sizeof(command), USB_TIMEOUT, 1) < 1) {
		printf("Error writing command CMD_WRITE_TA!\n");
		return 0;
	}

	display_buffer_hex("Sending command...\nWriting data into unit", (char *)unit, 4);

	if (transfer_bulk_async(dev, EP_OUT, data, size, USB_TIMEOUT, 1) < 1) {
		printf("Error writing unit data!\n");
		return 0;
	}

	crc32 = getCRC32(data, size);
	display_buffer_hex("CRC32", crc32, 4);

	printf("Writing crc32 for data...\n");

	if (transfer_bulk_async(dev, EP_OUT, crc32, 4, USB_TIMEOUT, 1) < 1) {
		printf("Error writing crc32!\n");
		return 0;
	}

	tmp2 = verify(dev, EP_IN, USB_TIMEOUT);
	if (tmp2 != NULL) {
		printf("Verifying crc32...\n");

		if (strstr(tmp2, "0000000D00000001") == NULL) {
			printf("Error: device reported that unit data is not writen!\n");
			free(tmp2);
			return 0;
		} else
			printf("success: device replied with %s which mean unit data is writen.\n", tmp2);

		printf("\n");
		free(tmp2);

	} else {
		printf("Unable to get reaply from device related to writing unit data!\n\n");
		return 0;
	}

	if (get_reaply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT) < 1)
		return 0;

	return 1;
}

static int close_ta(int unit_partition, HANDLE dev) {
	printf("Closing ta unit partition: %d\n", unit_partition);

	if (transfer_bulk_async(dev, EP_OUT, "\x00\x00\x00\x0A\x00\x00\x00\x03\x00\x00\x00\x00\x10\x00\x00\x00\x00", 17, USB_TIMEOUT, 1) < 1) {
		printf("Error writing command CMD_CLOSE_TA!\n");
		return 0;
	}

	tmp2 = verify(dev, EP_IN, USB_TIMEOUT);
	if (tmp2 != NULL) {
		if (strstr(tmp2, "0000000A00000001") == NULL) {
			printf("Error: device reported that ta unit partition is not closed!\n");
			free(tmp2);
			return 0;
		} else
			printf("success: device replied with %s which mean ta unit partition is closed.\n", tmp2);

		printf("\n");
		free(tmp2);
	} else {
		printf("Unable to get reaply from device related to ta unit partition closing!\n\n");
		return 0;
	}

	if (get_reaply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT) < 1)
		return 0;

	return 1;
}

#if 0
static int write_hook(HANDLE dev) {
	printf("Writing hook...\n");

	if (transfer_bulk_async(dev, EP_OUT, huk_cmd, sizeof(huk_cmd), USB_TIMEOUT, 1) < 1) {
		printf("Error writing huk!\n");
		return 0;
	}

	tmp2 = verify(dev, EP_IN, USB_TIMEOUT);
	if (tmp2 != NULL) {
		printf("Verifying huk...\n");

		if (strstr(tmp2, "0000001900000001") == NULL) {
			printf("Error: device reported that huk is not ok!\n");
			free(tmp2);
			return 0;
		} else
			printf("success: device replied with %s which mean huk ok.\n", tmp2);

		printf("\n");
		free(tmp2);

	} else {
		printf("Unable to get reaply from device related to writing huk!\n\n");
		return 0;
	}

	if (get_reaply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT) < 1)
		return 0;

	return 1;
}
#endif

/* track the current level in the xml tree */
static int depth = 0;

static char bootdelivery_xml[15][15][200];
static char bootdelivery_version[100];
static int td1 = 0;
static int td2 = 0;
static int td3 = 0;

/* {"CONFIGURATION", "ATTRIBUTES", "BOOT_CONFIG", "BOOT_IMAGES", "emmc", "s1", "sbl1", "tz", "..."}; */

/* first when start element is encountered */
static void XMLCALL start_element(void *data, const char *element, const char **attribute) {
	int i;

	/* for (i=0; i<depth; i++)
		printf("    ");

	printf("%s", element); */

	if (depth == 0) {
		if (memcmp(element, "BOOT_DELIVERY", strlen(element)) == 0) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "SPACE_ID", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(bootdelivery_version, sizeof(bootdelivery_version), "%s", attribute[i+1]);
				}
			}
		}
	}

	if (depth == 1) {
		if (memcmp(element, "CONFIGURATION", strlen(element)) == 0) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "NAME", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(bootdelivery_xml[td1][0], sizeof(bootdelivery_xml[td1][0]), "%s", attribute[i+1]);
				}
			}
		}
	}

	if (depth == 2) {
		if (memcmp(element, "BOOT_CONFIG", strlen(element)) == 0)
			td2++;

		if (memcmp(element, "BOOT_IMAGES", strlen(element)) == 0)
			td3++;

		if (memcmp(element, "ATTRIBUTES", strlen(element)) == 0) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "VALUE", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(bootdelivery_xml[td1][1], sizeof(bootdelivery_xml[td1][1]), "%s", attribute[i+1]);
				}
			}
		}

		if (memcmp(element, "HWCONFIG", strlen(element)) == 0) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "REVISION", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(bootdelivery_xml[td1][2], sizeof(bootdelivery_xml[td1][2]), "%s", attribute[i+1]);
				}
			}
		}
	}

	if (depth == 3) {
		if (memcmp(element, "FILE", strlen(element)) == 0 && td2) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "PATH", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(bootdelivery_xml[td1][3], sizeof(bootdelivery_xml[td1][3]), "%s", attribute[i+1]);
				}
			}
		}

		if (memcmp(element, "FILE", strlen(element)) == 0 && td3) {
			for (i=0; attribute[i]; i+=2) {
				if (memcmp(attribute[i], "PATH", strlen(attribute[i])) == 0) {
					/* printf(" %s = '%s'", attribute[i], attribute[i+1]); */
					snprintf(bootdelivery_xml[td1][3+td3], sizeof(bootdelivery_xml[td1][3+td3]), "%s", attribute[i+1]);
					td3++;
				}
			}
		}
	}

	depth++;
}

/* decrement the current level of the tree */
static void XMLCALL end_element(void *data, const char *element)
{
	if (memcmp(element, "CONFIGURATION", 13) == 0)
		td1++;

	if (memcmp(element, "BOOT_CONFIG", 11) == 0)
		td2 = 0;

	if (memcmp(element, "BOOT_IMAGES", 11) == 0)
		td3 = 0;

	depth--;
}

static void handle_data(void *data, const char *content, int length) {
	char *tmp = (char *)malloc(length+1);
	strncpy(tmp, content, length);
	tmp[length] = '\0';
	data = (void *)tmp;
	free(tmp);
	if (data) { }
}

static int parse_xml(char *xml_file) {
	int ret = 1;
	char *xml_buff;
	size_t xml_buff_size, xml_file_size=0;
	FILE *fp;

	XML_Parser parser = XML_ParserCreate(NULL);
	if (!parser) {
		printf("Couldn't allocate memory for xml parser!\n");
		return 0;
	}

	fp = fopen64(xml_file, "rb");
	if (fp == NULL) {
		printf("Failed to open %s file\n", xml_file);
		return 0;
	}

	fseeko64(fp, 0, SEEK_END);
	xml_file_size = ftello64(fp);
	fseeko64(fp, 0, SEEK_SET);

	xml_buff_size = xml_file_size;

	XML_SetElementHandler(parser, start_element, end_element);
	XML_SetCharacterDataHandler(parser, handle_data);

	xml_buff = (char *)malloc(xml_buff_size+1);

	fread_unus_res(xml_buff, 1, xml_buff_size, fp);
	xml_buff[xml_buff_size] = '\0';

	/* parse the xml */
	if (XML_Parse(parser, xml_buff, strlen(xml_buff), XML_TRUE) == XML_STATUS_ERROR) {
		printf("Error: at line %" XML_FMT_INT_MOD "u: %s\n", XML_GetCurrentLineNumber(parser), XML_ErrorString(XML_GetErrorCode(parser)));
		ret = 0;
	}

	if (fp)
		fclose(fp);

	XML_ParserFree(parser);

	free(xml_buff);

	return ret;
}

static int proced_ta_file(char *ta_file, HANDLE dev) {
	FILE *fp = NULL;
	char *line = NULL;
	size_t len = 0;
	ssize_t read;

	char unit[9];
	char unit_sz_tmp[5];
	unsigned int unit_sz;
	char *unit_data;
	unsigned int i, j;
	int the_rest=0, partition_oppened=0, finished=0, ret=1;

	printf("Proccessing %s...\n", ta_file);

	if ((unit_data = (char *)malloc(MAX_UNIT_LINE_LEN)) == NULL) {
		printf("Error allocating unit_data!\n");
		return 0;
	}

	if ((fp = fopen64(ta_file, "rb")) == NULL) {
		printf("Unable to open %s!\n", ta_file);
		return 0;
	}

	while((read = g_getline(&line, &len, fp)) != -1)
	{
		switch(read)
		{
			case 1:
				/*LOG("Skipped empty line.\n\n");*/
				break;

			case 3:
				if (line[0] >= 0x30 && line[0] <= 0x39 && line[1] >= 0x30 && line[1] <= 0x39)
				{
					LOG("Found partition: %02x\n", atoi(line));
					if (!partition_oppened)
					{
						printf("Sending command to bootloader to open partition: %02x ...\n", atoi(line));

						partition_oppened = atoi(line);

						if (!open_ta(partition_oppened, dev)) {
							printf("Error, unable to open ta partition %d !\n", partition_oppened);
							if (fp)
								fclose(fp);
							CloseHandle(dev);
							SetupDiDestroyDeviceInfoList(hDevInfo);
							return 1;
						}

						printf("Partition %02X opened.\n", partition_oppened);
						LOG("\n");
					}
					else
					{
						printf("Sending command to bootloader to close partition %02X and open new partition: %02x ...\n",
								 partition_oppened, atoi(line));

						if (!close_ta(partition_oppened, dev)) {
							printf("Error closing ta partition %d !\n", partition_oppened);
							if (fp)
								fclose(fp);
							CloseHandle(dev);
							SetupDiDestroyDeviceInfoList(hDevInfo);
							return 1;
						}

						printf("Partition %02X closed.\n", partition_oppened);

						partition_oppened = atoi(line);

						if (!open_ta(partition_oppened, dev)) {
							printf("Error, unable to open ta partition %d !\n", partition_oppened);
							if (fp)
								fclose(fp);
							CloseHandle(dev);
							SetupDiDestroyDeviceInfoList(hDevInfo);
							return 1;
						}

						printf("Partition %02X opened.\n", partition_oppened);
						LOG("\n");
					}
				}
				break;

			default:
				if (line[0] == '/') {
					/*LOG("Skipped comment line.\n\n");*/
				}
				else
				{
					LOG("Retrieved line of lenght: %lu\n", read);

					if (check_valid_unit(line))
					{
						finished = 0;
						memcpy(unit, line, 8);
						unit[8] = '\0';
						to_uppercase(unit);
						trim(line);
						LOG("Line lenght after trim: %lu\n", strlen(line));
						LOG("Found unit: %s\n", unit);

						/* unit(8) + unit size(4) + at least one hex(2) */
						if (strlen(line) < 14) {
							if (strlen(line) == 12) {
								printf("Found specific unit which don't contain data.\n");
								the_rest = 0;
								finished = 1;
								unit_sz = 0;
							} else {
								printf("Error: corrupted unit! Skipping this unit!\n\n");
								the_rest = 0;
								break;
							}
						}
						else
						{
							memcpy(unit_sz_tmp, line+8, 4);
							unit_sz_tmp[4] = '\0';
							sscanf(unit_sz_tmp, "%X", &unit_sz);
							LOG("Unit size: %02X\n", unit_sz);
							memset(unit_data, '\0', MAX_UNIT_LINE_LEN);
							i = 0;
							do {
								memcpy(unit_data+i, line+12+i, 1);
							} while(++i < strlen(line));
							unit_data[i] = '\0';

							if ((unsigned int)strlen(line)-12 < unit_sz*2) {
								LOG("Data probably continues in a new line (%u not match %u)!\n",
									(unsigned int)strlen(line)-12, unit_sz*2);
								the_rest = 1;
							} else
								the_rest = 0;

							if ((unsigned int)strlen(unit_data) == unit_sz*2)
								finished = 1;
						}

					}
					else
					{
						if (the_rest)
						{
							finished = 0;
							trim(line);
							LOG("Line lenght after trim: %lu\n", strlen(line));
							LOG("Found the rest ot the data!\n");
							j = strlen(unit_data);
							i = 0;
							do {
								memcpy(unit_data+j+i, line+i, 1);
							} while(++i < strlen(line));
							unit_data[j+i] = '\0';

							if ((unsigned int)strlen(unit_data) == unit_sz*2) {
								the_rest = 0;
								finished = 1;
							}
						}
					}

					if (finished)
					{
						char *unit_total_temp = NULL;
						char unit_sz_temp[4];
						char unit_backup[8];

						if (memcmp(unit, "000008B2", 8) == 0 || /* unlock key */
						    memcmp(unit, "000007D3", 8) == 0 || /* dangerous */
						    memcmp(unit, "000007DA", 8) == 0 || /* sim lock */
						    memcmp(unit, "00000851", 8) == 0 || /* dangerous */
						    memcmp(unit, "000008A2", 8) == 0 || /* device name */
						    memcmp(unit, "00001324", 8) == 0 || /* device sn */
						    memcmp(unit, "0001046B", 8) == 0) { /* drm key */
							printf("Skipping unit %s\n", unit);
							continue;
						}

						if ((unit_total_temp = (char *)malloc(unit_sz+8)) == NULL) {
							printf("Error allocating unit_temp!\n");
							ret = 0;
							goto finish_proced_ta;
						}

						finished = 0;
						printf("\n<<-------------------- Retrieval finished! Found unit: %s,"
							" Unit size: %04X, Unit data:%s\n",
							 unit, unit_sz, unit_sz ? "" : " NULL");
						to_ascii(unit_data, unit_data);
						display_buffer_hex(unit, unit_data, unit_sz);
						printf("---------------------->> Writing ta unit data to phone...\n");

						unit_sz_temp[0] = (unit_sz >> 24) & 0xff;
						unit_sz_temp[1] = (unit_sz >> 16) & 0xff;
						unit_sz_temp[2] = (unit_sz >> 8) & 0xff;
						unit_sz_temp[3] = (unit_sz >> 0) & 0xff;
						memcpy(unit_backup, unit, 8);
						to_ascii(unit, unit);
						memcpy(unit_total_temp, unit, 4);
						memcpy(unit_total_temp+4, unit_sz_temp, 4);
						if (unit_sz)
							memcpy(unit_total_temp+8, unit_data, unit_sz);

						if (!write_ta(unit, dev, unit_total_temp, unit_sz+8))
						{
							printf("Error writing unit: %s!\n", unit_backup);
							ret = 0;
							free(unit_total_temp);
							goto finish_proced_ta;
						}

						free(unit_total_temp);
					}
					LOG("\n");
				}
				break;
		}
	}

finish_proced_ta:
	if (unit_data)
		free(unit_data);

	if (fp)
		fclose(fp);

	if (line)
		free(line);

	if (partition_oppened)
	{
		printf("Sending command to bootloader to close ta.\n");
		LOG("\n");

		if (!close_ta(partition_oppened, dev)) {
			printf("Error closing ta partition %02X !\n", partition_oppened);
			if (fp)
				fclose(fp);
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		partition_oppened = 0;
	}

	return ret;
}

int main(int argc, char **argv) {
	int i, j, unlock_allowed=0;
	unsigned short VID, PID;
	char unit[4], rck_h[65], *unit_buff=NULL;
	struct stat filestat;
	FILE *fp = NULL;
	FILE *gt = NULL;
#ifdef _WIN32
	char *device = NULL;
#endif
	HANDLE dev;

	printf("-------------------------------------------------------\n");
	printf("            Xperia Command Line Flasher v23            \n");
	printf("                                                       \n");
	printf("                 by Munjeni @ 2014,2017                     \n");
	printf("-------------------------------------------------------\n");

	if (argc != 8) {
		printf("\nUsage:\n");
		printf("  %s LOADER  DUMP_S1?  UNLOCK_KEY  USB_VID  USB_PID FLASH_BOOT_DELIVERY? FLASH_TA_FILE?\n", argv[0]);
		printf("\nFor example:\n");
		printf("  %s loader.sin 0 AABBCCDDEEFF1122 0FCE ADDE 0 0\n", argv[0]);
		printf("\nFor more info double click exe and see xloader batch file!\n");

		xflasher_gen(argv[0]);

		return 1;
	}

	sscanf(argv[4], "%08hX", &VID);
	sscanf(argv[5], "%08hX", &PID);

#ifdef _WIN32
	device = open_dev(VID, PID);
	if (device[0] == '\0') {
		printf("\nNo usb device with vid:0x%04x pid:0x%04x !\n", VID, PID);
		return 1;
	}

	dev = CreateFile(
		 device,
		 GENERIC_WRITE | GENERIC_READ,
		 FILE_SHARE_READ | FILE_SHARE_WRITE,
		 NULL,
		 OPEN_EXISTING,
		 FILE_FLAG_OVERLAPPED,
		 NULL);

	if (dev == INVALID_HANDLE_VALUE) {
		DisplayError(TEXT("Error CreateFile!"));
		SetupDiDestroyDeviceInfoList(hDevInfo);
		return 1;
	}
#else
	dev = get_flashmode(VID, PID);

	if (dev == NULL) {
		printf("\nNo usb device with vid:0x%04x pid:0x%04x !\n", VID, PID);
		return 1;
	}
#endif

/*///////////// must read 3 times before any command ////////////////*/

	if (strcmp(argv[1], "loader.sin") == 0 || strcmp(argv[1], "noloader") == 0)
	{
		for (i=0; i<3; ++i)
		{
			if (get_reaply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT) < 1) {
				CloseHandle(dev);
				SetupDiDestroyDeviceInfoList(hDevInfo);
				return 1;
			}

			if (i == 1)
			{
				unsigned int strpo = 0;

				memset(device_otp_lock, 0, sizeof(device_otp_lock));
				memset(device_otpdata, 0, sizeof(device_otpdata));
				memset(device_idcode, 0, sizeof(device_idcode));
				memset(device_plfroot, 0, sizeof(device_plfroot));
				memset(bootdelivery_attr, 0, sizeof(bootdelivery_attr));

				if (strstr(tmp, "MAX_PKT_SZ=\"") != NULL)
				{
					unsigned int max_pkt_found_sz = 0;
					char bla[9];
					for (strpo=0; strpo < strlen(tmp)-12; ++strpo) {
						if (memcmp(tmp+strpo, "MAX_PKT_SZ=\"", 12) == 0) {
							memcpy(bla, tmp+strpo+12, 8);
							break;
						}
					}
					bla[8] = '\0';
					sscanf(bla, "%x", &max_pkt_found_sz);
					if (max_pkt_found_sz)
					{
						FILE *blaw = fopen("max_p", "wb");
						if (blaw != NULL) {
							fprintf(blaw, "%s", bla);
							fclose(blaw);
						}
						else
						{
							DisplayError(TEXT("Unable to open max_p for write!"));
							CloseHandle(dev);
							SetupDiDestroyDeviceInfoList(hDevInfo);
							return 1;
						}
					}
					else
					{
						printf("Error, max_pkt_found_sz!\n");
						CloseHandle(dev);
						SetupDiDestroyDeviceInfoList(hDevInfo);
						return 1;
					}
				}
				else
				{
					printf("Can't get MAX_PKT_SZ!\n");
					CloseHandle(dev);
					SetupDiDestroyDeviceInfoList(hDevInfo);
					return 1;
				}

				if (strstr(tmp, "OTP_LOCK_STATUS_1=\"UNLOCKED\"") != NULL)
					snprintf(device_otp_lock, sizeof(device_otp_lock), "OTP_LOCK_STATUS_1=\"UNLOCKED\"");

				if (strstr(tmp, "OTP_LOCK_STATUS_1=\"LOCKED\"") != NULL)
				{
					snprintf(device_otp_lock, sizeof(device_otp_lock), "OTP_LOCK_STATUS_1=\"LOCKED\"");

					if (strstr(tmp, "OTP_DATA_1=\"") != NULL) {
						for (strpo=0; strpo < strlen(tmp)-12; ++strpo) {
							if (memcmp(tmp+strpo, "OTP_DATA_1=\"", 12) == 0) {
								memcpy(device_otpdata, tmp+strpo, 12);
								memcpy(device_otpdata+12, tmp+strpo+12, 9);
								break;
							}
						}
					}

					if (strstr(tmp, "IDCODE_1=\"") != NULL) {
						for (strpo=0; strpo < strlen(tmp)-12; ++strpo) {
							if (memcmp(tmp+strpo, "IDCODE_1=\"", 10) == 0) {
								memcpy(device_idcode, tmp+strpo, 10);
								memset(device_idcode+10, 0x30, 2);
								memcpy(device_idcode+12, tmp+strpo+12, 7);
								break;
							}
						}
					}

					if (strstr(tmp, "PLF_ROOT_1=\"") != NULL) {
						for (strpo=0; strpo < strlen(tmp)-12; ++strpo) {
							if (memcmp(tmp+strpo, "PLF_ROOT_1=\"", 12) == 0) {
								memcpy(device_plfroot, tmp+strpo, 12);
								memcpy(device_plfroot+12, tmp+strpo+12, 65);
								break;
							}
						}
					}
				}

				if (device_otp_lock[0] == 'O' && device_otpdata[0] == 'O' && device_idcode[0] == 'I')
				{
					if ((gt = fopen64("btinf", "wb")) == NULL) {
						printf("Unable to create file btinf!\n");
						CloseHandle(dev);
						SetupDiDestroyDeviceInfoList(hDevInfo);
						return 1;
					}

					if (device_plfroot[0] == 'P')
						fprintf(gt, "%s;%s;%s;%s", device_otp_lock, device_otpdata, device_idcode, device_plfroot);
					else
						fprintf(gt, "%s;%s;%s", device_otp_lock, device_otpdata, device_idcode);

					fclose(gt);
				}
			}
		}
	}

/*///////////////////////// upload loader.sin ////////////////////////*/

	if (strcmp(argv[1], "0") != 0 && strcmp(argv[1], "noloader") != 0) {
		if (write_sin(argv[1], dev)) {
			printf("%s uploaded sucessfully.\n\n", argv[1]);
		} else {
			printf("Error uploading %s!\n\n", argv[1]);
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}
	}

/*///////////// must read 3 times before any command ////////////////*/

	if (strcmp(argv[1], "loader.sin") == 0) {
		for (i=0; i<3; ++i) {
			if (get_reaply(dev, EP_IN, tmp, sizeof(tmp), USB_TIMEOUT) < 1) {
				CloseHandle(dev);
				SetupDiDestroyDeviceInfoList(hDevInfo);
				return 1;
			}

			if (i == 1)
			{
				if (strstr(tmp, "MAX_PKT_SZ=\"") != NULL)
				{
					unsigned int strpo = 0;
					unsigned int max_pkt_found_sz = 0;
					char bla[9];
					for (strpo=0; strpo < strlen(tmp)-12; ++strpo) {
						if (memcmp(tmp+strpo, "MAX_PKT_SZ=\"", 12) == 0) {
							memcpy(bla, tmp+strpo+12, 8);
							break;
						}
					}
					bla[8] = '\0';
					sscanf(bla, "%x", &max_pkt_found_sz);
					if (max_pkt_found_sz)
					{
						FILE *blaw = fopen("max_p", "wb");
						if (blaw != NULL) {
							fprintf(blaw, "%s", bla);
							fclose(blaw);
						}
						else
						{
							DisplayError(TEXT("Unable to open max_p for write 2!"));
							CloseHandle(dev);
							SetupDiDestroyDeviceInfoList(hDevInfo);
							return 1;
						}
					}
					else
					{
						printf("Error, max_pkt_found_sz 2!\n");
			               CloseHandle(dev);
						SetupDiDestroyDeviceInfoList(hDevInfo);
						return 1;
					}
				}
				else
				{
					printf("Can't get MAX_PKT_SZ 2!\n");
					CloseHandle(dev);
					SetupDiDestroyDeviceInfoList(hDevInfo);
					return 1;
				}
			}
		}
	}

/*////////////////////////// dump ta ////////////////////////////////*/

	if (strcmp(argv[2], "1") == 0) {

		fp = fopen64("tadump.ta" , "wb");

		if(fp == NULL) {
			printf("Error: unable to open tadump.ta for write!\n");
               CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		for (j=1; j<3; ++j) {
			if (!open_ta(j, dev)) {
				printf("Error, unable to open ta partition %d !\n", j);
				if (fp)
					fclose(fp);
				CloseHandle(dev);
				SetupDiDestroyDeviceInfoList(hDevInfo);
				return 1;
			}

			printf("Dumping ta partition: %d...\n", j);
			snprintf(rest, sizeof(rest), "%02X", j&0xff);
			fwrite("//partition:\n", 1, 13, fp);
			fwrite(rest, 1, 2, fp);
			fwrite("\n\n", 1, 2, fp);

			for (i=0; i<0x13000; ++i) {
				unit[0] = (i >> 24) & 0xff;
				unit[1] = (i >> 16) & 0xff;
				unit[2] = (i >> 8) & 0xff;
				unit[3] = i & 0xff;

				if (dump_ta(unit, dev, fp))
					printf("ta unit %02X%02X%02X%02X is read.\n\n", unit[0]&0xff, unit[1]&0xff, unit[2]&0xff, unit[3]&0xff);
				else
					printf("Error reading unit %02X%02X%02X%02X!\n\n", unit[0]&0xff, unit[1]&0xff, unit[2]&0xff, unit[3]&0xff);
			}

			if (!close_ta(j, dev)) {
				if (fp)
					fclose(fp);
				CloseHandle(dev);
				SetupDiDestroyDeviceInfoList(hDevInfo);
				return 1;
			}
		}

		if (fp)
			fclose(fp);
	}

/*////////////////////////// unlock bootloader ////////////////////////*/

	if (strcmp(argv[3], "0") != 0)
	{
		if (strlen(argv[3]) != 16) {
			printf("Error: bootloader unlock key must contain 16 characters aka 8 hex!\n");
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		if (strstr(argv[3], "0x")) {
			printf("Error! Write unlock code without \"0x\"!\n");
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		for (i=0; i<16; ++i)
			argv[3][i] = toupper(argv[3][i]);

		printf("Checking if bootloader unlock key is ok...\n");

		if (!open_ta(2, dev)) {
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		unit_buff = read_ta("\x00\x00\x07\xDA", dev);

		if (unit_buff == NULL) {
			printf("Error: unit 7DA is NULL!\n");
			goto finish_unlocking;
		}

		if (unit_buff_sz < 0x48) {
			printf("Error: unit 7DA < 0x48!\n");
			free(unit_buff);
			goto finish_unlocking;
		}

		for (i=0; i < unit_buff_sz-19; ++i) {
			if (memcmp(unit_buff + i, "ROOTING_ALLOWED=\"1\"", 19) == 0) {
				unlock_allowed = 1;
				break;
			}
		}

		printf("Device unlocking posibility: ");

		if (!unlock_allowed) {
			printf("ROOTING_ALLOWED = 0\n");
			printf("Error: your device can not be unlocked! If you do this another way, your phone will be hard bricked!\n");
			free(unit_buff);
			goto finish_unlocking;
		}

		printf("ROOTING_ALLOWED = 1\n");
		printf("Verifying unlock key: %s...\n", argv[3]);

		unlock_allowed = 0;
		for (i=0; i < unit_buff_sz-0x47; ++i) {
			if (memcmp(unit_buff+i, "RCK_H=\"", 7) == 0) {
				memcpy(rck_h, unit_buff+i+7, 0x40);
				rck_h[64] = '\0';
				unlock_allowed = 1;
				break;
			}
		}

		free(unit_buff);

		if (!unlock_allowed) {
			printf("Error: unable to read RCK_H key from ta unit!\n");
			goto finish_unlocking;
		}

		printf("RCK_H key: %s\n", rck_h);

		to_ascii(temp, argv[3]);

		if (!getSHA256(temp, rck_h)) {
			printf("Error: sha256 validation reported that your unlock key is not ok!\n");
			goto finish_unlocking;
		}

		memset(tmp, '\0', sizeof(tmp));
		memcpy(tmp, "\x00\x00\x08\xB2\x00\x00\x00\x10", 8);
		memcpy(tmp+8, argv[3], 16);
		tmp[24] = '\0';

		if (!write_ta("\x00\x00\x08\xB2", dev, tmp, 24)) {
			printf("Error unlocking bootloader!\n");
			goto finish_unlocking;
		}
		else
		{
			printf("Bootloader is unlocked now.\n\n\n");
			close_ta(2, dev);
			goto continue_unlocking;
		}

finish_unlocking:
		close_ta(2, dev);
		CloseHandle(dev);
		SetupDiDestroyDeviceInfoList(hDevInfo);
		return 1;
	}

continue_unlocking:

/*////////////////////////// flash boot delivery ////////////////////////*/

	if (strcmp(argv[6], "0") != 0)
	{
		int strpo = 0;
		int po1 = 0;
		int po2 = 0;

		printf("Flashing boot delivery...\n");

		if (stat("boot/boot_delivery.xml", &filestat) < 0) {
			printf("Notice: boot_delivery.xml not exist in boot folder, skipping bootdelivery flashing.\n");
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		} else
			printf("Found boot_delivery.xml in boot folder.\n");

		if (!parse_xml("boot/boot_delivery.xml")) {
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		if (!strlen(bootdelivery_version)) {
			printf("Notice: unable to determine boot delivery version, skipping bootdelivery flashing.\n");
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}
		
		printf("Boot delivery version: %s\n", bootdelivery_version);

		printf("Verifying if boot delivery match with device...\n");

		if (!open_ta(2, dev)) {
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		unit_buff = read_ta("\x00\x00\x08\x9F", dev);

		if (unit_buff == NULL) {
			printf("Notice: unit 89F is NULL, skipping bootdelivery flashing.\n");
			close_ta(2, dev);
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		if (!close_ta(2, dev)) {
			free(unit_buff);
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		if (strstr(unit_buff, bootdelivery_version) == NULL) {
			printf("Notice: boot delivery version not match with your device!\nAre you sure it is for your device?\nSkipping bootdelivery flashing.\n");
			free(unit_buff);
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		free(unit_buff);

		printf("Success: boot delivery match.\n");

		if (!open_ta(2, dev)) {
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		unit_buff = read_ta("\x00\x00\x07\xD3", dev);

		if (unit_buff == NULL) {
			printf("Notice: unit 7D3 is NULL, skipping bootdelivery flashing.\n");
			close_ta(2, dev);
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		if (!close_ta(2, dev)) {
			CloseHandle(dev);
			free(unit_buff);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		memset(device_hwconfig, 0, sizeof(device_hwconfig));
		for (strpo=0; strpo < read_ta_howmany-6; ++strpo)
		{
			if (memcmp(unit_buff+strpo, "REV=\"", 5) == 0) {
				po1 = strpo;
				strpo = po1+5;
			}
			if (po1 > 0)
			{
				if (memcmp(unit_buff+strpo, "\"", 1) == 0) {
					po1 += 5;
                         po2 = strpo;
					break;
				}
			}
		}

		if (po2 > 0)
			memcpy(device_hwconfig, unit_buff+po1, po2-po1);

		free(unit_buff);

		printf("Determining OTP data...\n");

		if ((gt = fopen64("btinf", "rb")) == NULL) {
			printf("Unable to open file btinf! Skipping bootdelivery flashing.\n");
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		i = 0;
		while (!feof(gt)) {
			fread_unus_res(bootdelivery_attr+i, 1, 1, gt);
			i += 1;
			if (i > sizeof(bootdelivery_attr))
				break;
		}
		fclose(gt);

		if (bootdelivery_attr[0] != 'O') {
			printf("Unable to determine device attributes! Skipping bootdelivery flashing.\n");
	          CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		if (device_hwconfig[0] == 0) {
			printf("Unable to determine device hwconfig! Skipping bootdelivery flashing.\n");
	          CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}

		printf("DEVICE_ATTRIBUTES: %s\n", bootdelivery_attr);
		printf("HWCONFIG_VERSION: %s\n", device_hwconfig);

		for(i=0; i<td1; ++i)
		{
			if (strstr(bootdelivery_xml[i][1], bootdelivery_attr) != NULL)
			{
				for (j=2; j<13; ++j)
				{
					if (strlen(bootdelivery_xml[i][j]) != 0)
					{
                            	if (j == 2)
						{
							if (strstr(bootdelivery_xml[i][j], device_hwconfig) != NULL)
                                           bootdelivery_found = 1;
						}

						if (bootdelivery_found)
						{
							if (j == 2) printf("\nFound bootdelivery match: %s\n", bootdelivery_xml[i][0]);
							/*if (j == 2) printf("%d: %s\n", j, bootdelivery_xml[i][1]);*/
							if (j == 3)
							{
								char *flash_bdta = (char *)malloc(strlen(bootdelivery_xml[i][j])+5+1);
								if (flash_bdta == NULL) {
									printf("Error malloc flash_bdta! Skipping bootdelivery flashing.\n");
									CloseHandle(dev);
									SetupDiDestroyDeviceInfoList(hDevInfo);
									return 1;
								}
								snprintf(flash_bdta, strlen(bootdelivery_xml[i][j])+5+1, "boot/%s", bootdelivery_xml[i][j]);
								printf("Found bootdelivery hwconfig ta file: %s, flashing it...\n", flash_bdta);
								if (!proced_ta_file(flash_bdta, dev)) {
									printf("Error flashing %s!\n", flash_bdta);
									CloseHandle(dev);
									free(flash_bdta);
									SetupDiDestroyDeviceInfoList(hDevInfo);
									return 1;
								}
								free(flash_bdta);
							}
							if (j > 3)
							{
								char *flash_bdsin = (char *)malloc(strlen(bootdelivery_xml[i][j])+5+1);
								if (flash_bdsin == NULL) {
									printf("Error malloc flash_bdsin!\n");
									CloseHandle(dev);
									SetupDiDestroyDeviceInfoList(hDevInfo);
									return 1;
								}
								snprintf(flash_bdsin, strlen(bootdelivery_xml[i][j])+5+1, "boot/%s", bootdelivery_xml[i][j]);
								printf("Found bootdelivery sin file: %s, flashing it...\n", flash_bdsin);
								if (!write_sin(flash_bdsin, dev)) {
									printf("Error flashing %s!\n", flash_bdsin);
									CloseHandle(dev);
									free(flash_bdsin);
									SetupDiDestroyDeviceInfoList(hDevInfo);
									return 1;
								}
								free(flash_bdsin);
							}
						}
					}
				}
			}
		}

          if (!bootdelivery_found)
			printf("Didn't found bootdelivery that match your device!\n\n\n");
	}

/*////////////////////////// flash .ta ////////////////////////*/

	if (strcmp(argv[7], "0") != 0) {
		if (!proced_ta_file(argv[7], dev)) {
			CloseHandle(dev);
			SetupDiDestroyDeviceInfoList(hDevInfo);
			return 1;
		}
	}

/*/////////////////////////////////////////////////////////////////////*/

	CloseHandle(dev);
	SetupDiDestroyDeviceInfoList(hDevInfo);

	return 0;
}


