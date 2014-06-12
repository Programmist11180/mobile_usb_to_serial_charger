/*
 * charger.c
 * 
 * Program to enable charging for mobile phones with usb-to-serial cables.
 * Some data cables for mobile phones is a usb to serial converters. 
 * Usually this cables needs a special command to enable charge on mobile phone. 
 * This program can enable charge on cable Mobile Action MA-8910P.
 * 
 * Copyright 2014 Programmist11180 <programmer11180@programist.ru>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 * 
 * 
 */
 
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

const char chars[6]={0x55,0x55,0x55,0x55,0x4,0x1};
int tty_fd, ioctl_flags=0;
char *ttydevice;

int main (int argc, char *argv[])
{
  struct termios MA_options;
  if (argc > 1) ttydevice = argv[1];
  else ttydevice = "/dev/ttyUSB0";
  tty_fd = open (ttydevice, O_RDWR | O_NOCTTY | O_NDELAY);
	if (tty_fd == -1) {
	  fprintf (stderr, "Open_port: Unable to open tty_fd %s: %s\n", ttydevice, strerror(errno));
	  return 1;    
	};
	if (tcgetattr(tty_fd, &MA_options)!=0) fprintf(stderr, "tcgetattr failed: %s\n", strerror(errno));
cfsetispeed(&MA_options, B9600);
cfsetospeed(&MA_options, B9600);
	if (tcsetattr(tty_fd, TCSAFLUSH, &MA_options)!= 0) fprintf(stderr, "tcsetattr failed: %s\n", strerror(errno));
	ioctl_flags &= TIOCM_RTS;
	ioctl_flags &= TIOCM_DTR;
	if (ioctl(tty_fd, TIOCMSET, &ioctl_flags)!=0) fprintf(stderr, "ioctl failed: %s\n", strerror(errno));
MA_options.c_cflag &= ~PARENB;
MA_options.c_cflag &= ~CSTOPB;
MA_options.c_cflag &= ~CSIZE;
MA_options.c_cflag |= CS8;
MA_options.c_cflag |=  (CLOCAL | CREAD);
  MA_options.c_cc[VEOF] = 0;
  MA_options.c_cc[VINTR] = 0;
  MA_options.c_cc[VERASE] = 0;
  MA_options.c_cc[VSTART] = 0;
  MA_options.c_cc[VSTOP] = 0;
  MA_options.c_cc[VMIN] = 0;
  MA_options.c_cc[VTIME] = 10;
  MA_options.c_cc[VSTART] = 0x11;
  MA_options.c_cc[VSTOP] = 0x13;
if (tcsetattr(tty_fd, TCSANOW, &MA_options)!= 0) fprintf(stderr, "tcsetattr failed: %s.\n", strerror(errno));
if (tcflush(tty_fd, TCIOFLUSH)!=0) fprintf(stderr, "tcflush failed: %s.\n", strerror(errno));
if (tcgetattr(tty_fd, &MA_options)!=0) fprintf(stderr, "tcgetattr failed: %s\n", strerror(errno));
cfsetispeed(&MA_options, B9600);
cfsetospeed(&MA_options, B9600);
	if (tcsetattr(tty_fd, TCSAFLUSH, &MA_options)!= 0) fprintf(stderr, "tcsetattr failed: %s\n", strerror(errno));
	ioctl_flags &= TIOCM_RTS;
	ioctl_flags &= ~TIOCM_DTR;
	if (ioctl(tty_fd, TIOCMSET, &ioctl_flags)!=0) fprintf(stderr, "ioctl failed: %s\n", strerror(errno));
	MA_options.c_cflag &= ~PARENB;
MA_options.c_cflag &= ~CSTOPB;
MA_options.c_cflag &= ~CSIZE;
MA_options.c_cflag |= CS8;
MA_options.c_cflag |=  (CLOCAL | CREAD);
  MA_options.c_cc[VEOF] = 0;
  MA_options.c_cc[VINTR] = 0;
  MA_options.c_cc[VERASE] = 0;
  MA_options.c_cc[VSTART] = 0;
  MA_options.c_cc[VSTOP] = 0;
  MA_options.c_cc[VMIN] = 0;
  MA_options.c_cc[VTIME] = 10;
  MA_options.c_cc[VSTART] = 0x11;
  MA_options.c_cc[VSTOP] = 0x13;
if (tcsetattr(tty_fd, TCSANOW, &MA_options)!= 0) fprintf(stderr, "tcsetattr failed: %s.\n", strerror(errno));
write(tty_fd, chars, 6);
if (tcflush(tty_fd, TCIOFLUSH)!=0) fprintf(stderr, "tcflush failed: %s.\n", strerror(errno));
if (close(tty_fd)!= 0) fprintf(stderr, "close(tty_fd) failed: %s.\n", strerror(errno));
  return 0;
}
