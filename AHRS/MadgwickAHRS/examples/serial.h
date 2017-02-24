#ifndef __SERIAL_H__
#define __SERIAL_H__

int open_dev(char *dev);

void set_speed(int fd, int speed);

int set_parity(int fd,int databits,int stopbits,int parity);


#endif
