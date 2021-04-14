/**
 * @file rt_gpio.h
 * @brief Hardware interface for GPIO pins on the ADL
 *
 * In simulation, this code does nothing.
 *
 */
#ifndef _rt_gpio
#define _rt_gpio

#define DEFAULT_BASEPORT 0xF040

void init_gpio();

void set_gpio(int index, int value);
void update_gpio();
void estop();
void enable_gpio();
int gpio_read(int port, unsigned char *data);
#endif
