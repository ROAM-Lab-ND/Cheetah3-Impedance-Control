/**
 * @brief Logging utilities
 *
 */
#ifndef _rt_logger
#define _rt_logger
#include <stdio.h>

void init_logger();

FILE* get_logger_fd();
void log_error(char* error);
#endif
