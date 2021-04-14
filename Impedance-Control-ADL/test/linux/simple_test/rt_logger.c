/**
 * @file rt_logger.c
 * @brief Create and save log files when the controller crashes or encounters an error.
 *
 * 
 */
#include <stdlib.h>
#include <stdio.h>
#include <sched.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <pthread.h>
#include <time.h>

#include "rt_logger.h"


FILE *fd;
/**
 * @brief Open log file and write configuration information to file
 */
void init_logger()
{
    printf("[Logger] Initializing logger...\n");
    char date_buffer[20];
    snprintf(date_buffer, 20, "%ld.rtlog", time(NULL));
    fd = fopen(date_buffer, "w+");
    fprintf(fd,"Compiled: %s at %s \n", __DATE__, __TIME__);
    fprintf(fd,"C Version: %ld\n", __STDC_VERSION__);
    fprintf(fd,"GCC Version: %s\n", __VERSION__);
    fprintf(fd,"Size of pointer: %ld Size of int: %ld Size of long: %ld \n", sizeof(void*), sizeof(int), sizeof(long));
    time_t program_start = time(NULL);
    char* time_str = ctime(&program_start);
    fprintf(fd,"Started running at %s\n", time_str);
    printf("[Logger] Done!\n");
    //fclose(fd);
}

/**
 * @brief Get the FD of the log file.
 * The log file must be opened first.
 * @return FD of the log file
 */
FILE* get_logger_fd()
{
    return fd;
}


/**
 * @brief Write an error to the log file.  Date and time is automatically added to the message.
 * @param error Error string
 */
void log_error(char* error)
{
    time_t error_time = time(NULL);
    char* time_str = ctime(&error_time);
    fprintf(fd, "%s: %s\n", time_str, error);
    printf("%s\n", error);
}
