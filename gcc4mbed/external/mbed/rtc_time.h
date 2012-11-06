/* Title: time
 * Implementation of the C time.h functions
 *
 * Provides mechanisms to set and read the current time, based
 * on the microcontroller Real-Time Clock (RTC), plus some 
 * standard C manipulation and formating functions. 
 *
 * Example:
 * > #include "mbed.h"
 * >
 * > int main() {
 * >     set_time(1256729737);  // Set RTC time to Wed, 28 Oct 2009 11:35:37
 * >      
 * >     while(1) {    
 * >         time_t seconds = time(NULL);
 * >         
 * >         printf("Time as seconds since January 1, 1970 = %d\n", seconds);
 * >  
 * >         printf("Time as a basic string = %s", ctime(&seconds));
 * >
 * >         char buffer[32];
 * >         strftime(buffer, 32, "%I:%M %p\n", localtime(&seconds));
 * >         printf("Time as a custom formatted string = %s", buffer);
 * >    
 * >         wait(1);
 * >     }
 * > }
 */
 
/* mbed Microcontroller Library - rtc_time
 * Copyright (c) 2009 ARM Limited. All rights reserved.
 */

#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif

#if 0 // for documentation only
/* Function: time
 * Get the current time
 *
 * Returns the current timestamp as the number of seconds since January 1, 1970
 * (the UNIX timestamp). The value is based on the current value of the 
 * microcontroller Real-Time Clock (RTC), which can be set using <set_time>.
 *
 * Example:
 * > #include "mbed.h"
 * >
 * > int main() {
 * >     time_t seconds = time(NULL);
 * >     printf("It is %d seconds since January 1, 1970\n", seconds);
 * > }
 *
 * Variables:
 *  t - Pointer to a time_t to be set, or NULL if not used
 *  returns - Number of seconds since January 1, 1970 (the UNIX timestamp)
 */
time_t time(time_t *t);
#endif

/* Function: set_time
 * Set the current time
 *
 * Initialises and sets the time of the microcontroller Real-Time Clock (RTC)
 * to the time represented by the number of seconds since January 1, 1970 
 * (the UNIX timestamp). 
 * 
 * Example:
 * > #include "mbed.h"
 * >
 * > int main() {
 * >     set_time(1256729737); // Set time to Wed, 28 Oct 2009 11:35:37
 * > }
 *
 * Variables:
 *  t - Number of seconds since January 1, 1970 (the UNIX timestamp) 
 */ 
void set_time(time_t t);

#if 0 // for documentation only
/* Function: mktime
 * Converts a tm structure in to a timestamp
 *  
 * Converts the tm structure in to a timestamp in seconds since January 1, 1970
 * (the UNIX timestamp). The values of tm_wday and tm_yday of the tm structure 
 * are also updated to their appropriate values.
 *
 * Example:
 * > #include "mbed.h"
 * >
 * > int main() {
 * >     // setup time structure for Wed, 28 Oct 2009 11:35:37
 * >     struct tm t;
 * >     t.tm_sec = 37;    // 0-59
 * >     t.tm_min = 35;    // 0-59
 * >     t.tm_hour = 11;   // 0-23
 * >     t.tm_mday = 28;   // 1-31
 * >     t.tm_mon = 9;     // 0-11
 * >     t.tm_year = 109;  // year since 1900
 * > 
 * >     // convert to timestamp and display (1256729737)
 * >     time_t seconds = mktime(&t);
 * >     printf("Time as seconds since January 1, 1970 = %d\n", seconds);
 * > }
 * 
 * Variables:
 *  t - The tm structure to convert
 *  returns - The converted timestamp
 */
time_t mktime(struct tm *t);
#endif

#if 0 // for documentation only
/* Function: localtime
 * Converts a timestamp in to a tm structure
 *  
 * Converts the timestamp pointed to by t to a (statically allocated) 
 * tm structure. 
 *
 * Example:
 * > #include "mbed.h"
 * >
 * > int main() {
 * >     time_t seconds = 1256729737;
 * >     struct tm *t = localtime(&seconds);
 * > }
 * 
 * Variables:
 *  t - Pointer to the timestamp
 *  returns - Pointer to the (statically allocated) tm structure
 */
struct tm *localtime(const time_t *t);
#endif

#if 0 // for documentation only
/* Function: ctime
 * Converts a timestamp to a human-readable string
 *  
 * Converts a time_t timestamp in seconds since January 1, 1970 (the UNIX
 * timestamp) to a human readable string format. The result is of the 
 * format: "Wed Oct 28 11:35:37 2009\n"
 *
 * Example:
 * > #include "mbed.h"
 * >
 * > int main() {
 * >     time_t seconds = time(NULL);
 * >     printf("Time as a string = %s", ctime(&seconds));
 * > }
 * 
 * Variables:
 *  t - The timestamp to convert
 *  returns - Pointer to a (statically allocated) string containing the
 *            human readable representation, including a '\n' character
 */
char *ctime(const time_t *t);
#endif
 
#if 0 // for documentation only
/* Function: strftime
 * Converts a tm structure to a custom format human-readable string
 *  
 * Creates a formated string from a tm structure, based on a string format 
 * specifier provided.
 *
 * Format Specifiers: 
 *  %S - Second (00-59)
 *  %M - Minute (00-59)
 *  %H - Hour (00-23)
 *  %d - Day (01-31)
 *  %m - Month (01-12)
 *  %Y/%y - Year (2009/09)
 *
 *  %A/%a - Weekday Name (Monday/Mon)
 *  %B/%b - Month Name (January/Jan)
 *  %I - 12 Hour Format (01-12)
 *  %p - "AM" or "PM"
 *  %X - Time (14:55:02)
 *  %x - Date (08/23/01)
 * 
 * Example:
 * > #include "mbed.h"
 * >
 * > int main() {
 * >     time_t seconds = time(NULL);
 * >  
 * >     char buffer[32];
 * >     strftime(buffer, 32, "%I:%M %p\n", localtime(&seconds));
 * >     printf("Time as a formatted string = %s", buffer);
 * > }   
 * 
 * Variables:
 *  buffer - String buffer to store the result
 *  max - Maximum number of characters to store in the buffer
 *  format - Format specifier string
 *  t - Pointer to the tm structure to convert
 *  returns - Number of characters copied
 */
size_t strftime(char *buffer, size_t max, const char *format, const struct tm *t);
#endif

#ifdef __cplusplus
}
#endif 
