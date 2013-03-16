/**    IAP : internal Flash memory access library
 *
 *        The internal Flash memory access is described in the LPC1768 usermanual. 
 *            http://www.nxp.com/documents/user_manual/UM10360.pdf
 *
 *            Chapter  2: "LPC17xx Memory map"
 *            Chapter 32: "LPC17xx Flash memory interface and programming"
 *                refering Rev. 01 - 4 January 2010
 *
 *        Released under the MIT License: http://mbed.org/license/mit
 *
 *        revision 1.0  09-Mar-2010   1st release
 *        revision 1.1  12-Mar-2010   chaged: to make possible to reserve flash area for user
 *                                            it can be set by USER_FLASH_AREA_START and USER_FLASH_AREA_SIZE in IAP.h
 *
 *     by Tedd OKANO http://mbed.org/users/okano/notebook/iap-in-application-programming-internal-flash-eras/
 */

#ifndef        MBED_IAP
#define        MBED_IAP

#include    "mbed.h"

#define     USER_FLASH_AREA_START   FLASH_SECTOR_22
#define     USER_FLASH_AREA_SIZE    (FLASH_SECTOR_SIZE_16_TO_29 * 8)

/*
 *  memory map information is available in next URL also.
 *    http://mbed.org/projects/libraries/svn/mbed/trunk/LPC1768/LPC17xx.h
 */
 
/**    Table for start adress of sectors
 *    
 *        LPC1768 internal flash memory sector numbers and addresses
 *
 *        LPC1768 flash memory are and sector number/size
 *        Table 568 "Sectors in a LPC17xx device", Section 5. "Sector numbers", usermanual
 *
 *        0x00000000 - 0x0007FFFF        flash (29 sectors)
 *
 *      Sector0:     0x00000000 - 0x00000FFF        4K
 *      Sector1:     0x00001000 - 0x00001FFF        4K
 *      Sector2:     0x00002000 - 0x00002FFF        4K
 *      Sector3:     0x00003000 - 0x00003FFF        4K
 *      Sector4:     0x00004000 - 0x00004FFF        4K
 *      Sector5:     0x00005000 - 0x00005FFF        4K
 *      Sector6:     0x00006000 - 0x00006FFF        4K
 *      Sector7:     0x00007000 - 0x00007FFF        4K
 *      Sector8:     0x00008000 - 0x00008FFF        4K
 *      Sector9:     0x00009000 - 0x00009FFF        4K
 *      Sector10:    0x0000A000 - 0x0000AFFF        4K
 *      Sector11:    0x0000B000 - 0x0000BFFF        4K
 *      Sector12:    0x0000C000 - 0x0000CFFF        4K
 *      Sector13:    0x0000D000 - 0x0000DFFF        4K
 *      Sector14:    0x0000E000 - 0x0000EFFF        4K
 *      Sector15:    0x0000F000 - 0x0000FFFF        4K
 *
 *      Sector16:    0x00010000 - 0x00017FFF        32K
 *      Sector17:    0x00018000 - 0x0001FFFF        32K
 *      Sector18:    0x00020000 - 0x00027FFF        32K
 *      Sector19:    0x00028000 - 0x0002FFFF        32K
 *      Sector20:    0x00030000 - 0x00037FFF        32K
 *      Sector21:    0x00038000 - 0x0003FFFF        32K
 *      Sector22:    0x00040000 - 0x00047FFF        32K
 *      Sector23:    0x00048000 - 0x0004FFFF        32K
 *      Sector24:    0x00050000 - 0x00057FFF        32K
 *      Sector25:    0x00058000 - 0x0005FFFF        32K
 *      Sector26:    0x00060000 - 0x00067FFF        32K
 *      Sector27:    0x00068000 - 0x0006FFFF        32K
 *      Sector28:    0x00070000 - 0x00077FFF        32K
 *      Sector29:    0x00078000 - 0x0007FFFF        32K
 */
 
#define     FLASH_SECTOR_0       0x00000000
#define     FLASH_SECTOR_1       0x00001000
#define     FLASH_SECTOR_2       0x00002000
#define     FLASH_SECTOR_3       0x00003000
#define     FLASH_SECTOR_4       0x00004000
#define     FLASH_SECTOR_5       0x00005000
#define     FLASH_SECTOR_6       0x00006000
#define     FLASH_SECTOR_7       0x00007000
#define     FLASH_SECTOR_8       0x00008000
#define     FLASH_SECTOR_9       0x00009000
#define     FLASH_SECTOR_10      0x0000A000
#define     FLASH_SECTOR_11      0x0000B000
#define     FLASH_SECTOR_12      0x0000C000
#define     FLASH_SECTOR_13      0x0000D000
#define     FLASH_SECTOR_14      0x0000E000
#define     FLASH_SECTOR_15      0x0000F000
#define     FLASH_SECTOR_16      0x00010000
#define     FLASH_SECTOR_17      0x00018000
#define     FLASH_SECTOR_18      0x00020000
#define     FLASH_SECTOR_19      0x00028000
#define     FLASH_SECTOR_20      0x00030000
#define     FLASH_SECTOR_21      0x00038000
#define     FLASH_SECTOR_22      0x00040000
#define     FLASH_SECTOR_23      0x00048000
#define     FLASH_SECTOR_24      0x00050000
#define     FLASH_SECTOR_25      0x00058000
#define     FLASH_SECTOR_26      0x00060000
#define     FLASH_SECTOR_27      0x00068000
#define     FLASH_SECTOR_28      0x00070000
#define     FLASH_SECTOR_29      0x00078000

#define     FLASH_SECTOR_SIZE_0_TO_15    ( 4 * 1024)
#define     FLASH_SECTOR_SIZE_16_TO_29   (32 * 1024)

static char * sector_start_adress[]    = {
    (char *)FLASH_SECTOR_0,
    (char *)FLASH_SECTOR_1,
    (char *)FLASH_SECTOR_2,
    (char *)FLASH_SECTOR_3,
    (char *)FLASH_SECTOR_4,
    (char *)FLASH_SECTOR_5,
    (char *)FLASH_SECTOR_6,
    (char *)FLASH_SECTOR_7,
    (char *)FLASH_SECTOR_8,
    (char *)FLASH_SECTOR_9,
    (char *)FLASH_SECTOR_10,
    (char *)FLASH_SECTOR_11,
    (char *)FLASH_SECTOR_12,
    (char *)FLASH_SECTOR_13,
    (char *)FLASH_SECTOR_14,
    (char *)FLASH_SECTOR_15,
    (char *)FLASH_SECTOR_16,
    (char *)FLASH_SECTOR_17,
    (char *)FLASH_SECTOR_18,
    (char *)FLASH_SECTOR_19,
    (char *)FLASH_SECTOR_20,
    (char *)FLASH_SECTOR_21,
    (char *)FLASH_SECTOR_22,
    (char *)FLASH_SECTOR_23,
    (char *)FLASH_SECTOR_24,
    (char *)FLASH_SECTOR_25,
    (char *)FLASH_SECTOR_26,
    (char *)FLASH_SECTOR_27,
    (char *)FLASH_SECTOR_28,
    (char *)FLASH_SECTOR_29    
};


/**    Error code by IAP routine
 *  
 *        Table 588 "ISP Return Codes Summary", Section 7.15 "ISP Return Codes", usermanual
 */

enum error_code
    {
            CMD_SUCCESS,
            INVALID_COMMAND,
            SRC_ADDR_ERROR,
            DST_ADDR_ERROR,
            SRC_ADDR_NOT_MAPPED,
            DST_ADDR_NOT_MAPPED,
            COUNT_ERROR,
            INVALID_SECTOR,
            SECTOR_NOT_BLANK,
            SECTOR_NOT_PREPARED_FOR_WRITE_OPERATION,
            COMPARE_ERROR,
            BUSY,
            PARAM_ERROR,
            ADDR_ERROR,
            ADDR_NOT_MAPPED,
            CMD_LOCKED,
            INVALID_CODE,
            INVALID_BAUD_RATE,
            INVALID_STOP_BIT,
            CODE_READ_PROTECTION_ENABLED
    };



/*
 *  IAP routine entry
 *
 *        Chapter 8. "IAP commands", usermanual
 */

#define     IAP_LOCATION    0x1fff1ff1
typedef     void (*IAP_call)(unsigned int [], unsigned int []);

/**    IAP class
 *  
 *        Interface for internal flash memory access
 */


class IAP {
public:

    /*
     *  SystemCoreClock ??? :  
     *    http://mbed.org/forum/mbed/topic/229/
     *    http://mbed.org/users/simon/programs/SystemCoreClock/16mhsh/
     */

    
    /**    Constructor for IAP
     *
     */

    IAP() : iap_entry( reinterpret_cast<IAP_call>(IAP_LOCATION) ), cclk_kHz( SystemCoreClock / 1000 ) {}
    int read_ID( void );
    int read_serial( void );
    int blank_check( int start, int end );
    int erase( int start, int end );
    int prepare( int start, int end );
    int write( char *source_addr, char *target_addr, int size );
    int compare( char *source_addr, char *target_addr, int size );
    int reinvoke_isp( void );
    
    char *reserved_flash_area_start( void );
    int   reserved_flash_area_size( void );

private:
    IAP_call        iap_entry;
    unsigned int    IAP_command[ 5 ];
    unsigned int    IAP_result[ 5 ];
    int             cclk_kHz;
    
    //int cpu_clock( void );
}
;

#endif    //  #ifndef  MBED_IAP
