/****************************************************************************************
*****************************************************************************************
***                                                                                   ***
***                                 Copyright (c) 2013                                ***
***                                                                                   ***
***                                Conexant Systems, Inc.                             ***
***                                                                                   ***
***                                 All Rights Reserved                               ***
***                                                                                   ***
***                                    CONFIDENTIAL                                   ***
***                                                                                   ***
***               NO DISSEMINATION OR USE WITHOUT PRIOR WRITTEN PERMISSION            ***
***                                                                                   ***
*****************************************************************************************
**
**  File Name:
**      cxflash.c
**
**  Abstract:
**      This code is to download the firmware to HUDSON device via I2C bus.
**
**
**  Product Name:
**      Conexant Hudson
**
**  Remark:
**
**  Version: 3.12.0.0.
**
*****************************************************************************************/
#include <linux/init.h>
#include <linux/module.h>
#include "cxflash.h"
/* FIX me*/
#ifndef __BYTE_ORDER
#define __BYTE_ORDER  __ITTLE_ENDIAN
#define __LITTLE_ENDIAN 1234
#define __ITTLE_ENDIAN 1234
#endif
#define ENABLE_I2C_BURST_MODE


enum I2C_STATE{I2C_OK,I2C_ERR,I2C_RETRY,I2C_ERASE_CHIPS} ;

#if defined(_WINDOWS )
#define DBG_ERROR
#define DBG_INFO
#define LOG( _msg_ ) /*printk _msg_*/
#if !defined(DUMP_I2C)
//void ShowProgress(int curPos,int bForceRedraw,int eState,const int MaxPos);
//void InitShowProgress(const int MaxPos);
#define InitShowProgress(MaxPos)
#define ShowProgress(curPos,  bForceRedraw, eState,MaxPos)
#else
#define InitShowProgress(MaxPos)  printk( "Downloading ... %2d%%", 0); fflush(stdout);
#define ShowProgress(curPos,  bForceRedraw, eState,MaxPos) printk( "\b\b\b%2d%%", (char) (((curPos) *100)/MaxPos) ); fflush(stdout);
#endif
#else
#define DBG_ERROR
#define DBG_INFO
#define LOG( _msg_ )
#define InitShowProgress(MaxPos)
#define ShowProgress(curPos,  bForceRedraw, eState,MaxPos)
#endif

#define BIBF(_x_) if(!(_x_)) break;
#define BIF(_x_) {err_no=_x_; if(err_no) break;}

#ifndef BIBF
#define BIBF( _x_ ) if(!(_x_)) break;
#endif


#define SFS_MAGIC(a,b,c,d)	((u32)(((a)<<(0*8))|((b)<<(1*8))|((c)<<(2*8))|((d)<<(3*8))))
#define SFS_MAGIC_HEADER	SFS_MAGIC('S','F','S','E')
#define SFS_MAGIC_BOOT		SFS_MAGIC('S','F','S','L')
#define SFS_MAGIC_BOOT_H	SFS_MAGIC('S','F','S','H')
#define SFS_MAGIC_END		SFS_MAGIC('S','F','S','O')
#define SFS_MAGIC_HDRECP	SFS_MAGIC('S','F','S','M')	// ... 0x4D -> ... 0100 1101 [6] ->
#define SFS_MAGIC_HDRPAD	SFS_MAGIC('S','F','S','K')	// ... 0x4B -> ... 0100 1011 [5] ->
#define SFS_MAGIC_HDRDEL	SFS_MAGIC('S','F','S','A')	// ... 0x41 -> ... 0100 0001 [0]
#define SFS_BLKSIZE 4096
#define BLKNONE (blknr)-1
#define FIRST_FREE_BLK	2

typedef unsigned short blknr;
#undef offsetof
#define offsetof(s,m)	((uint)(int)&(((s *)0)->m))

#define SFS_HEADER_DEF_POS	0
#define SFS_HEADER_ALT_POS	1
#define SFS_FIRST_FREE_BLK	2
#define SFS_SECOND_FREE_BLK	3


#if defined(_WINDOWS)
#pragma pack(push,1)
#endif

struct sfs_end_s {
  u32 magic;			// ID of a sfs end record
} ;

struct sfs_padded_s {
  u32 magic;			// ID for the sfs file record
  blknr	asize;			// allocated size of memory in blocks
} ;

struct sfs_encapsulated_s {
  u32 magic;			// ID for the sfs file record
  blknr	asize;			// allocated size of memory in blocks
} ;

enum i2c_cmd_e
  {
  I2C_PING,			//  ping
  I2C_RSTAT,			//  get status
  I2C_WSTAT,			//  write status
  I2C_ERASES,			//  erase 4K sector
  I2C_ERASEC,			//  erase chip
  I2C_READ,			//  read from a 4K block, from offset 0
  I2C_WRITE_NORMAL,		//  write to a 4K block, from offset 0
  I2C_WRITE_VERIFY,		//  write to a 4K block, from offset 0, verify after write
  I2C_PROTECT,			//  change sw protection (on or off)
  I2C_RTUNNEL,			//  send a SPI read command
  I2C_WTUNNEL,			//  send a SPI write command
  I2C_VERIFY_IMG,		// 11 verify image CRC
  } ;

struct i2c_message_s
  {
  int   cmd:6;			// the command
  uint  err:1;			// error in message
  uint  repl:1;			// done - command completed
  uint  blk:8;			// block nr
  uint  len:16;			// total length of mesage (in u8s)
  u32 crc;			// crc over message
  } ;				//

#define I2C_PAYLOAD_SIZE	4096
#define I2C_PAYLOAD_SIZE_B	(I2C_PAYLOAD_SIZE/sizeof(u32))
#define SFS_LOG2BLKSIZ		12			//
#define SFS_BLOCKSIZE		4096			// size of a SFS block (min allocation unit)
#define SFS_BLK2POS(n)		((n)<<SFS_LOG2BLKSIZ)	//

struct i2c_ping_cmd_s		{ struct i2c_message_s hdr; } ;
struct i2c_ping_rpl_s		{ struct i2c_message_s hdr; u32 id; } ;
struct i2c_rstat_cmd_s		{ struct i2c_message_s hdr; } ;
struct i2c_rstat_rpl_s		{ struct i2c_message_s hdr; u32 status; } ;
struct i2c_wstat_cmd_s		{ struct i2c_message_s hdr; u32 status; } ;
struct i2c_wstat_rpl_s		{ struct i2c_message_s hdr; u32 status; } ;
struct i2c_erases_cmd_s		{ struct i2c_message_s hdr; u16  sector; u16 padding; } ;
struct i2c_erases_rpl_s		{ struct i2c_message_s hdr; u32 status; } ;
struct i2c_erasec_cmd_s		{ struct i2c_message_s hdr; } ;
struct i2c_erasec_rpl_s		{ struct i2c_message_s hdr; u32 status; } ;
struct i2c_read_cmd_s		{ struct i2c_message_s hdr; u16  sector; u16 length; } ;
///////////////////////////////////////////////////////////////////////////////////////
//In order to redcued the memory usage, we splite the large structures into header and payload.
//struct i2c_read_rpl_s		{ struct i2c_message_s hdr; u16  sector; u16 length; u32 data[I2C_PAYLOAD_SIZE_B]; };
struct i2c_read_rpl_h_s		{ struct i2c_message_s hdr; u16  sector; u16 length;} ;
struct i2c_read_rpl_d_s		{ u32 data[I2C_PAYLOAD_SIZE_B]; } ;
//struct i2c_write_cmd_s		{ struct i2c_message_s hdr; u16  sector; u16 length; u32 data[I2C_PAYLOAD_SIZE_B]; };
struct i2c_write_cmd_h_s		{ struct i2c_message_s hdr; u16  sector; u16 length; } ;
struct i2c_write_cmd_d_s		{  u32 data[I2C_PAYLOAD_SIZE_B]; } ;
struct i2c_write_rpl_s		{ struct i2c_message_s hdr; u32 status; } ;
struct i2c_protect_cmd_s	{ struct i2c_message_s hdr; u32 state;  } ;
struct i2c_protect_rpl_s	{ struct i2c_message_s hdr; u32 state;  } ;
struct i2c_rtunnel_cmd_s	{ struct i2c_message_s hdr; u32 cmd; u16 clen; u16 rlen; } ;
//struct i2c_rtunnel_rpl_s	{ struct i2c_message_s hdr; u32 cmd; u16 clen; u16 rlen; u32 data[I2C_PAYLOAD_SIZE_B]; };
struct i2c_rtunnel_rpl_h_s	{ struct i2c_message_s hdr; u32 cmd; u16 clen; u16 rlen; } ;
struct i2c_rtunnel_rpl_d_s	{ u32 data[I2C_PAYLOAD_SIZE_B]; } ;
//struct i2c_wtunnel_cmd_s	{ struct i2c_message_s hdr; u32 cmd; u16 clen; u16 wlen; u32 data[I2C_PAYLOAD_SIZE_B]; };
struct i2c_wtunnel_cmd_h_s	{ struct i2c_message_s hdr; u32 cmd; u16 clen; u16 wlen; } ;
struct i2c_wtunnel_cmd_d_s	{ u32 data[I2C_PAYLOAD_SIZE_B]; } ;
struct i2c_wtunnel_rpl_s	{ struct i2c_message_s hdr; } ;
struct i2c_verify_img_cmd_s	{ struct i2c_message_s hdr; u16 header; u16 first; u16 last; u16 padding; u32 magic; } ;
struct i2c_verify_img_rpl_s	{ struct i2c_message_s hdr; u32 status; } ;

struct i2c_xfer_s
{
  int                 xfer_flag;
  unsigned int        block_index;
  unsigned int        check_sum;
} ;

struct i2c_xfer_reno_s
{
  struct i2c_xfer_s   xfer;
  unsigned int        num_u32; // unit is u32.
} ;

// reno
struct sfs_header_s
  {
  u32 magic;			// 0000 ID of a sfs header
  blknr asize;			// 0004 allocated size of this chunk, should be 1
  blknr sfssize;		// 0006 maximum size of the sfs file system
  blknr nxthdr;			// 0008 should be 1 or -1
   u16  flags;			// 000A
#define SFS_NO_USB_BOOT	(1<<0)	//  1=do not boot over USB (default)
#define SFS_PIMARY_BOOT	(1<<1)	//  1=primary image is image[0], 0=primary image is image[1]
  blknr image[2];		// 000C location of the boot images, primary and secondary
  blknr uflash;			// 0010 location of the uflash boot loader (if any)
  blknr iflash;			// 0012 location of the iflash boot loader (if any)
  blknr xflash[4];		// 0014 location of the aux boot codes
  u32 version;		// 001c version of sfs
  u32 crc_seed;		// 0020 crc seed
  u16  usb_rst_timeout;	// 0024 timeout (in ms) until USB reset must be received
  u16  usb_dfu_timeout;	// 0026 timeout (in ms) until 1st DFU packet must be received
  u32 _reserved1;		// 002A
  blknr sfsbeg;			// 002C start of sfs data area
  blknr sfsend;			// 002E end of of sfs data area
} ;


struct sfs_header_v300_s
  {
  u32 magic;			// 0000 ID of a sfs header
  blknr asize;			// 0004 allocated size of this chunk, should be 1
  blknr sfssize;		// 0006 maximum size of the sfs file system
  blknr nxthdr;			// 0008 should be 1 or -1
  blknr boot;			// 000A location of the boot loader
  blknr image;			// 000C location of the boot images (if any)
  u16  _reserved_w1;	// 000E
  u32 version;		// 0010 version of sfs
  u32 crc_seed;		// 0014 crc seed
  blknr uflash;			// 0018 location of the uflash boot loader (if any)
  blknr iflash;			// 001a location of the iflash boot loader (if any)
  u32 _reserved_d2;	// 001c
  u32 _reserved1;		// 0020
  blknr sfsbeg;			// 002C start of sfs data area
  blknr sfsend;			// 002E end of of sfs data area
  u32 descr[4];		// 0030 description (8-bit ASCII)
} ;


union sfs_record_u
  {
  struct sfs_header_s  hdr;
  u32		       d[SFS_BLKSIZE/4];
  u8		       b[SFS_BLKSIZE];
  } ;


#if defined(_WINDOWS)
#pragma pack(pop)
#endif

#define INIT_I2CS_MESSAGE 0x000080

#undef ERROR
#undef ERROR_TIMEOUT
#undef ERROR_NOT_SUPPORTED

enum {
  ERROR_BLOCK_NR=-1,
  ERROR_CHECKSUM=-2,
  ERROR_LENGTH=-3,
  ERROR_TIMEOUT=-4,
  ERROR_NOT_SUPPORTED=-5,
  ERROR_ERASE=-6,
  ERROR_WRITE=-7,
  ERROR_VERIFY=-8,
  ERROR_INVALID_CMD=-9,
  ERROR_CHIP_NSUPP=-10,

  ERROR_I2C_INTERNAL=-20,
  };


/*
set S  [binary format i 83]; # device->host: standing by for next packet
set T  [binary format i 84]; # device->host: bad packet. retry.
set F  [binary format i 70]; # device->host: fatal error

set R  [binary format i 82]; # host->device: packet transfer complete
set D  [binary format i 68]; # host->device: file transfer complete
set A  [binary format i 65]; # host->device: aborting transfer
set C  [binary format i 67]; # host->device: packet transfer complete
*/


#define    TRANSFER_STATE_COMPLETE        0x43u  //host->device: packet transfer complete. For bootloader.
#define    TRANSFER_STATE_ABORT_TRANSFER  0x41u  // # host->device: aborting transfer
#define    TRANSFER_STATE_FILE_DONE       0x44u  //# host->device: file transfer complete
#define    TRANSFER_STATE_PACKET_READY    0x52u  //# host->device: packet transfer complete
#define    TRANSFER_STATE_FATAL           0x46u  //# device->host: fatal error
#define    TRANSFER_STATE_BAD_PACKET      0x54u  //# device->host: bad packet. retry
#define    TRANSFER_STATE_STANDBY         0x53u  //#  device->host: standing by for next packet.
#define    TRANSFER_STATE_RENO_LOADER_READY  0X80u
#define    TRANSFER_STATE_TRANSFER_COMPLETE  0x4fu  //#  device->host: Trnasfer complete, report CRC





#define TIME_OUT_MS          100   //50 ms
#define MANUAL_TIME_OUT_MS   10000  //1000 ms
#define POLLING_INTERVAL_MS  1     // 1 ms
#define RESET_INTERVAL_MS    200    //200 ms
#define IMG_BLOCK_SIZE       2048
#define IMG_BLOCK_SIZE_RENO  4096
#define MAX_RETRIES          3
#define MAX_I2C_SUB_ADDR_SIZE 4
#define ALIGNMENT_SIZE       (sizeof(long))
#define MAX_WRITE_BUFFER     (sizeof(struct i2c_verify_img_cmd_s))
#define MAX_READ_BUFFER      (sizeof(struct i2c_rtunnel_rpl_h_s) )
#define BUFFER_SIZE          (ALIGNMENT_SIZE+MAX_WRITE_BUFFER+ MAX_READ_BUFFER )
#define ALIG_SIZE(_x_)       ((sizeof(_x_) + ALIGNMENT_SIZE-1)/ALIGNMENT_SIZE)
#define MAX_SMALL_WRITE      0x10
#define _SFS_VERSION(a,b,c,d)	((u32)((a)<<24)|((b)<<16)|((c)<<8)|((d)<<0))


int (*g_I2cReadMemPtr) (void *, u8, u32, u32, u8*);
int (*g_I2cWriteMemPtr) (void *, u8, u32, u32, u8*);
int (*g_SetResetPinPtr) (void*, int);



unsigned char *      g_pBuffer                  = NULL;
unsigned char *      g_pWrBuffer                = NULL;
unsigned char *      g_pRdBuffer                = NULL;

unsigned int        g_cbMaxI2cWrite            = 0;
unsigned int        g_cbMaxI2cRead             = 0;
void *               g_pContextI2cWrite         = NULL;
void *               g_pContextI2cWriteThenRead = NULL;
void *               g_pContextSetResetpin      = NULL;
unsigned char        g_bChipAddress             = 0x00;                   /*Specify the i2c chip address*/
unsigned char        g_bAddressOffset           = 2;
int                  g_bIsRenoDev               = 0;
uint                 g_i2c_blknr                = 0;;
unsigned int         g_bEraseChip               = 0;
int                  g_is_partial_img           = 0;
u16                 g_partial_offset           = 0;
int                  g_is_dual_img              = 0;
u32             g_firmware_version[4]      = {(u32)-1};
SFS_UPDATE_PARTITION g_update_mode              = SFS_UPDATE_BOTH;
u32			 g_sfs_version			    = 0;

const char* const update_mode_str[]=
{
  "Auto",  "0",  "1",  "Both"
};

// supported SPI memory.
#define ID_S25FL032P		0x00150201
#define ID_AT25DF041A		0x0001441F
#define ID_AT25DQ161		0x0000861F
#define ID_AT25DQ321A		0x0000871F
#define ID_AT25DF641		0x0000481F
#define ID_AT25DF321A		0x0001471F
#define ID_GD25Q80B         0x001440C8
#define ID_GD25Q32B         0x001640C8
#define ID_MX25L4006E		0x001320C2
#define ID_MX25L3206E		0x001620C2
#define ID_MX25L6406E		0x001720C2


struct id_2_name{
  uint   id;
  char   name[20];
};

#define ID_NAME(_x_) { ID_##_x_,#_x_}
const struct id_2_name g_spiid_2_name_tlb[]=
{
  ID_NAME(S25FL032P),
  ID_NAME(AT25DF041A),
  ID_NAME(AT25DQ161),
  ID_NAME(AT25DQ321A),
  ID_NAME(AT25DF641),
  ID_NAME(AT25DF321A),
  ID_NAME(GD25Q80B),
  ID_NAME(GD25Q32B),
  ID_NAME(MX25L4006E),
  ID_NAME(MX25L3206E),
  ID_NAME(MX25L6406E),
};


/*
 * The SetupI2cWriteMemCallback sets the I2cWriteMem callback function.
 *
 * PARAMETERS
 *
 *    pCallbackContext [in] - A pointer to a caller-defined structure of data items
 *                            to be passed as the context parameter of the callback
 *                            routine each time it is called.
 *
 *    I2cWritePtr      [in] - A pointer to a i2cwirte callback routine, which is to
 *                            write I2C data. The callback routine must conform to
 *                            the following prototype:
 *
 *                        int (*fn_I2cWriteMem)(
 *                                void * context,
 *                                unsigned char slave_addr,
 *                                unsigned long sub_addr,
 *                                unsigned long write_len,
 *                                unsigned char* write_buf,
 *                             );
 *
 *                        The callback routine parameters are as follows:
 *
 *                        context          [in] - A pointer to a caller-supplied
 *                                                context area as specified in the
 *                                                CallbackContext parameter of
 *                                                SetupI2cWriteMemCallback.
 *                        slave_addr       [in] - The i2c chip address.
 *                        sub_addr         [in] - The i2c data address.
 *                        write_len        [in] - The size of the output buffer, in u8s.
 *                        write_buf        [in] - A pointer to the putput buffer that contains
 *                                                the data required to perform the operation.
 *
 *    cbMaxWriteBufSize  [in] - Specify the maximux transfer size for a I2c continue
 *                              writing with 'STOP'. This is limited in I2C bus Master
 *                              device. The size can not less then 4.
 *
 * RETURN
 *
 *    None
 *
 */
extern void SetupI2cWriteMemCallback( void * pCallbackContext,
    fn_I2cWriteMem         I2cWritePtr,
    unsigned int          cbMaxWriteBufSize)
{
    g_pContextI2cWrite  = pCallbackContext;
    g_I2cWriteMemPtr    = I2cWritePtr;
    g_cbMaxI2cWrite     = cbMaxWriteBufSize&0xfffffffc;
}

/*
 * The SetupI2cReadMemCallback sets i2cReadMem callback function.
 *
 * PARAMETERS
 *
 *    pCallbackContext    [in] - A pointer to a caller-defined structure of data items
 *                               to be passed as the context parameter of the callback
 *                               routine each time it is called.
 *
 *    pI2cReadMemPtr      [in] - A pointer to a i2cwirte callback routine, which is to
 *                               write I2C data. The callback routine must conform to
 *                               the following prototype:
 *
 *                        int (*fn_I2cReadMem)(
 *                                void * context,
 *                                unsigned char slave_addr,
 *                                unsigned long sub_addr,
 *                                unsigned long rd_len
 *                                void*         rd_buf,
 *                             );
 *
 *                        The callback routine parameters are as follows:
 *
 *                         context            [in]  - A pointer to a caller-supplied
 *                                                    context area as specified in the
 *                                                    CallbackContext parameter of
 *                                                    SetupI2cWriteMemCallback.
 *                         slave_addr         [in]  - The i2c chip address.
 *                         sub_addr           [in]  - slave addr.
 *                         rd_buf             [out] - A pointer to the input buffer
 *                         rd_len             [in]  - Specify the read data size.
 *
 *   cbMaxI2CRead         [in] - Specify the maximux transfer size for a I2c continue
 *                               read. The size can not less then 4.
 * RETURN
 *      None
 *
 */
extern void SetupI2cReadMemCallback( void * pCallbackContext,
    fn_I2cReadMem pI2cReadMemPtr,
    unsigned int cbMaxI2CRead)
{
    g_pContextI2cWriteThenRead  = pCallbackContext;
    g_I2cReadMemPtr             = pI2cReadMemPtr;
    g_cbMaxI2cRead              = cbMaxI2CRead & ~((unsigned int) 0x3); // Needs 4 u8s aligned.
}


/*
* Set the SetResetPin callback function.
*
* PARAMETERS
*
*    pCallbackContext    [in] - A pointer to a caller-defined structure of data items
*                               to be passed as the context parameter of the callback
*                               routine each time it is called.
*
*    SetResetPinPtr      [in] - A pointer to a i2cwirte callback routine, which is to
*                               write I2C data. The callback routine must conform to
*                               the following prototype:
*
*                        int (*fn_SetResetPin)(
*                                  void * pCallbackContext,
*                                  int    bSet );
*
*                        The callback routine parameters are as follows:
*
*                         pCallbackContext [in] - A pointer to a caller-supplied
*                                                 context area as specified in the
*                                                 CallbackContext parameter of
*                                                 SetupI2cWriteMemCallback.
*                         bSet             [in] - Indicates whether to high or low the GPIO pin.
*
* RETURN
*
*    If the operation completes successfully, the return value is ERRNO_NOERR.
*    Otherwise, return ERRON_* error code.
*
*/
extern void SetupSetResetPin(  void * pCallbackContext,
    fn_SetResetPin  SetResetPinPtr)
{
    g_SetResetPinPtr = SetResetPinPtr;
    g_pContextSetResetpin = pCallbackContext;
}


/*
 * cx_get_buffer_size.
 *
 *  Calculates the buffer size required for firmware update processing..
 *
 * PARAMETERS
 *    None
 *
 * RETURN
 *
 *    return buffer size required for firmware update processing..
 *
 */
extern u32 GetSizeOfBuffer(void)
{
    return BUFFER_SIZE;
}




/*
* Convert a 4-u8 number from generic u8 order into Little Endia
*/
static u32 to_little_endia_ul(u32 i)
{
#if __BYTE_ORDER != __LITTLE_ENDIAN
    return u8_swap_ulong(i);
#else
    return i;
#endif
}

/*
* Convert a 2-u8 number from generic u8 order into Little Endia
*/
static uint16_t to_little_endia_us(uint16_t i)
{
#if __BYTE_ORDER != __LITTLE_ENDIAN
    return u8_swap_ushort(i);
#else
    return i;
#endif
}



/*
* Convert a 4-u8 number from little endia into generic u8
*/
u32 from_little_endia_ul(u32 i)
{
#if __BYTE_ORDER != __LITTLE_ENDIAN
    return u8_swap_ulong(i);
#else
    return i;
#endif
}

// return 1 if the cpu is little_endian
static int is_little_endian(void)
{
  unsigned int endianness = 1;
  return ((char*)(&endianness))[0]==1;
}


static void signal_mem_handler  ( int n )
{
      LOG((DBG_ERROR "A segmentation fault occurs. number = %d\n",n));
}
/*
* Read a u8 from the specified  register address.
*
* PARAMETERS
*
*    i2c_sub_addr             [in] - Specifies the register address.
*    pErrNo              [out]  A pointer to a int, to retrieve the error code value.
*                               If operation is successful, then the number is zero. Otherwise,
*                               return ERRON_* error code.
*
* RETURN
*
*    Returns the u8 that is read from the specified register address.
*
*/
static u32 i2c_sub_read(u32 i2c_sub_addr, int *pErrNo)
{
    u32 val = 0;

    if(!g_I2cReadMemPtr)
    {
        LOG((DBG_ERROR "i2C function is not set.\n"));
        if(pErrNo) *pErrNo = -ERRNO_I2CFUN_NOT_SET;
        return 0;
    }

    if ( i2c_sub_addr & 0x3)
    {
      LOG((DBG_ERROR "The I2C read address is NOT 4 u8s aligned \n"));
       if(pErrNo) *pErrNo =-ERROR_I2CREAD_ADDR_MISALIGNEMNT;
    }

    if(g_I2cReadMemPtr(g_pContextI2cWriteThenRead, g_bChipAddress,
        i2c_sub_addr,1,(unsigned char*)&val)<0)
    {
        if(pErrNo) *pErrNo = -ERROR_I2C_ERROR;
    }
    else
    {
        if(pErrNo) *pErrNo = ERRNO_NOERR;
        val = from_little_endia_ul(val);
    }
    return val;
}


/*
* Write a u8 from the specified register address.
*
* PARAMETERS
*
*    i2c_sub_addr             [in] - Specifies the register address.
*
* RETURN
*
*    Returns the u8 that is read from the specified register address.
*
* REMARK
*
*    The g_I2cWriteMemPtr must be set before calling this function.
*/
static int i2c_sub_write(u32 i2c_sub_addr, u32 i2c_data)
{

    if(!g_I2cWriteMemPtr)
    {
        LOG((DBG_ERROR "i2C function is not set.\n"));
        return -ERRNO_I2CFUN_NOT_SET;
    }

    if ( i2c_sub_addr & 0x3)
    {
      LOG((DBG_ERROR "The I2C write address is NOT 4 u8s align \n"));
      return -ERROR_I2CWRITE_ADDR_MISALIGNEMNT;
    }

    // the address is big-endian, but the data is little-endian.
    i2c_data= to_little_endia_ul(i2c_data);

    return g_I2cWriteMemPtr(g_pContextI2cWrite,g_bChipAddress,i2c_sub_addr,1,(uint8_t*) &i2c_data);

}

/*
*  Writes a number of u8s from a buffer to Channel via I2C bus.
*
* PARAMETERS
*
*    num_u8         [in] - Specifies the number of u8s to be written
*                              to the memory address.
*    pData              [in] - Pointer to a buffer from an array of I2C data
*                              are to be written.
*
* RETURN
*
*    If the operation completes successfully, the return value is ERRNO_NOERR.
*    Otherwise, return ERRON_* error code.
*/
static int i2c_sub_burst_write(u32 startAddr, u32 num_u8, const unsigned char *pData)
{
    int err_no    = ERRNO_NOERR;
    u32  BytesToProcess       = 0;

    u32  cbMaxDataLen         = g_cbMaxI2cWrite;


    if( num_u8 & 0x3 )
    {
      LOG((DBG_ERROR "The data size for I2C write is NOT 4 u8s aligned \n"));
      return -ERROR_I2CWRITE_DATA_MISALIGNMENT;
    }
    if ( startAddr & 0x3)
    {
      LOG((DBG_ERROR "The I2C write address is NOT 4 u8s align \n"));
      return -ERROR_I2CWRITE_ADDR_MISALIGNEMNT;
    }

    if(!g_I2cWriteMemPtr )
    {
        LOG((DBG_ERROR "i2C function is not set.\n"));
        return -ERRNO_I2CFUN_NOT_SET;
    }

    for(;num_u8;)
    {

        BytesToProcess = num_u8 > cbMaxDataLen ? cbMaxDataLen :num_u8;

        if(g_I2cWriteMemPtr(g_pContextI2cWrite,g_bChipAddress, startAddr, BytesToProcess/4,(uint8_t*)pData)<0)
        {
            err_no = -ERROR_I2C_ERROR ;
            break;
        }
        num_u8-=BytesToProcess;
        startAddr+=BytesToProcess;
        pData +=  BytesToProcess;
    }

    return err_no;
}
/*
*  Read a number of u8s from  I2C bus.
*
* PARAMETERS
*
*    startAddr            [in] - Specifies the i2c address.
*    num_u8             [in] - Specifies the number of u8s to read
*    pData                [out] - A pointer to read buffer.
*
*
* RETURN
*
*    If the operation completes successfully, the return value is ERRNO_NOERR.
*    Otherwise, return ERRON_* error code.
*/

static int i2c_sub_burst_read(u32 startAddr, u32 num_u8, uint8_t *pData)
{
    int err_no = 0;
    u32 cbByteToRead ;


    if( num_u8 & 0x3 )
    {
      LOG((DBG_ERROR "The data size for I2C read is NOT 4 u8s aligned \n"));
      return -ERROR_I2CREAD_DATA_MISALIGNMENT;
    }
    if ( startAddr & 0x3)
    {
      LOG((DBG_ERROR "The I2C read address is NOT 4 u8s align \n"));
      return -ERROR_I2CREAD_ADDR_MISALIGNEMNT;
    }

    if(!g_I2cReadMemPtr)
    {
        LOG((DBG_ERROR "i2C function is not set.\n"));
        err_no = -ERRNO_I2CFUN_NOT_SET;
        return err_no;
    }

    for(;num_u8;)
    {
      cbByteToRead  = num_u8 > g_cbMaxI2cRead ?  g_cbMaxI2cRead : num_u8;

      // the address is big-endian, but the data is little-endian.
      if(g_I2cReadMemPtr(g_pContextI2cWriteThenRead, g_bChipAddress,
        startAddr,cbByteToRead/4,(uint8_t*)pData)<0)
      {
        err_no = -ERROR_I2C_ERROR;
        break;
      }
      else
      {
        err_no = ERRNO_NOERR;
      }

      //update pointer;
      num_u8 -= cbByteToRead;
      pData      += cbByteToRead;
      startAddr  +=  cbByteToRead;
    };

    return err_no;
}

/*
 *  Transmit a command via I2C bus.
 *
 * PARAMETERS
 *
 *    header              [in] - A pointer to write buffer of header.
 *    num_header          [in] - Specifies the number of u8s to write
 *    payload             [in] - A pointer to write buffer of command.
 *    num_payload         [in] - Specifies the number of u8s to write
 *
 * RETURN
 *
 *    If the operation completes successfully, the return value is ERRNO_NOERR.
 *    Otherwise, return ERRON_* error code.
 */
static int i2c_transmit( const unsigned char *header, u32 num_header, const unsigned char *payload, unsigned num_payload  )
{
   int err_no = 0;

   if( num_payload && (err_no==ERRNO_NOERR) )
   {
     if( payload == NULL )
     {
       err_no = ERROR_NULL_POINTER;
     }
     else
     {
       err_no = i2c_sub_burst_write(num_header,num_payload, payload);
     }
   }
   err_no = err_no==ERRNO_NOERR ? i2c_sub_burst_write(4,num_header-4, header+4): err_no;
   err_no = err_no==ERRNO_NOERR ? i2c_sub_burst_write(0,4, header): err_no;
   return err_no;
}





// Returns the actual size of the image (without padding)
static struct sfs_header_s * ParseImageHeader(const unsigned char * image_data,
                                       u32      * image_size,
                                       int *is_unpadded)
{
  u32  cbImageDataInUse;
  union sfs_record_u *pCurRecord;
  struct sfs_header_s *header1;
  int index;

  struct sfs_header_s *header = (struct sfs_header_s *)image_data;


  if (header->magic != SFS_MAGIC_HEADER)
  {
    header = (struct sfs_header_s *)(image_data+SFS_BLKSIZE);
    if ((uint) header->magic != (uint)SFS_MAGIC_HEADER)

    {
      return NULL;
    }
  }


    /*Check if the firmware version information is avaiable or not*/
  if( image_data[0x40+3] != 0xff )
  {	int j;
	  for( j=0;j<4;j++)
	  {
		g_firmware_version[j] =
		  from_little_endia_ul(*((u32*)&image_data[0x40+j*sizeof(u32)]));
	  }
  }


  header1 =  (struct sfs_header_s *) &image_data[SFS_HEADER_ALT_POS*SFS_BLKSIZE];

  g_is_dual_img = (header1->magic == SFS_MAGIC_HEADER) ? 1:0;

  if (*image_size > (u32)(header->sfssize*SFS_BLKSIZE))
  {
      return NULL;
  }

  // check if it is an unpadding image.
  if (*image_size < (u32)(header->sfssize*SFS_BLKSIZE))
  {
    if(is_unpadded) *is_unpadded = 1;
    return header;
  }
  else
  {
      if(is_unpadded) *is_unpadded = 0;
  }

  cbImageDataInUse  = 0;
  pCurRecord = (union sfs_record_u *)&image_data[FIRST_FREE_BLK*SFS_BLKSIZE];

  while (pCurRecord->hdr.magic != SFS_MAGIC_END)
  {
    u32 ulAllocatedSize = pCurRecord->hdr.asize*SFS_BLKSIZE;
    pCurRecord        += pCurRecord->hdr.asize;
    cbImageDataInUse  += ulAllocatedSize;
    if ((cbImageDataInUse + (FIRST_FREE_BLK+1)*SFS_BLKSIZE) > *image_size) {
      return NULL;
	}
  }

  cbImageDataInUse += (FIRST_FREE_BLK+1)*SFS_BLKSIZE;




 *image_size = cbImageDataInUse;
  return header;
}

/* RENO specific code*/

static uint hash(const void *dat,uint n)
  {
  const u16 *d=(const u16 *)dat;
  uint s1=0xffff;
  uint s2=0xffff;
  n=n/sizeof(short);
  while(n)
    {
    u32 l=n>359?359:n;
    n-=l;
    do {
      s2+=s1+=*d++;
      } while (--l);
    s1=(s1&0xffff)+(s1>>16);
    s2=(s2&0xffff)+(s2>>16);
    }
  /* Second reduction step to reduce sums to 16 bits */
  s1=(s1&0xffff)+(s1>>16);
  s2=(s2&0xffff)+(s2>>16);
  return (s2<<16)|s1;
  }

static int i2c_send(const void *tbuf,uint tlen,void *rbuf,uint rlen, const void *tpayld, uint tpaylen, void *rpayld, uint rpayldlen)
  {
    int err_no = ERRNO_NOERR;
  struct i2c_message_s *msg;

  msg=(struct i2c_message_s *)tbuf;
  msg->blk=g_i2c_blknr;
  msg->len= to_little_endia_us((uint16_t)tlen);
  msg->crc=0;
  msg->crc=to_little_endia_ul(hash(msg,tlen));

  err_no = i2c_transmit((uint8_t*)msg,tlen,(u8 *)tpayld,tpaylen);
  if(ERRNO_NOERR != err_no) return err_no;

  msg=(struct i2c_message_s *)rbuf;
  for(;;)
  {
    if(i2c_sub_burst_read(0,4,(uint8_t*)msg) != ERRNO_NOERR)
    {
      return ERROR_I2C_ERROR;
    }
    else if (msg->repl==1)
      break;
  }
  g_i2c_blknr++;

  if (msg->err)
  {
     if ( msg->cmd == -1)
     {
       LOG((DBG_ERROR "\nGot I2C message error: cmd = 0x%08x , block nr = %d\n", msg->cmd,msg->repl ));
       g_i2c_blknr = msg->repl;
       return ERROR_I2C_BLOCK_NR_ERROR;
     }
    LOG((DBG_ERROR "\nGot I2C message error: cmd = 0x%08x \n", msg->cmd));
    return ERROR_SEND_I2C_ERROR;
  }

  if(i2c_sub_burst_read(0,rlen,((uint8_t*)msg)) != ERRNO_NOERR)
  {
    return ERROR_I2C_ERROR;
  }

  if( rpayldlen)
  {
    err_no = i2c_sub_burst_read(rlen,rpayldlen,(uint8_t*)rpayld) ;
  }

  return err_no;
}

static int i2c_ping(u32 *pRet)
{
  int err_no = ERRNO_NOERR;

  struct i2c_ping_cmd_s *cmd = ( struct i2c_ping_cmd_s *) g_pWrBuffer;
  struct i2c_ping_rpl_s *rpl = (struct i2c_ping_rpl_s *)g_pRdBuffer;

  if(pRet == NULL) return -ERROR_NULL_POINTER;
  cmd->hdr.cmd=I2C_PING;
  cmd->hdr.err=0;
  cmd->hdr.repl=0;
  err_no = i2c_send(cmd,sizeof(*cmd),rpl,sizeof(*rpl),NULL,0,NULL,0);
  if( err_no == ERRNO_NOERR)
  {
    *pRet = from_little_endia_ul(rpl->id);
  }
  return err_no;
}

static int i2c_read_status(u32 *pRet)
{
  int err_no = ERRNO_NOERR;
  struct i2c_rstat_cmd_s *cmd =(struct i2c_rstat_cmd_s *) g_pWrBuffer;
  struct i2c_rstat_rpl_s *rpl = (struct i2c_rstat_rpl_s *)g_pRdBuffer;
  if(pRet == NULL) return -ERROR_NULL_POINTER;
  cmd->hdr.cmd=I2C_RSTAT;
  cmd->hdr.err=0;
  cmd->hdr.repl=0;

  err_no = i2c_send(cmd,sizeof(*cmd),rpl,sizeof(*rpl),NULL,0,NULL,0);
  if( err_no == ERRNO_NOERR)
  {
    *pRet = from_little_endia_ul (rpl->status);
  }
  return err_no;
}

static int i2c_write_status(u32 status,u32 *pRet)
{
  int err_no = ERRNO_NOERR;
  struct i2c_wstat_cmd_s *cmd = (struct i2c_wstat_cmd_s *)g_pWrBuffer;
  struct i2c_wstat_rpl_s *rpl = (struct i2c_wstat_rpl_s *)g_pRdBuffer;


  cmd->hdr.cmd=I2C_WSTAT;
  cmd->hdr.err=0;
  cmd->hdr.repl=0;
  cmd->status=to_little_endia_ul(status);
  err_no = i2c_send(cmd,sizeof(*cmd),rpl,sizeof(*rpl),NULL,0,NULL,0);
  if( pRet)
  {
    *pRet = from_little_endia_ul(rpl->status);
  }
  return err_no;
}

static int i2c_erase_sector(u16 sector,u32 *pRet)
{
  int err_no = ERRNO_NOERR;
  struct i2c_erases_cmd_s *cmd = (struct i2c_erases_cmd_s *)g_pWrBuffer;
  struct i2c_erases_rpl_s *rpl = (struct i2c_erases_rpl_s *)g_pRdBuffer;

  if( sector >=2  ) sector +=g_partial_offset;

  cmd->hdr.cmd=I2C_ERASES;			//  erase 4K sector
  cmd->hdr.err=0;
  cmd->hdr.repl=0;
  cmd->sector=to_little_endia_us(sector);
  cmd->padding=0;
  err_no = i2c_send(cmd,sizeof(*cmd),rpl,sizeof(*rpl),NULL,0,NULL,0);
  if( pRet)
  {
    *pRet = from_little_endia_ul(rpl->status);
  }
  return err_no;
}

static int i2c_erase_chip(u32 *pRet)
{
  int err_no = ERRNO_NOERR;
  struct i2c_erasec_cmd_s *cmd = (struct i2c_erasec_cmd_s *)g_pWrBuffer;
  struct i2c_erasec_rpl_s *rpl = (struct i2c_erasec_rpl_s *)g_pRdBuffer;

  cmd->hdr.cmd=I2C_ERASEC;			//  erase chip
  cmd->hdr.err=0;
  cmd->hdr.repl=0;


  err_no = i2c_send(cmd,sizeof(*cmd),rpl,sizeof(*rpl),NULL,0,NULL,0);
  if( pRet)
  {
    *pRet = from_little_endia_ul( rpl->status);
  }
  return err_no;
}

int i2c_read(u16 sector,u16 length,void *data,u32 *pRet)
{
  int err_no = ERRNO_NOERR;
  struct i2c_read_cmd_s   *cmd = (struct i2c_read_cmd_s   *)g_pWrBuffer;
  struct i2c_read_rpl_h_s *rpl = (struct i2c_read_rpl_h_s *)g_pRdBuffer;

  if( sector >= 2 ) sector +=g_partial_offset;

  cmd->hdr.cmd=I2C_READ;			//  read from a 4K block, from offset 0
  cmd->hdr.err=0;
  cmd->hdr.repl=0;
  cmd->sector=to_little_endia_us(sector);
  cmd->length=(u16)to_little_endia_ul(length);

  err_no = i2c_send(cmd,sizeof(*cmd),rpl,sizeof(*rpl),NULL,0,(uint8_t*)data,length);

  if( pRet)
  {
    *pRet = from_little_endia_ul(rpl->length);
  }
  return err_no;
}

static int i2c_write_normal(u16 sector,u16 length,const void *data,u32 *pRet)
  {
  int err_no = ERRNO_NOERR;
  struct i2c_write_cmd_h_s  *cmd = (struct i2c_write_cmd_h_s  *)g_pWrBuffer;
  struct i2c_write_rpl_s    *rpl = (struct i2c_write_rpl_s    *)g_pRdBuffer;

  if( sector >= 2 ) sector +=g_partial_offset;

  cmd->hdr.cmd=I2C_WRITE_NORMAL;			//  write to a 4K block, from offset 0
  cmd->hdr.err=0;
  cmd->hdr.repl=0;
  cmd->sector=to_little_endia_us(sector);
  cmd->length=(u16)to_little_endia_ul(length);

  err_no =  i2c_send(cmd,sizeof(*cmd),rpl,sizeof(*rpl),(u8*)data,length,NULL,0);

  if( pRet)
  {
    *pRet = from_little_endia_ul(rpl->status);
  }
  return err_no;
}

static int i2c_write_verify(u16 sector,u16 length,const void *data, u32 *pRet)
{
  int err_no = ERRNO_NOERR;
  struct i2c_write_cmd_h_s  *cmd = (struct i2c_write_cmd_h_s  *)g_pWrBuffer;
  struct i2c_write_rpl_s    *rpl = (struct i2c_write_rpl_s    *)g_pRdBuffer;

  if( sector >= 2 ) sector +=g_partial_offset;

  cmd->hdr.cmd=I2C_WRITE_VERIFY;			//  write to a 4K block, from offset 0
  cmd->hdr.err=0;
  cmd->hdr.repl=0;
  cmd->sector=to_little_endia_us(sector);
  cmd->length=(u16)to_little_endia_ul(length);

  err_no =  i2c_send(cmd,sizeof(*cmd),rpl,sizeof(*rpl),data,length,NULL,0);

  if( pRet)
  {
    *pRet = from_little_endia_ul(rpl->status);
  }
  return err_no;
}

static int i2c_protect(uint state, u32 *pRet)
{
  int err_no = ERRNO_NOERR;
  struct i2c_protect_cmd_s  *cmd = (struct i2c_protect_cmd_s  *)g_pWrBuffer;
  struct i2c_protect_rpl_s  *rpl = (struct i2c_protect_rpl_s  *)g_pRdBuffer;

  cmd->hdr.cmd=I2C_PROTECT;			//  change sw protection (on or off)
  cmd->hdr.err=0;
  cmd->hdr.repl=0;
  cmd->state=to_little_endia_ul(state);

  err_no =   i2c_send(cmd,sizeof(*cmd),rpl,sizeof(*rpl),NULL,0,NULL,0);

  if( pRet)
  {
    *pRet =  from_little_endia_ul(rpl->state);
  }
  return err_no;


}

static int i2c_rtunnel(u32 rcmd,u16 clen,u16 rlen,void *rbuf)
  {
     int err_no = ERRNO_NOERR;
  struct i2c_rtunnel_cmd_s   *cmd = (struct i2c_rtunnel_cmd_s   *)g_pWrBuffer;
  struct i2c_rtunnel_rpl_h_s *rpl = (struct i2c_rtunnel_rpl_h_s *)g_pRdBuffer;

  cmd->hdr.cmd=I2C_RTUNNEL;			//  send a SPI read command to SPI memory
  cmd->hdr.err=0;
  cmd->hdr.repl=0;
  cmd->cmd= to_little_endia_ul(rcmd);
  cmd->clen= to_little_endia_us( clen);
  cmd->rlen= to_little_endia_us(rlen);

  err_no =   i2c_send(cmd,sizeof(*cmd),rpl,sizeof(*rpl),NULL,0,rbuf,rlen);

  return err_no;
  }

// retunr number of NON 0xffffffff data within specified range.
static u16 num_non_ff(u16 range, const uint *data)
{
    const uint *cur = data + range -1;
    for(;cur >= data;cur--)
    {
        if( *cur != 0xffffffff)
        {
            return (u16)(cur-data+1);
        }
    }
    return range;
}

static u32 i2c_verify_image(u16 hdr,u16 fst,u16 lst,u32 magic)
  {
  struct i2c_verify_img_cmd_s *cmd = (struct i2c_verify_img_cmd_s *)g_pWrBuffer;
  struct i2c_verify_img_rpl_s *rpl = (struct i2c_verify_img_rpl_s *)g_pRdBuffer;

  fst +=g_partial_offset;
  lst +=g_partial_offset;

  cmd->hdr.cmd=I2C_VERIFY_IMG;			//  send a SPI read command to SPI memory
  cmd->hdr.err=0;
  cmd->hdr.repl=0;
  cmd->header=hdr;
  cmd->first=fst;
  cmd->last=lst;
  cmd->padding=0;
  cmd->magic=magic;
  i2c_send(cmd,sizeof(*cmd),rpl,sizeof(*rpl),NULL,0,NULL,0);
  return rpl->status;
  }

static int i2c_small_write(u16 blk,u8 num_uint, const u8 *base)
{
  int err_no = ERRNO_NOERR;
  uint buf[MAX_SMALL_WRITE];
  u8 n;
  const uint * src = (uint *)&base[SFS_BLK2POS(blk)];

  do{
    BIF(num_uint > MAX_SMALL_WRITE);
    BIF(i2c_read(blk,num_uint*sizeof(uint),buf,NULL));
    for( n=0;n!=num_uint;n++)
    {
      if( buf[n] != src[n] )
      {
        break;
      }
    }
    if ( n != num_uint)  /* not idential */
    {
      if (!g_bEraseChip) {
        for( n=0;n!=num_uint;n++)
        {
          if( buf[n] != 0xffffffff )
          {
            break;
          }
        }
        if ( n != num_uint)  /* if not all 0xff*/
        {
          BIF(i2c_erase_sector((u16)blk, NULL));
        }
      }
      err_no = i2c_write_verify((u16)blk,num_uint*sizeof(uint),src,NULL);
    }
  }while(0);
  return err_no;
}

static int i2c_download_block(u16 fst,u16 end,const u8 *base, uint cur, uint total)
{
  int err_no = ERRNO_NOERR;

  u16              c;
  uint              len;
  struct sfs_padded_s * p;

  for(c=fst;c<end && (err_no == ERRNO_NOERR) ;c++,cur++)
  {
    p=(struct sfs_padded_s *)&base[SFS_BLK2POS(c)];
    if( p->magic == SFS_MAGIC_HDRPAD)
    {
      BIF(i2c_small_write(c,ALIG_SIZE(struct sfs_padded_s) ,base));
      err_no = ERRNO_NO_MORE_DATA;
    }
    else
    {
      if (!g_bEraseChip) {
        BIF(i2c_erase_sector((u16)c, NULL));
      }
      len = num_non_ff(SFS_BLKSIZE/sizeof(uint),(uint*)&base[SFS_BLK2POS(c)]);
      if(len)
      {
        err_no = i2c_write_verify((u16)c,(u16)len*sizeof(uint),&base[SFS_BLK2POS(c)],NULL);
      }
    }

    if( err_no < 0)
    {
      ShowProgress(cur,0,I2C_ERR,total);
      break;
    }
    else
    {
      ShowProgress(cur,0,I2C_OK,total);
    }
  }
  return err_no;
}


static int i2c_download_partition(u16 part,const u8 *base)
{
  int err_no = ERRNO_NOERR;

  struct sfs_header_s *sfshdr=(struct sfs_header_s *)&base[SFS_BLK2POS(part)];
  struct sfs_encapsulated_s *e;
  struct sfs_end_s * x;
  u16   fst,end,n, real_end;
  int    found = 0;

  if( g_is_partial_img )
  {
     struct sfs_header_s *sfshdr0 =(struct sfs_header_s *)&base[SFS_BLK2POS(0)];
     fst = sfshdr0->sfsbeg-1;
     g_partial_offset = sfshdr->sfsbeg - sfshdr0->sfsbeg;
  }
  else
  {
    fst = sfshdr->sfsbeg-1;
    g_partial_offset = 0;
  }

  e=(struct sfs_encapsulated_s *)&base[SFS_BLK2POS(fst)];
  end= real_end = fst+e->asize;

  if (e->magic!=SFS_MAGIC_HDRECP)
  {
      LOG((DBG_ERROR "Invalid encapsulation header\n"));
      return -ERROR_INVALID_IMAGE;
  }
  // get the real image data size.
  for(n=fst;n<=real_end ;n++)
  {
    x =(struct sfs_end_s *) &base[SFS_BLK2POS(n)];
    if( x->magic == SFS_MAGIC_HDRPAD)
    {
      found = 1;
      end = n+1;
      break;
    }
  }

  if( found == 0)
  {
    LOG((DBG_ERROR "Padded magic id not found!\n"));
    return -ERROR_INVALID_IMAGE;
  }

  n=end-fst+1;	// SFS header+nr of 4K blocks in the partition
  LOG((DBG_INFO "\nUpdating partition %d...",part));

  InitShowProgress( n);

  // write the header, mark the header inactive (deleted)
  x=(struct sfs_end_s *)&base[SFS_BLK2POS(part)];
  x->magic=SFS_MAGIC_HDRDEL;
  err_no = i2c_download_block(part,part+1,base,1,n);
  // write the partition
  err_no = err_no ? err_no:i2c_download_block(fst,end,base,2,n);
  if( ERRNO_NO_MORE_DATA == err_no) err_no =ERRNO_NOERR;
  if( err_no < 0 ) return err_no;

  LOG((DBG_INFO "\n\nVerify CRC for partition %d...",part));

  // Jeter 20170109 Don't need to verify the image if the update mode is SFS_UPDATE_AUTO.
  //if (i2c_verify_image((u16)part,(u16)fst,
  //  (u16)end,SFS_MAGIC_HEADER)==0)
  if ((g_update_mode==SFS_UPDATE_AUTO)||(i2c_verify_image((u16)part,(u16)fst,
    (u16)end,SFS_MAGIC_HEADER)==0))
    LOG((DBG_INFO  "Good\n"));
  else
  {
    LOG((DBG_ERROR "Error\n"));
    return -ERROR_CRC_CHECK_ERROR;
  }
  LOG((DBG_INFO "Mark partition %d as active...",part));
  x=(struct sfs_end_s *)&base[SFS_BLK2POS(part)];
  x->magic=SFS_MAGIC_HEADER;
  err_no = i2c_erase_sector((u16)part, NULL);
  err_no = err_no ? err_no:i2c_write_verify((u16)part,SFS_BLKSIZE,&base[SFS_BLK2POS(part)],NULL);
  if (err_no<0)
    LOG((DBG_INFO "\tError\n"));
  else
    LOG((DBG_INFO "\tDone\n"));
  return err_no;
}

static const char* const get_spi_mem_name(uint id)
{
  unsigned int i;
  for(i=0;i<sizeof(g_spiid_2_name_tlb)/sizeof(g_spiid_2_name_tlb[0]);
    i++)
  {
    if( id == g_spiid_2_name_tlb[i].id)
      return g_spiid_2_name_tlb[i].name;
  }
  return "Unknown spi";
}

static int i2c_download_image(const u8 *img,uint siz)
{
  int err_no = ERRNO_NOERR;
  u32 magic;
  unsigned int update=0;
  u32 id;
  u32 status;
  u16 partition;

  g_i2c_blknr=0;
  do
  {
    BIF(i2c_ping(&id));

    LOG((DBG_INFO "\tSPI memory has ID   : %08xh => %s \n",id,get_spi_mem_name(id)));
    BIF(i2c_read_status(&status));
    LOG((DBG_INFO "\tSPI memory status   : %08xh\n",status));
    LOG((DBG_INFO "\tImage file size     : %xh u8s\n",siz));

#if defined(_WINDOWS )
    for ( partition=0;partition<2;partition++)
    {
      i2c_read((u16)partition,4,&magic,&id);
      LOG((DBG_INFO "\tpartition %d : %s\n",partition,SFS_MAGIC_HEADER == magic?"Active":"Inactive"));
    }
#endif

    BIF(i2c_protect(0,NULL));

    if ( g_bEraseChip)
    {
      LOG((DBG_INFO "Erasing chip .. "));
      BIF(i2c_erase_chip(NULL));
      LOG((DBG_INFO "Done\n"));
    }

    // Automatically determine which partition shold be updated.
    if(g_update_mode == SFS_UPDATE_AUTO)
    {
      i2c_read(0,4,&magic,&id);
      if(SFS_MAGIC_HEADER != magic)
        update = SFS_UPDATE_PARTITION_0;
      else
        update = SFS_UPDATE_PARTITION_1;
    }
    else
    {
      update = g_update_mode;
    }

    for (partition=0;partition<2&&err_no == ERRNO_NOERR;partition++)
    {
      if (update&(1<<partition))
      {
         BIF(i2c_download_partition(partition,img));
      }
    }
  } while(0);

  if ((g_update_mode == SFS_UPDATE_AUTO)&&(err_no == ERRNO_NOERR))
  {
    for (partition=0;partition<2&&err_no == ERRNO_NOERR;partition++)
    {
      // figure out which partition wasn't updated
      if (!(update&(1<<partition)))
      {
        LOG((DBG_INFO "Mark partition %d as inactive...",partition));
        magic=SFS_MAGIC_HDRDEL;
        err_no=i2c_write_normal((u16)partition,sizeof(magic),&magic,NULL);
        if (err_no<0)
          LOG((DBG_INFO "\tError\n"));
        else
          LOG((DBG_INFO "\tDone\n"));
      }
    }
  }

  i2c_protect(1,NULL);

  return err_no;
  }




static int wait_for_loader(void)
{
  unsigned int ret;
  int err_no = ERRNO_NOERR;
  unsigned int   time_out_loop = TIME_OUT_MS/POLLING_INTERVAL_MS;
  u32 ready  = TRANSFER_STATE_STANDBY;

  if( g_bIsRenoDev )
  {
      ready  = TRANSFER_STATE_RENO_LOADER_READY;
  }


  /*
  # Look for reply at address 0. Flags that a transfer can be initiated
  */
  for(;time_out_loop;time_out_loop--)
  {
    if( ((ret= i2c_sub_read(0x00,&err_no))  == ready) && (err_no ==ERRNO_NOERR))
    {

      break;
    }

    cx_mdelay(POLLING_INTERVAL_MS);
  }
  if( time_out_loop == 0)
  {
    LOG((DBG_ERROR " aborting. timeout wating for device after %d ms\n",TIME_OUT_MS));
    err_no = -ERROR_WAITING_LOADER_TIMEOUT;
  }

  return err_no;
}

static int TransferData( const unsigned char * image_data, unsigned int image_size)
{

  int err_no = ERRNO_NOERR;
  const unsigned char *pBlock;
  const unsigned char *pCur;
  const unsigned char  *pBlockEnd = NULL;
  u32  ulBlockCheckSum  = 0;
  u32  ulBlockIndex     = 0;
  u32  ulRemainder       = 0;
  u32  nRetry           = 0;
  u32  done             = 0;
  u32  CRCReturned      = 0;
  struct i2c_xfer_reno_s          i2c_xfer_reno;
  struct i2c_xfer_s               i2c_xfer;


  u32  max_xfer_size =   IMG_BLOCK_SIZE;


  int  ReadErr = ERRNO_NOERR;
  //
  // Download FW image data.
  //
  if( g_bIsRenoDev )
  {
    max_xfer_size = IMG_BLOCK_SIZE_RENO;
  }
  InitShowProgress(image_size);
  ulBlockIndex=0;
  pBlock = image_data;
  ulRemainder = image_size;
  for(;;){

    u32 state = i2c_sub_read(0x00,&ReadErr);
    if ( ReadErr !=ERRNO_NOERR )
    {
      ShowProgress(image_size-ulRemainder,0,I2C_ERR,image_size);
      err_no = ReadErr;
      break;
    }
    switch(state)
    {
    case TRANSFER_STATE_FATAL://# device->host: fatal error
      LOG((DBG_ERROR "\nFATAL error happened. aborting. \n"));
      err_no = -ERROR_STATE_FATAL;
      break;
    case TRANSFER_STATE_BAD_PACKET://# device->host: bad packet. retry
      {
        if( nRetry >= MAX_RETRIES )
        {
          i2c_sub_write(0x0000,TRANSFER_STATE_ABORT_TRANSFER);
          LOG((DBG_ERROR "\naborting. exceeded max number of retries \n"));
          ShowProgress(image_size-ulRemainder,0,I2C_ERR,image_size);
          err_no = -ERROR_LOAD_IMG_TIMEOUT;
          break;
        }
        else
        {
          /// re-try
          if( g_bIsRenoDev )
          {
            i2c_transmit((unsigned char*)&i2c_xfer_reno,sizeof(i2c_xfer_reno),
                          (unsigned char*)pBlock,pBlockEnd-pBlock );
            ShowProgress(image_size-ulRemainder,0,I2C_RETRY,image_size);
          }
          else
          {

            i2c_transmit((unsigned char*)&i2c_xfer,sizeof(i2c_xfer),
              (unsigned char*)pBlock,pBlockEnd-pBlock );
            ShowProgress(image_size-ulRemainder,0,I2C_RETRY,image_size);

          }
          nRetry++;

        }
      }
      break;
    case TRANSFER_STATE_STANDBY://#  device->host: standing by for next packet.
      {
        int  *xfer_int;
        nRetry =0;
        if(pBlockEnd != 0)
        {
          ulRemainder = image_size - (pBlockEnd -image_data);
          pBlock = pBlockEnd;
          ulBlockIndex++;
        }

        if(ulRemainder == 0)
        {   // No remainder data, write 'D' to address 0.
          ShowProgress(image_size,0,I2C_OK,image_size);
          //LOG((DBG_INFO "\nImage transfer completed successfully. \n"));
          i2c_sub_write(0x0000, TRANSFER_STATE_FILE_DONE);

          break;
        }

        pBlockEnd = pBlock+ ( ulRemainder > max_xfer_size ?max_xfer_size:ulRemainder);

        if( g_bIsRenoDev )
        {
          // get the checksum of this block data.
          ulBlockCheckSum = 0;
          for( pCur =pBlock;pCur!=pBlockEnd; pCur +=4)
          {
            ulBlockCheckSum+= from_little_endia_ul(*((u32*)pCur));
          }

          i2c_xfer_reno.xfer.xfer_flag    = TRANSFER_STATE_PACKET_READY;
          i2c_xfer_reno.xfer.block_index  = ulBlockIndex;
          i2c_xfer_reno.xfer.check_sum    = 0;
          i2c_xfer_reno.num_u32         = (pBlockEnd-pBlock) / 4;

          // get the checksum of xfer header.
          for( xfer_int =(int*)&i2c_xfer_reno;
                xfer_int !=((int*)&i2c_xfer_reno)+4; xfer_int++)
          {
            ulBlockCheckSum+= *xfer_int;
          }

         i2c_xfer_reno.xfer.check_sum  = ~ulBlockCheckSum;



         // conver the xfer to little endia
         for( xfer_int =(int*)&i2c_xfer_reno;
           xfer_int !=((int*)&i2c_xfer_reno)+4; xfer_int++)
         {
            *xfer_int = to_little_endia_ul(*xfer_int);
         }

         i2c_transmit((unsigned char*)&i2c_xfer_reno,sizeof(i2c_xfer_reno),
           (unsigned char*)pBlock,pBlockEnd-pBlock );
        }
        else
        {
          // get the checksum of this block data.
          ulBlockCheckSum = 0;
          for( pCur =pBlock;pCur!=pBlockEnd; pCur +=4)
          {
            ulBlockCheckSum+= from_little_endia_ul(*((u32*)pCur));
          }

          i2c_xfer.xfer_flag   = TRANSFER_STATE_PACKET_READY;
          i2c_xfer.block_index = ulBlockIndex;
          i2c_xfer.check_sum   = ulBlockCheckSum;

         i2c_transmit((unsigned char*)&i2c_xfer,sizeof(i2c_xfer),
           (unsigned char*)pBlock,pBlockEnd-pBlock );
        }
        if (!(g_bIsRenoDev || g_is_dual_img) && (image_size-ulRemainder ==0) )
		{
			  /* using old download firmare approach. this will take about few secondes
			   for easing whole chip*/
			ShowProgress(image_size-ulRemainder,0,I2C_ERASE_CHIPS,image_size);
		} else{
				ShowProgress(image_size-ulRemainder,0,I2C_OK,image_size);
		}
      }
      break;
    case TRANSFER_STATE_TRANSFER_COMPLETE://#  device->host: transfer complete
      {
        if (g_bIsRenoDev || g_sfs_version >= _SFS_VERSION(0,3,0,0))
        {
			done =1;
			break;
        }

        CRCReturned = i2c_sub_read(8,&ReadErr);
        if( ReadErr == ERRNO_NOERR)
        {
          if( CRCReturned != 0x53435243 )
          {
            ShowProgress(image_size,0,I2C_ERR,image_size);
            LOG((DBG_INFO "\nCRC error!\n"));
            err_no = -ERROR_CRC_CHECK_ERROR;
          }
          else
          {
            ShowProgress(image_size,0,I2C_OK,image_size);
            LOG((DBG_INFO "\nImage transfer completed successfully. \n"));
            done =1;
          }
        }
        else
        {
          ShowProgress(image_size,0,I2C_ERR,image_size);
          err_no = ReadErr;
        }

      }
      break;
	case TRANSFER_STATE_PACKET_READY:/*echo*/
	case TRANSFER_STATE_FILE_DONE:/*echo*/
		break;
    default:
		err_no = -ERRNO_FAILED;
      break;
    }

    if( err_no !=ERRNO_NOERR || done ) break;
  }
  return err_no;
}

static int download_image( const unsigned char * image_data, unsigned int image_size)
{
  if( g_bIsRenoDev )
  {
    return i2c_download_image(image_data,image_size);
  }
  else
  {
    return TransferData(image_data,image_size);
  }
}

static int download_boot_loader(const unsigned char *  loader_data, unsigned int num_u8)
{
      int err_no = ERRNO_NOERR;

      if( g_bIsRenoDev )
      {
        err_no = TransferData(loader_data,num_u8);
      }
      else
      {
        i2c_transmit(loader_data,num_u8,NULL,0);
      }
      return err_no;
}


/*
 * Download Firmware to Hudson.
 *
 * PARAMETERS
 *
 *    pBuffer             [in] - A potinter to a buffer for firmware update processing.
 *    pBootLoader         [in] - A pointer to the input buffer that contains I2C bootloader data.
 *    cbBootLoader        [in] - The size of Bootloader data in u8s.
 *    pImageData          [in] - A pointer to the input buffer that contains Image data.
 *    cbImageData         [in] - The size of Image data in u8s.
 *    slave_address        [in] - Specifies I2C chip address.
 *    DownLoadMode       [in] - Specifies the download mode, Could be the follwoing one of values.(Reno only)
 *                               0 :  auto
 *                               1 :  flash partition 0
 *                               2 :  flash partition 1
 *                               3 :  flash both partitions

 * RETURN
 *
 *    If the operation completes successfully, the return value is ERRNO_NOERR.
 *    Otherwise, return ERRON_* error code.
 *
 * REMARKS
 *
 *    You need to set up both I2cWrite and I2cWriteThenRead callback function by calling
 *    SetupI2cWriteMemCallback and SetupI2cReadMemCallback before you call this function.
 */
int cx_download_fw(        void *                      pBuffer,
                       const unsigned char *       loader_data,
                       u32               loader_size,
                       const unsigned char *       image_data,
                       u32               image_size,
                       unsigned char               slave_address,
                       SFS_UPDATE_PARTITION        eDownLoadMode)
{
    int err_no = ERRNO_NOERR;

    struct sfs_header_s *header;
//	struct sfs_header_v300_s *header300;

    int  ReadErr = ERRNO_NOERR;
    volatile u32       val;

    unsigned int   time_out_loop = TIME_OUT_MS/POLLING_INTERVAL_MS;
    const int      little_endian = is_little_endian();


    do{

      // Verfiy the u8 order
#if __BYTE_ORDER != __LITTLE_ENDIAN
      if(little_endian) return ERROR_MIS_ENDIANNESS;
#else
      if(!little_endian) return ERROR_MIS_ENDIANNESS;
#endif

#if defined(USER_DEBUG)
         signal (SIGSEGV, signal_mem_handler);
#endif

        // check if the work buffer is set or not.
        if( NULL == pBuffer )
        {
            err_no = -ERRNO_INVALID_PARAMETER;
            LOG((DBG_ERROR "The buffer pointer is not set\n"));
            break;
        }

        // check if the i2c function is set or not.
        if( NULL == g_I2cReadMemPtr||
            NULL == g_I2cWriteMemPtr )
        {
            err_no = -ERRNO_I2CFUN_NOT_SET;
            LOG((DBG_ERROR "i2C function is not set.\n"));
            break;
        }

        /*make sure the buffer align to 4 u8s boundary*/
	{
		int    x = ALIGNMENT_SIZE;
		x = ((size_t)pBuffer%ALIGNMENT_SIZE) ;
		g_pBuffer        = (uint8_t*)((uint8_t*)pBuffer + ALIGNMENT_SIZE-((size_t)pBuffer%ALIGNMENT_SIZE)) ;
	}
        g_pWrBuffer      = g_pBuffer;
        g_pRdBuffer      = g_pWrBuffer +MAX_WRITE_BUFFER;

        g_bChipAddress   = slave_address;
        g_update_mode    = eDownLoadMode;

        if ((header = ParseImageHeader(image_data,&image_size,&g_is_partial_img))==NULL)
        {
            err_no = -ERROR_INVALID_IMAGE;
            LOG((DBG_ERROR "SFS image invalid.\n"));
            break;
        }

        if( g_update_mode > SFS_UPDATE_BOTH )
        {
            err_no = -ERRNO_INVALID_PARAMETER;
            LOG((DBG_ERROR "Parameter is invalid.\n"));
            break;
        }

        // If bootloader isn't specified, look for a valid iflash.bin in the image. 
        if (loader_data == NULL)
        {
			u32 iFlash = BLKNONE;
			if ( !g_bIsRenoDev )
			{
				iFlash = ((struct sfs_header_v300_s *)header)->iflash;
			}
			else
			{
				iFlash = header->iflash;
			}

			if (iFlash ==(u32)BLKNONE)
			{
				err_no = -ERRON_INVALID_BOOTLOADER;
				LOG((DBG_ERROR "didn't find a valid iflash bootloader in image. aborting.\n"));
				break;
			}
			else
			{
				loader_data  = (unsigned char *)&image_data[iFlash*SFS_BLKSIZE];
				loader_size = SFS_BLKSIZE;
			}
        }

		g_sfs_version = g_bIsRenoDev?header->version:
				((struct sfs_header_v300_s *)header)->version;

        // check file for a valid bootloader
        if( *((u32 *)loader_data) == SFS_MAGIC_BOOT)
        {
           g_bIsRenoDev =0;  // Boot loader for B0 board.
           /*LOG((DBG_INFO "Found boot loader for Hudson\n"));*/
        }
        else if ( *((u32 *)loader_data) == SFS_MAGIC_BOOT_H)
        {
           g_bIsRenoDev =1;   // Boot loader is for Reno device.
           /*LOG((DBG_INFO "Found boot loader for Reno\n"));*/
        }
        else
        {
            err_no = -ERRON_INVALID_BOOTLOADER;
            LOG((DBG_ERROR "didn't find a valid bootloader magic number. aborting.\n"));
            break;
        }

        if( !g_is_dual_img && ( g_update_mode !=SFS_UPDATE_PARTITION_0 ))
        {
          g_update_mode = SFS_UPDATE_PARTITION_0;
        }

        // Dump the system environment
        LOG((DBG_INFO "Device Type          : %s \n", g_bIsRenoDev? "Reno":"Hudson"));
        LOG((DBG_INFO "I2C Slave Address    : %02xh\n", g_bChipAddress));
        LOG((DBG_INFO "    Max burst write  : %d u8s\n", g_cbMaxI2cWrite));
        LOG((DBG_INFO "    Max burst read   : %d u8s\n", g_cbMaxI2cRead));
        LOG((DBG_INFO "Endianness           : %s-Endian \n", little_endian? "Little":"Big"));
        LOG((DBG_INFO "SFS version          : %d.%d.%d.%d \n", (uint8_t) (g_sfs_version>>24),
													  (uint8_t) (g_sfs_version>>16),
													  (uint8_t) (g_sfs_version>>8),
													  (uint8_t) (g_sfs_version))); 
		if( ((u32) -1) != g_firmware_version[0])
			LOG((DBG_INFO "Firmware version     : %d.%d.%d.%d \n", g_firmware_version[0],
							g_firmware_version[1],g_firmware_version[2],g_firmware_version[3]));
      //  LOG((DBG_INFO "Firmware version     :
        LOG((DBG_INFO "Boot loader size     : %xh u8s\n", loader_size));
        LOG((DBG_INFO "Image size           : %xh u8s\n", image_size));
        LOG((DBG_INFO "Unpadded image       : %s \n", g_is_partial_img?"Yes":"No"));
        LOG((DBG_INFO "Is Dual Image        : %s \n", g_is_dual_img?"Yes":"No"));
        LOG((DBG_INFO "Update partition     : %s \n", update_mode_str[g_update_mode] ));

        LOG((DBG_INFO "\n"));

        LOG((DBG_INFO "Downloading boot loader\n"));

        // Check if system can reset the device programmatically or not.
        if( g_SetResetPinPtr )
        {
            // System can reset the device prgorammatically.
            g_SetResetPinPtr(g_pContextSetResetpin,0); //sets RESET PIN to low.
            cx_mdelay(RESET_INTERVAL_MS);
            g_SetResetPinPtr(g_pContextSetResetpin,1); //sets RESET PIN to high.
            time_out_loop = TIME_OUT_MS/POLLING_INTERVAL_MS;
        }
        else
        {   // Need to reset the device manually.
            LOG((DBG_INFO "Waiting. Please reset the board\n"));
            //sets the Timeout setting to 2 seconds.
            time_out_loop = MANUAL_TIME_OUT_MS/POLLING_INTERVAL_MS;
        }

        /*
        # Look for 'C' at address 0. Flags that a transfer can be initiated
        */
        for(;time_out_loop;time_out_loop--)
        {
            /*if( (u32)(i2c_sub_read(0x00,&ReadErr)  ==(u32) 0X43) && ((int)ReadErr==(int)ERRNO_NOERR))*/
            val = i2c_sub_read(0x00,&ReadErr);
            if( val==0X000043U)
            {
                break;
            }
           cx_mdelay(POLLING_INTERVAL_MS);
        }
        if( time_out_loop == 0)
        {
            LOG((DBG_ERROR " aborting. timeout wating for device after %d ms\n",
				(g_SetResetPinPtr?TIME_OUT_MS:MANUAL_TIME_OUT_MS)*POLLING_INTERVAL_MS));
            err_no = -ERROR_WAITING_RESET_TIMEOUT;
            break;
        }

        /*
        # Delete the magic number to signal that the download just started
        # but hasn't finished yet
        */
        i2c_sub_write(0x0000,0x0000); // to stop download sequence
        err_no =  download_boot_loader(loader_data,loader_size);
        if( err_no == 0)
        {
            LOG((DBG_INFO "\nBootloader transfer completed successfully\n"));
        }
        else
        {
            LOG((DBG_ERROR "\nFailed to download boot loader\n"));
            break;
        }

        if( (err_no =wait_for_loader()) !=0) break ;

        LOG((DBG_INFO "\nDownloading image\n"));
        err_no = download_image(image_data,image_size);
        if(err_no  == 0)
        {
          LOG((DBG_INFO "\nImage transfer completed successfully\n"));

          if( g_SetResetPinPtr )
          {
            // System can reset the device prgorammatically.
            g_SetResetPinPtr(g_pContextSetResetpin,0); //sets RESET PIN to low.
            cx_mdelay(RESET_INTERVAL_MS);
            g_SetResetPinPtr(g_pContextSetResetpin,1); //sets RESET PIN to high.
          }
        }
        else
        {
          LOG((DBG_ERROR "\nFailed to download Image, error code = %d\n", err_no));
        }

    }while(0);
   // ShowProgress(0,2,0,0);
    return err_no;
}

