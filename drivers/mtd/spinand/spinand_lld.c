/*
spinand_lld.c

Copyright (c) 2009-2010 Micron Technology, Inc.

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/math64.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spinand.h>

#include <linux/spi/spi.h>

//#define DEBUG_PRINT_JACOBW_ADD // enable debug print statements added by jacob willis

#define mu_spi_nand_driver_version "M2S-MTD_01.00_Linux2.6.33_20100507"
#define SPI_NAND_MICRON_DRIVER_KEY 0x1233567

/****************************************************************************/

/**
   OOB area specification layout:  Total 32 available free bytes.
   OOB = Out of Band
*/
static struct nand_ecclayout spinand_oob_64 = {
	.eccbytes = 24,
	.eccpos = {
		   1, 2, 3, 4, 5, 6,
		   17, 18, 19, 20, 21, 22,
		   33, 34, 35, 36, 37, 38,
		   49, 50, 51, 52, 53, 54, },
	.oobavail = 32,
	.oobfree = {
		{.offset = 8,
		 .length = 8},
		{.offset = 24,
		 .length = 8},
		{.offset = 40,
		 .length = 8},
		{.offset = 56,
		 .length = 8}, }
};

// Added by Jacob Willis from micron spi nand software solution
static struct nand_ecclayout spinand_oob_128 = {
	.eccbytes = 64,
	.eccpos = {
		64, 65, 66, 67, 68, 69, 70, 71,
		72, 73, 74, 75, 76, 77, 78, 79,
		80, 81, 82, 83, 84, 85, 86, 87,
		88, 89, 90, 91, 92, 93, 94, 95,
		96, 97, 98, 99, 100, 101, 102, 103,
		104, 105, 106, 107, 108, 109, 110, 111,
		112, 113, 114, 115, 116, 117, 118, 119,
		120, 121, 122, 123, 124, 125, 126, 127},
	.oobavail = 62,
	.oobfree = {
		{.offset = 2,
		 .length = 62}, }
};


/**
 * spinand_cmd - to process a command to send to the SPI Nand
 * 
 * Description:
 *    Set up the command buffer to send to the SPI controller.
 *    The command buffer has to initized to 0
 */
int spinand_cmd(struct spi_device *spi, struct spinand_cmd *cmd)
{

    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	int					ret;
	struct spi_message	message;
	struct spi_transfer		x[4];
	u8 dummy = 0xff;


	spi_message_init(&message);
	memset(x, 0, sizeof x);
	
	x[0].len = 1;
	x[0].tx_buf = &cmd->cmd;
	spi_message_add_tail(&x[0], &message);
	
	if (cmd->n_addr)
	{
		x[1].len = cmd->n_addr;
		x[1].tx_buf = cmd->addr;
		spi_message_add_tail(&x[1], &message);
	}

	if (cmd->n_dummy)
	{
		x[2].len = cmd->n_dummy;
		x[2].tx_buf = &dummy;
		spi_message_add_tail(&x[2], &message);		
	}

	if (cmd->n_tx)
	{
		x[3].len = cmd->n_tx;
		x[3].tx_buf = cmd->tx_buf;
		spi_message_add_tail(&x[3], &message);		
	}

	if (cmd->n_rx)
	{
		x[3].len = cmd->n_rx;
		x[3].rx_buf = cmd->rx_buf;
		spi_message_add_tail(&x[3], &message);	
	}
	
	ret = spi_sync(spi, &message);

	return ret; 
}


/**
 * spinand_die_select -
 * 
 * Description:
 *     Given the msb of the read/write address, determines and selects the appropriate die.
 *     
 * @spi_nand - pointer to a spi_device struct for the device we will send info to
 * @die_bit - the most significant bit of the read/write address
 *
 * @return - the status of the spinand_cmd transaction
 *
 * Example:
 *      u8 db = (uint8_t)from >> info->die_select_shift
 *      if(info->has_die_select){
 *         spinand_die_select(struct spi_device *spi_nand, db);
 *      }     
 */
static int spinand_die_select(struct spi_device *spi_nand, u8 die_bit){
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif

    // determine which die to select
    // if die_bit == 1, use die 1, if die_bit == 0, use die 0
    u8 ds_cmd = (die_bit==0) ? DIE_SEL_DIE0 : DIE_SEL_DIE1;

    //printk("\n\rSPINAND_LLD: spinand_die_select die_bit = %x, ds_cmd = %x", die_bit, ds_cmd);

    // set up cmd struct for die select command
	struct spinand_cmd cmd = {0};
	cmd.cmd = CMD_SET_FEATURES; // use set features (0x1F) command
    cmd.n_addr = 1; // 1 byte address after set features sent
    cmd.addr[0] = REG_DIE_SEL; // die select address (0xD0)
    cmd.n_tx = 1; // 1 byte to be written to address
    cmd.tx_buf = &ds_cmd; // byte to write

    return spinand_cmd(spi_nand, &cmd);    
}


/**
 * spinand_reset- send reset command "0xff" to the Nand device
 * 
 * Description:
 *    Reset the SPI Nand with the reset command 0xff
 */
static int spinand_reset(struct spi_device *spi_nand)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct spinand_cmd cmd = {0};

	cmd.cmd = CMD_RESET;

	return spinand_cmd(spi_nand, &cmd);
}
/**
 * spinand_read_id- Read SPI Nand ID
 * 
 * Description:
 *    Read ID: read two ID bytes from the SPI Nand device
 */
static int spinand_read_id(struct spi_device *spi_nand, u8 *id)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct spinand_cmd cmd = {0};
	ssize_t retval;

	
	cmd.cmd = CMD_READ_ID;
	cmd.n_dummy = 1;
	cmd.n_rx = 2;
	cmd.rx_buf = id;
	
	retval = spinand_cmd(spi_nand, &cmd);

	if (retval != 0) {
		dev_err(&spi_nand->dev, "error %d reading id\n",
				(int) retval);
		return retval;
	}
	
	return 0;	
}

/**
 * spinand_lock_block- send write register 0x1f command to the Nand device
 * 
 * Description:
 *    After power up, all the Nand blocks are locked.  This function allows
 *    one to unlock the blocks, and so it can be wriiten or erased.
 */
static int spinand_lock_block(struct spi_device *spi_nand, struct spinand_info *info, u8 lock)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct spinand_cmd cmd = {0};
	ssize_t retval;
	
	cmd.cmd = CMD_SET_FEATURES;
	cmd.n_addr = 1;
	cmd.addr[0] = REG_BLOCK_LOCK;
	cmd.n_tx = 1;
	cmd.tx_buf = &lock;
	
	retval = spinand_cmd(spi_nand, &cmd);

	if (retval != 0) {
		dev_err(&spi_nand->dev, "error %d lock block\n",
				(int) retval);
		return retval;
	}
	
	return 0;	
}

/**
 * spinand_read_status- send command 0xf to the SPI Nand status register
 * 
 * Description:
 *    After read, write, or erase, the Nand device is expected to set the busy status.
 *    This function is to allow reading the status of the command: read, write, and erase.
 *    Once the status turns to be ready, the other status bits also are valid status bits.
 */
static int spinand_read_status(struct spi_device *spi_nand, struct spinand_info *info, u8 *status)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct spinand_cmd cmd = {0};
	ssize_t retval;
	
	cmd.cmd = CMD_GET_FEATURES;
	cmd.n_addr = 1;
	cmd.addr[0] = REG_STATUS;
	cmd.n_rx = 1;
	cmd.rx_buf = status;
	
	retval = spinand_cmd(spi_nand, &cmd);

	if (retval != 0) {
		dev_err(&spi_nand->dev, "error %d reading status register\n",
				(int) retval);
		return retval;
	}

	return 0;
}

/**
 * spinand_get_otp- send command 0xf to read the SPI Nand OTP register
 * 
 * Description:
 *   There is one bit( bit 0x10 ) to set or to clear the internal ECC.
 *   Enable chip internal ECC, set the bit to 1
 *   Disable chip internal ECC, clear the bit to 0
 */
static int spinand_get_otp(struct spi_device *spi_nand, struct spinand_info *info, u8* otp)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct spinand_cmd cmd = {0};
	ssize_t retval;
	
	cmd.cmd = CMD_GET_FEATURES;
	cmd.n_addr = 1;
	cmd.addr[0] = REG_OTP;
	cmd.n_rx = 1;
	cmd.rx_buf = otp;
	
	retval = spinand_cmd(spi_nand, &cmd);

	if (retval != 0) {
		dev_err(&spi_nand->dev, "error %d get otp\n",
				(int) retval);
		return retval;
	}
	
	return 0;	
}


/**
 * spinand_set_otp- send command 0x1f to write the SPI Nand OTP register
 * 
 * Description:
 *   There is one bit( bit 0x10 ) to set or to clear the internal ECC.
 *   Enable chip internal ECC, set the bit to 1
 *   Disable chip internal ECC, clear the bit to 0
 */
static int spinand_set_otp(struct spi_device *spi_nand, struct spinand_info *info, u8* otp)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct spinand_cmd cmd = {0};
	ssize_t retval;
	
	cmd.cmd = CMD_SET_FEATURES;
	cmd.n_addr = 1;
	cmd.addr[0] = REG_OTP;
	cmd.n_tx = 1;
	cmd.tx_buf = otp;
	
	retval = spinand_cmd(spi_nand, &cmd);

	if (retval != 0) {
		dev_err(&spi_nand->dev, "error %d set otp\n",
				(int) retval);
		return retval;
	}
	
	return 0;
}

/**
 * sspinand_enable_ecc- send command 0x1f to write the SPI Nand OTP register
 * 
 * Description:
 *   There is one bit( bit 0x10 ) to set or to clear the internal ECC.
 *   Enable chip internal ECC, set the bit to 1
 *   Disable chip internal ECC, clear the bit to 0
 */
static int spinand_enable_ecc(struct spi_device *spi_nand, struct spinand_info *info)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	ssize_t retval;
	u8 otp = 0;
	
	retval = spinand_get_otp(spi_nand, info, &otp);

	if ((otp & OTP_ECC_MASK) == OTP_ECC_MASK)
	{
		return 0;
	}
	else
	{
		otp |= OTP_ECC_MASK;
		retval = spinand_set_otp(spi_nand, info, &otp);
		retval = spinand_get_otp(spi_nand, info, &otp);
		return retval;
	}
}

static int spinand_disable_ecc(struct spi_device *spi_nand, struct spinand_info *info)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	ssize_t retval;
	u8 otp = 0;
	
	retval = spinand_get_otp(spi_nand, info, &otp);


	if ((otp & OTP_ECC_MASK) == OTP_ECC_MASK)
	{
		otp &= ~OTP_ECC_MASK;
		retval = spinand_set_otp(spi_nand, info, &otp);
		retval = spinand_get_otp(spi_nand, info, &otp);
		return retval;
	}
	else
	{
		return 0;
	}
}

/**
 * spinand_write_enable- send command 0x06 to enable write or erase the Nand cells
 * 
 * Description:
 *   Before write and erase the Nand cells, the write enable has to be set.
 *   After the write or erase, the write enable bit is automatically cleared( status register bit 2 )
 *   Set the bit 2 of the status register has the same effect
 */
static int spinand_write_enable(struct spi_device *spi_nand, struct spinand_info *info)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct spinand_cmd cmd = {0};

	cmd.cmd = CMD_WR_ENABLE;

	return spinand_cmd(spi_nand, &cmd);
}

static int spinand_read_page_to_cache(struct spi_device *spi_nand, struct spinand_info *info, u16 page_id)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct spinand_cmd cmd = {0};
	u16 row;

	row = page_id;

	cmd.cmd = CMD_READ;
	cmd.n_addr = 3;
	cmd.addr[1] = (u8)((row&0xff00)>>8);
	cmd.addr[2] = (u8)(row&0x00ff);

	return spinand_cmd(spi_nand, &cmd);
}

/**
 * spinand_read_from_cache- send command 0x03 to read out the data from the cache register( 2176 bytes max )
 * 
 * Description:
 *   The read can specify 1 to 2112 bytes of data read at the coresponded locations.
 *   No tRd delay.
 */
static int spinand_read_from_cache(struct spi_device *spi_nand, struct spinand_info *info, u16 byte_id, u16 len, u8* rbuf)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct spinand_cmd cmd = {0};
	u16 column;

	column = byte_id;

	cmd.cmd = CMD_READ_RDM;
	cmd.n_addr = 2;
	cmd.addr[0] = (u8)((column&0xff00)>>8);
	cmd.addr[1] = (u8)(column&0x00ff);
	cmd.n_dummy = 1;
	cmd.n_rx = len;
	cmd.rx_buf = rbuf;
	
	return spinand_cmd(spi_nand, &cmd);	
}

/**
 * spinand_read_page-to read a page with:
 * @page_id: the physical page number
 * @offset:  the location from 0 to 2175
 * @len:     number of bytes to read
 * @rbuf:    read buffer to hold @len bytes
 *
 * Description:
 *   The read icludes two commands to the Nand: 0x13 and 0x03 commands
 *   Poll to read status to wait for tRD time.
 */
static int spinand_read_page(struct spi_device *spi_nand, struct spinand_info *info, u16 page_id, u16 offset, u16 len, u8* rbuf)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	ssize_t retval;
	u8 status = 0;
	
    // read the page into the cache
	retval = spinand_read_page_to_cache(spi_nand, info, page_id);

	while (1)
	{   // wait for the page to arrive in the cache
		retval = spinand_read_status(spi_nand, info, &status);
		if (retval<0) {
			dev_err(&spi_nand->dev, "error %d reading status register\n",
					(int) retval);
			return retval;
		}

		if ((status & STATUS_OIP_MASK) == STATUS_READY)
		{
			if ((status & STATUS_ECC_MASK) == STATUS_ECC_ERROR)
			{
				dev_err(&spi_nand->dev, "ecc error, page=%d\n", page_id);
				if (spi_nand == SPI_NAND_MICRON_DRIVER_KEY)
					printk(KERN_INFO "Error: reformat or erase your device. \n"); 
				else
				return -1;
			}
			break;
		}
	}
    
    // read the data out from the cache over spi
	retval = spinand_read_from_cache(spi_nand, info, offset, len, rbuf);

#ifdef DEBUG_PRINT_JACOBW_ADD
        // print raw data from rbuf to terminal
    if(page_id < 4){
        printk("\n\rSPINAND_LLD: spinand_read_data");
        printk("\n\r\tData read of page %d: length: %d", page_id, len);
        int i;
        printk("\n\r%.6d:\t ",0); // new line and tab out one
        for(i=0; i < len; i++){
            if(i != 0 && i%32 == 0){
                printk("\n\r%.6d:\t",i); // newline every 32 bytes
            }
            if(i !=0 && i%8 == 0){
                printk(" ");
            }
            printk("%.2x ", rbuf[i]);

        }
        printk("\n\r");
    }
#endif

	return 0;
		
}

/**
 * spinand_program_data_to_cache--to write a page to cache with:
 * @byte_id: the location to write to the cache
 * @len:     number of bytes to write
 * @rbuf:    read buffer to hold @len bytes
 *
 * Description:
 *   The write command used here is 0x84--indicating that the cache is not cleared first.
 *   Since it is writing the data to cache, there is no tPROG time.
 */
static int spinand_program_data_to_cache(struct spi_device *spi_nand, struct spinand_info *info, u16 byte_id, u16 len, u8* wbuf)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct spinand_cmd cmd = {0};
	u16 column;

	column = byte_id;

	cmd.cmd = CMD_PROG_PAGE_CLRCACHE;
	cmd.n_addr = 2;
	cmd.addr[0] = (u8)((column&0xff00)>>8);
	cmd.addr[1] = (u8)(column&0x00ff);
	cmd.n_tx = len;
	cmd.tx_buf = wbuf;

	return spinand_cmd(spi_nand, &cmd);
}

/**
 * spinand_program_execute--to write a page from cache to the Nand array with:
 * @page_id: the physical page location to write the page.
 *
 * Description:
 *   The write command used here is 0x10--indicating the cache is writing to the Nand array.
 *   Need to wait for tPROG time to finish the transaction.
 */
static int spinand_program_execute(struct spi_device *spi_nand, struct spinand_info *info, u16 page_id)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct spinand_cmd cmd = {0};
	u16 row;

	row = page_id;

	cmd.cmd = CMD_PROG_PAGE_EXC;
	cmd.n_addr = 3;
	cmd.addr[1] = (u8)((row&0xff00)>>8);
	cmd.addr[2] = (u8)(row&0x00ff);

	return spinand_cmd(spi_nand, &cmd);
}

/**
 * spinand_program_page--to write a page with:
 * @page_id: the physical page location to write the page.
 * @offset:  the location from the cache starting from 0 to 2111
 * @len:     the number of bytes to write 
 * @wbuf:    the buffer to hold the number of bytes
 *
 * Description:
 *   The commands used here are 0x06, 0x84, and 0x10--indicating that the write enable is first
 *   sent, the write cache command, and the write execute command
 *   Poll to wait for the tPROG time to finish the transaction.
 */
static int spinand_program_page(struct spi_device *spi_nand, struct spinand_info *info, u16 page_id, u16 offset, u16 len, u8* wbuf)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	ssize_t retval;
	u8 status = 0;

	retval = spinand_write_enable(spi_nand, info);
	
	retval = spinand_program_data_to_cache(spi_nand, info, offset, len, wbuf);

#ifdef DEBUG_PRINT_JACOBW_ADD
    if(page_id < 4){
        printk("\n\rSPINAND_LLD: spinand_program_data_to_cache");
        printk("\n\r\tData to be programmed to page %d: length: %d", page_id, len);
        int i;
        printk("\n\r%.6d:\t ",0); // new line and tab out one
        for(i=0; i < len; i++){
            if(i != 0 && i%32 == 0){
                printk("\n\r%.6d:\t",i); // newline every 32 bytes
            }
            if(i !=0 && i%8 == 0){
                printk(" ");
            }
            printk("%.2x ", wbuf[i]);

        }
        printk("\n\r");
    }
#endif

	retval = spinand_program_execute(spi_nand, info, page_id);

    // wait for the page program to complete, tPROG time
    // poll status register to determine when done
	while (1)
	{
		retval = spinand_read_status(spi_nand, info, &status);
		if (retval<0) {
			dev_err(&spi_nand->dev, "error %d reading status register\n",
					(int) retval);
			return retval;
		}

		if ((status & STATUS_OIP_MASK) == STATUS_READY)
		{

			if ((status & STATUS_P_FAIL_MASK) == STATUS_P_FAIL)
			{
				dev_err(&spi_nand->dev, "program error, page=%d\n", page_id);
				return -1;
			}
			else
				break;
		}
	}

	return 0;
}

/**
 * spinand_erase_block_erase--to erase a page with:
 * @block_id: the physical block location to erase.
 *
 * Description:
 *   The command used here is 0xd8--indicating an erase command to erase one block--64 pages
 *   Need to wait for tERS.
 */
static int spinand_erase_block_erase(struct spi_device *spi_nand, struct spinand_info *info, u16 block_id)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct spinand_cmd cmd = {0};
	u16 row;

	row = block_id << 6;
	cmd.cmd = CMD_ERASE_BLK;
	cmd.n_addr = 3;
	cmd.addr[1] = (u8)((row&0xff00)>>8);
	cmd.addr[2] = (u8)(row&0x00ff);

	return spinand_cmd(spi_nand, &cmd);	
}

/**
 * spinand_erase_block--to erase a page with:
 * @block_id: the physical block location to erase.
 *
 * Description:
 *   The commands used here are 0x06 and 0xd8--indicating an erase command to erase one block--64 pages
 *   It will first to enable the write enable bit ( 0x06 command ), and then send the 0xd8 erase command
 *   Poll to wait for the tERS time to complete the tranaction.
 */
static int spinand_erase_block(struct spi_device *spi_nand, struct spinand_info *info, u16 block_id)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	ssize_t retval;
	u8 status= 0;

	retval = spinand_write_enable(spi_nand, info);
	
	retval = spinand_erase_block_erase(spi_nand, info, block_id);

	while (1)
	{
		retval = spinand_read_status(spi_nand, info, &status);
		if (retval<0) {
			dev_err(&spi_nand->dev, "error %d reading status register\n",
					(int) retval);
			return retval;
		}

		if ((status & STATUS_OIP_MASK) == STATUS_READY)
		{
			if ((status & STATUS_E_FAIL_MASK) == STATUS_E_FAIL)
			{
				dev_err(&spi_nand->dev, "erase error, block=%d\n", block_id);
				return -1;
			}
			else
				break;
		}
	}

	return 0;
}

/*
* spinand_get_info: get NAND info, from read id or const value 

 * Description:
 *   To set up the device parameters.
 */
static int spinand_get_info(struct spi_device *spi_nand, struct spinand_info *info, u8* id)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	if (id[0]==0x2C && (id[1]==0x11 || id[1]==0x12 || id[1]==0x13 || id[1] == 0x36))
	{
		/* FIX SIZES AND SUCH HERE: TODO */
		info->mid = id[0];
		info->did = id[1];
		info->name = "MT29F4G01ADAGDXX";
		info->nand_size = (4096 * 64 * 2176);
		info->usable_size = (4096 * 64 * 2048);
		info->block_size = (2176*64);
		info->block_main_size = (2048*64);
		info->block_num_per_chip = 4096;
		info->page_size = 2176;
		info->page_main_size = 2048;
		info->page_spare_size = 128;
		info->page_num_per_block = 64;
        
		info->block_shift = 17;     // used to get the block number on the device
		info->block_mask = 0x1ffff; // used to get the lower bits (page number and position on page)

		info->page_shift = 11;     // 2^(info->page_shift) = number of bytes on a page, used to get a page number in the device
		info->page_mask = 0x7ff;   // 11 ones, used to get the position on a page 
        
        info->die_select_shift = 28; // shift down this far to determine if in the upper or lower address space
        info->has_die_select = 1; // true if it has die select
		
		info->ecclayout = &spinand_oob_128;
	}	
	return 0;
}

/**
 * spinand_probe - [spinand Interface] 
* @spi_nand: registered device driver.
 *
 * Description:
 *   To set up the device driver parameters to make the device available.
 */
static int __devinit spinand_probe(struct spi_device *spi_nand)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	ssize_t retval;
	struct mtd_info *mtd;
	struct spinand_chip *chip; 
	struct spinand_info *info;
	u8 id[2]= {0};
	
	retval = spinand_reset(spi_nand);
	retval = spinand_reset(spi_nand);
	retval = spinand_read_id(spi_nand, (u8*)&id);
	if (id[0]==0 && id[1]==0)
	{
		printk(KERN_INFO "SPINAND: read id error! 0x%02x, 0x%02x!\n", id[0], id[1]); 
		return 0;
	}

	info  = kzalloc(sizeof(struct spinand_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	
	retval = spinand_get_info(spi_nand, info, (u8*)&id);
	printk(KERN_INFO "%s\n", mu_spi_nand_driver_version);
	retval = spinand_lock_block(spi_nand, info, BL_ALL_UNLOCKED);

#ifdef CONFIG_MTD_SPINAND_ONDIEECC
	retval = spinand_enable_ecc(spi_nand, info);
#else
	retval = spinand_disable_ecc(spi_nand, info);
#endif

	chip  = kzalloc(sizeof(struct spinand_chip), GFP_KERNEL);
	printk(KERN_INFO "0x%p",chip);
	if (!chip)
		return -ENOMEM;

	chip->spi_nand = spi_nand;
	chip->info = info;
	chip->reset = spinand_reset;
	chip->read_id = spinand_read_id;
	chip->read_page = spinand_read_page;
	chip->program_page = spinand_program_page;
	chip->erase_block = spinand_erase_block;
    chip->die_select = spinand_die_select; 

	chip->buf = kzalloc(info->page_size, GFP_KERNEL);
	if (!chip->buf)
		return -ENOMEM;

	chip->oobbuf = kzalloc(info->ecclayout->oobavail, GFP_KERNEL);
	if (!chip->oobbuf)
		return -ENOMEM;

	mtd = kzalloc(sizeof(struct mtd_info), GFP_KERNEL);
	if (!mtd)
		return -ENOMEM;

	dev_set_drvdata(&spi_nand->dev, mtd);
	
	mtd->priv = chip;

	retval = spinand_mtd(mtd);
	
	return retval;
}

/**
 * __devexit spinand_remove--Remove the device driver
 * @spi: the spi device.
 *
 * Description:
 *   To remove the device driver parameters and free up allocated memories.
 */
static int __devexit spinand_remove(struct spi_device *spi)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	struct mtd_info *mtd;
	struct spinand_chip *chip; 

	DEBUG(MTD_DEBUG_LEVEL1, "%s: remove\n", dev_name(&spi->dev));

	mtd = dev_get_drvdata(&spi->dev);
	
	spinand_mtd_release(mtd);

	chip = mtd->priv;
	
	kfree(chip->info);
	kfree(chip->buf);
	kfree(chip->oobbuf);
	kfree(chip);
	kfree(mtd);
	
	return 0;
}

/**
 * Device name structure description
*/
static struct spi_driver spinand_driver = {

	.driver = {
		.name		= "spi_nand",
		.bus		= &spi_bus_type,
		.owner		= THIS_MODULE,
	},

	.probe		= spinand_probe,
	.remove		= __devexit_p(spinand_remove),
};

/**
 * Device driver registration
*/
static int __init spinand_init(void)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	return spi_register_driver(&spinand_driver);
}

/**
 * unregister Device driver.
*/
static void __exit spinand_exit(void)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_LLD: %s", __func__);
    #endif
	spi_unregister_driver(&spinand_driver);
}

module_init(spinand_init);
module_exit(spinand_exit);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Henry Pan <hspan@micron.com>");
MODULE_DESCRIPTION("SPI NAND driver code");
