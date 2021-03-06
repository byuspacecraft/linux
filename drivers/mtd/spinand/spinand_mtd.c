/*
spinand_mtd.c

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


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/spinand.h>
#include <linux/mtd/nand_ecc.h>

//#define DEBUG_PRINT_JACOBW_ADD // enable debug print statements added by jacob willis
//#define DEBUG_PRINT_JACOBW_ADD_READ_WRITE // enable debug print statements added by jacob willis

/**
 * spinand_get_device - [GENERIC] Get chip for selected access
 * @param mtd		MTD device structure
 * @param new_state	the state which is requested
 *
 * Get the device and lock it for exclusive access
 */
#define mu_spi_nand_driver_version "Beagle-MTD_01.00_Linux2.6.33_20100507"
static int spinand_get_device(struct mtd_info *mtd, int new_state)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif

	struct spinand_chip *this = mtd->priv;
	DECLARE_WAITQUEUE(wait, current);

	/*
	 * Grab the lock and see if the device is available
	 */
	while (1) {
		spin_lock(&this->chip_lock);
		if (this->state == FL_READY) {
			this->state = new_state;
			spin_unlock(&this->chip_lock);
			break;
		}
		if (new_state == FL_PM_SUSPENDED) {
			spin_unlock(&this->chip_lock);
			return (this->state == FL_PM_SUSPENDED) ? 0 : -EAGAIN;
		}
		set_current_state(TASK_UNINTERRUPTIBLE);
		add_wait_queue(&this->wq, &wait);
		spin_unlock(&this->chip_lock);
		schedule();
		remove_wait_queue(&this->wq, &wait);
	}
	return 0;
}

/**
 * spinand_release_device - [GENERIC] release chip
 * @param mtd		MTD device structure
 *
 * Deselect, release chip lock and wake up anyone waiting on the device
 */
static void spinand_release_device(struct mtd_info *mtd)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif
	struct spinand_chip *this = mtd->priv;

	/* Release the chip */
	spin_lock(&this->chip_lock);
	this->state = FL_READY;
	wake_up(&this->wq);
	spin_unlock(&this->chip_lock);
}

#ifdef CONFIG_MTD_SPINAND_SWECC 
static void spinand_calculate_ecc(struct mtd_info *mtd)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif
	int i; 
	int eccsize = 512;
	int eccbytes = 3;
	int eccsteps = 4;
	int ecctotal = 12;
	struct spinand_chip *chip = mtd->priv;
	struct spinand_info *info = chip->info;
	unsigned char *p = chip->buf;

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize)
		__nand_calculate_ecc(p, eccsize, &chip->ecc_calc[i]);

	for (i = 0; i < ecctotal; i++)
		chip->buf[info->page_main_size + info->ecclayout->eccpos[i]] = chip->ecc_calc[i];
}

static int spinand_correct_data(struct mtd_info *mtd)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif
	int i; 
	int eccsize = 512;
	int eccbytes = 3;
	int eccsteps = 4;
	int ecctotal = 12;
	struct spinand_chip *chip = mtd->priv;
	struct spinand_info *info = chip->info;
	unsigned char *p = chip->buf;
	int errcode = 0;

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize)
		__nand_calculate_ecc(p, eccsize, &chip->ecc_calc[i]);

	for (i = 0; i < ecctotal; i++)
		chip->ecc_code[i] = chip->buf[info->page_main_size + info->ecclayout->eccpos[i]];

	for (i = 0; eccsteps; eccsteps--, i += eccbytes, p += eccsize)
	{
		int stat;
		
		stat = __nand_correct_data(p, &chip->ecc_code[i], &chip->ecc_calc[i], eccsize);
		if (stat < 0)
		{
			errcode = -1;
		}
		else if (stat == 1)
		{
			errcode = 1;
		}
	}

	return errcode;
}
#endif

static int spinand_read_ops(struct mtd_info *mtd, loff_t from,
			  struct mtd_oob_ops *ops)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif
	struct spinand_chip *chip = mtd->priv;
	struct spi_device *spi_nand = chip->spi_nand;
	struct spinand_info *info = chip->info;
	int page_id, page_offset, page_num, oob_num;

	int count;
	int main_ok, main_left, main_offset;
	int oob_ok, oob_left;
	
	signed int retval;
	signed int errcode=0;

	if (!chip->buf)
		return -1;

    // use from to determine what page we start on
    // 2^(info->page_shift) = number of bytes on a page,
    // so that means from >> info->page_shift gives the page
    // number within the device
	page_id = from >> info->page_shift; 

	/* for main data */
	page_offset = from & info->page_mask; // position to start on the page is held in lower bits of from
#ifdef DEBUG_PRINT_JACOBW_ADD
    printk("\n\rSPINAND_MTD: spinand_read_ops from = 0x%llx, page_id = %d", from, page_id);
#endif
    // Determine which die we should start on
    u8 db_start = ((u8)(from >> info->die_select_shift)) & 0x01; // get the topmost bit of from    
    if(info->has_die_select){   // check if this device has die select
      chip->die_select(spi_nand, db_start); // set the appropriate die bit.
    } 
    
    // page_num = the total number of pages that are to be read
	page_num = (page_offset + ops->len + info->page_main_size -1 ) / info->page_main_size;

	/* for oob */
	oob_num = (ops->ooblen + info->ecclayout->oobavail -1) / info->ecclayout->oobavail;
    
    // count is the number of pages we have read from
	count = 0;

	main_left = ops->len; // amount of data left to be read
	main_ok = 0; // amount of data that has been read
	main_offset = page_offset; // starting point on the first page is assigned to main offset

	oob_left = ops->ooblen;
	oob_ok = 0;

	while (1)
	{
		if (count < page_num || count < oob_num) 
		{
			memset(chip->buf, 0, info->page_size); // assign 0s to chip->buf to clear it prior to reading
            // read page into chip->buf, page number determined by page_id+count
			retval = chip->read_page(spi_nand, info, page_id + count, 0, info->page_size, chip->buf); 
#ifdef DEBUG_PRINT_JACOBW_ADD
                // print raw data from chip->buf to terminal
            if(page_id+count < 2){
                printk("\n\rSPINAND_MTD: spinand_read_ops");
                printk("\n\r\tData read of page %d: length: %d", page_id+count, info->page_size);
                int i;
                printk("\n\r%.6d:\t ",0); // new line and tab out one
                for(i=0; i < info->page_size; i++){
                    if(i != 0 && i%32 == 0){
                        printk("\n\r%.6d:\t",i); // newline every 32 bytes
                    }
                    if(i !=0 && i%8 == 0){
                        printk(" ");
                    }
                    printk("%.2x ", chip->buf[i]);
                }
                printk("\n\r");
            }
#endif
            // read failed, return and report failure
			if (retval != 0)
			{
				errcode = -1;
				printk(KERN_INFO "spinand_read_ops: fail, page=%d!\n", page_id);

				return errcode;
			}
		}
		else // no more data to read from the device
		{
			break;
		}
	
		if (count < page_num && ops->datbuf) 
		{
			int size;
			
#ifdef CONFIG_MTD_SPINAND_SWECC
			retval = spinand_correct_data(mtd);
			if (retval == -1)
				printk(KERN_INFO "SWECC uncorrectable error! page=%x\n", page_id+count); 
			else if (retval == 1)
				printk(KERN_INFO "SWECC 1 bit error, corrected! page=%x\n", page_id+count); 

#endif
         // copy data from chip->buf into ops->datbuf
         // chip->buf contains full pages, so this is where we segment pages if only
         // a portion of the page was requested, or if the length doesn't fall on a 
         // page boundary


            // if we are on the last page, the size of the data to copy into ops->dat 
            // is main_left
			if ((main_offset + main_left) < info->page_main_size)
			{
				size = main_left;
			}
            // main_offset is 0, unless on the first page, so size should be the 
            // size of the page minus any offset.
			else
			{
				size = info->page_main_size - main_offset;
			}
			
            // copy the data from the most recent pass into datbuf
            // add main_ok to datbuf to make sure data is saved at its proper location
            // add main_offset to chip->buf to account for starting in the middle of the page
			memcpy (ops->datbuf + main_ok, chip->buf + main_offset, size);

			main_ok += size;
			main_left -= size; // we have read size data, so 
			main_offset = 0;

			ops->retlen = main_ok;
		}

		if (count < oob_num && ops->oobbuf && chip->oobbuf)
		{
			int size;
			int offset, len, temp;
		
			/* repack spare to oob */
			memset(chip->oobbuf, 0, info->ecclayout->oobavail);

			temp = 0;
			offset = info->ecclayout->oobfree[0].offset;
			len = info->ecclayout->oobfree[0].length;
			memcpy (chip->oobbuf + temp, chip->buf + info->page_main_size + offset, len);
	
			temp += len;
			offset = info->ecclayout->oobfree[1].offset;
			len = info->ecclayout->oobfree[1].length;
			memcpy (chip->oobbuf + temp, chip->buf + info->page_main_size + offset, len);
	
			temp += len;
			offset = info->ecclayout->oobfree[2].offset;
			len = info->ecclayout->oobfree[2].length;
			memcpy (chip->oobbuf + temp, chip->buf + info->page_main_size + offset, len);

			temp += len;
			offset = info->ecclayout->oobfree[3].offset;
			len = info->ecclayout->oobfree[3].length;
			memcpy (chip->oobbuf + temp, chip->buf + info->page_main_size + offset, len);

			/* copy oobbuf to ops oobbuf */
			if (oob_left < info->ecclayout->oobavail) 
			{
				size = oob_left;
			}
			else
			{
				size = info->ecclayout->oobavail;
			}

			memcpy (ops->oobbuf + oob_ok, chip->oobbuf, size);

			oob_ok += size;
			oob_left -= size;

			ops->oobretlen = oob_ok;
		}

		count++;
       
        // check that we haven't crossed the die boundary 
        u8 db_temp; // temp variable to store new die bit in
        // count+page_id is the next page we will be on
        if (db_start != (db_temp = ((count+page_id)>>(info->die_select_shift - info->page_shift))&0x01)){
            // crossed die boundary
            if(info->has_die_select){   // check if this device has die select
                chip->die_select(spi_nand, db_temp); // set the appropriate die bit.
            }                 
        }
	}
	return errcode;
}

static int spinand_write_ops(struct mtd_info *mtd, loff_t to,
			  struct mtd_oob_ops *ops)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif
	struct spinand_chip *chip = mtd->priv;
	struct spi_device *spi_nand = chip->spi_nand;
	struct spinand_info *info = chip->info;
	int page_id, page_offset, page_num, oob_num;

	int count;

	int main_ok, main_left, main_offset;
	int oob_ok, oob_left;
	
	signed int retval;
	signed int errcode=0;

	if (!chip->buf)
		return -1;

	page_id = to >> info->page_shift; // location of first page

	/* for main data */
	page_offset = to & info->page_mask;
	page_num = (page_offset + ops->len + info->page_main_size -1 ) / info->page_main_size;
#ifdef DEBUG_PRINT_JACOB_ADD
   printk("\n\rSPINAND_MTD: spinand_write_ops to = 0x%llx, page_id = %d", to, page_id);
#endif
    // Determine which die we should start on
    u8 db_start = ((u8)(to >> info->die_select_shift)) & 0x01; // get the topmost bit of to
    
    
    if(info->has_die_select){   // check if this device has die select
      chip->die_select(spi_nand, db_start); // set the appropriate die bit.
    } 

	/* for oob */
	oob_num = (ops->ooblen + info->ecclayout->oobavail -1) / info->ecclayout->oobavail;

	count = 0;

	main_left = ops->len;
	main_ok = 0;
	main_offset = page_offset;

	oob_left = ops->ooblen;
	oob_ok = 0;

	while (1)
	{

#ifdef DEBUG_PRINT_JACOBW_ADD
    printk("\n\rSPINAND_MTD: spinand_write_ops - page_num = %d, oob_num = %d, count = %d", page_num, oob_num, count);
#endif
		if (count < page_num || count < oob_num) 
		{
			memset(chip->buf, 0xFF, info->page_size); 
		}
		else
		{
			break; 
		}

		if (count < page_num && ops->datbuf) 
		{
			int size;

			if ((main_offset + main_left) < info->page_main_size)
			{
				size = main_left;
			}
			else
			{
				size = info->page_main_size - main_offset;
			}

			memcpy (chip->buf, ops->datbuf + main_ok, size);
			
			main_ok += size;
			main_left -= size;
			main_offset = 0;
			
#ifdef CONFIG_MTD_SPINAND_SWECC 
			spinand_calculate_ecc(mtd);
#endif
		}

		if (count < oob_num && ops->oobbuf && chip->oobbuf)
		{
			int size;
			int offset, len, temp;

			memset(chip->oobbuf, 0xFF, info->ecclayout->oobavail);

			if (oob_left < info->ecclayout->oobavail) 
			{
				size = oob_left;
			}
			else
			{
				size = info->ecclayout->oobavail;
			}

			memcpy (chip->oobbuf, ops->oobbuf + oob_ok, size);

			oob_ok += size;
			oob_left -= size;

			/* repack oob to spare */
			temp = 0;
			offset = info->ecclayout->oobfree[0].offset;
			len = info->ecclayout->oobfree[0].length;
			memcpy (chip->buf + info->page_main_size + offset, chip->oobbuf + temp, len);
		
			temp += len;
			offset = info->ecclayout->oobfree[1].offset;
			len = info->ecclayout->oobfree[1].length;
			memcpy (chip->buf + info->page_main_size + offset, chip->oobbuf + temp, len);

			temp += len;
			offset = info->ecclayout->oobfree[2].offset;
			len = info->ecclayout->oobfree[2].length;
			memcpy (chip->buf + info->page_main_size + offset, chip->oobbuf + temp, len);

			temp += len;
			offset = info->ecclayout->oobfree[3].offset;
			len = info->ecclayout->oobfree[3].length;
			memcpy (chip->buf + info->page_main_size + offset, chip->oobbuf + temp, len);
		
		}

		if (count < page_num || count < oob_num) 
		{

			retval = chip->program_page(spi_nand, info, page_id + count, 0, info->page_size, chip->buf);
			if (retval != 0)
			{
				errcode = -1;
				printk(KERN_INFO "spinand_write_ops: fail, page=%d!\n", page_id + count);

				return errcode;
			}
		}

		if (count < page_num && ops->datbuf) 
		{
			ops->retlen = main_ok;
		}

		if (count < oob_num && ops->oobbuf && chip->oobbuf)
		{
			ops->oobretlen = oob_ok;
		}

		count++;
        
        // check that if crossed the die boundary 
        u8 db_temp;
        if (db_start != (db_temp = ((count+page_id)>>(info->die_select_shift - info->page_shift))&0x01)){
            // crossed die boundary
            if(info->has_die_select){   // check if this device has die select
                chip->die_select(spi_nand, db_temp); // set the appropriate die bit.
            }                 
        }

	}

	return errcode;
}

static int spinand_read(struct mtd_info *mtd, loff_t from, size_t len,
	size_t *retlen, u_char *buf)
{
	struct mtd_oob_ops ops = {0};
	int ret;
	
	/* Do not allow reads past end of device */
	if ((from + len) > mtd->size)
		return -EINVAL;

	if (!len)
		return 0;

	spinand_get_device(mtd, FL_READING);

	ops.len = len;
	ops.datbuf = buf;

	ret = spinand_read_ops(mtd, from, &ops);
 
 	*retlen = ops.retlen;

	spinand_release_device(mtd);
    
    #ifdef DEBUG_PRINT_JACOBW_ADD_READ_WRITE
                // print raw data from chip->buf to terminal
                printk("\n\rSPINAND_MTD: spinand_read");
                printk("\n\r\tData read length: %d, from: %d", len, from);
                int i;
                printk("\n\r%.6d:\t ",0); // new line and tab out one
                for(i=0; i < len && i < 2048; i++){
                    if(i != 0 && i%32 == 0){
                        printk("\n\r%.6d:\t",i); // newline every 32 bytes
                    }
                    if(i !=0 && i%8 == 0){
                        printk(" ");
                    }
                    printk("%.2x ", buf[i]); // print out the data in buf
                }
                printk("\n\r");
#endif

	return ret;
}

static int spinand_write(struct mtd_info *mtd, loff_t to, size_t len,
	size_t *retlen, const u_char *buf)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif

#ifdef DEBUG_PRINT_JACOBW_ADD_READ_WRITE
                // print raw data from chip->buf to terminal
                printk("\n\rSPINAND_MTD: spinand_write");
                printk("\n\r\tData write length: %d", len);
                int i;
                printk("\n\r%.6d:\t ",0); // new line and tab out one
                for(i=0; i < len && i < 2048; i++){
                    if(i != 0 && i%32 == 0){
                        printk("\n\r%.6d:\t",i); // newline every 32 bytes
                    }
                    if(i !=0 && i%8 == 0){
                        printk(" ");
                    }
                    printk("%.2x ", buf[i]); // print out the data in buf
                }
                printk("\n\r");
#endif
	struct mtd_oob_ops ops = {0};
	int ret;
	
	/* Do not allow reads past end of device */
	if ((to + len) > mtd->size)
		return -EINVAL;
	if (!len)
		return 0;

	spinand_get_device(mtd, FL_WRITING);

	ops.len = len;
	ops.datbuf = (uint8_t *)buf;

	ret =  spinand_write_ops(mtd, to, &ops);

	*retlen = ops.retlen;

	spinand_release_device(mtd);

	return ret;
}

static int spinand_read_oob(struct mtd_info *mtd, loff_t from,
			  struct mtd_oob_ops *ops)
{
	int ret;

	spinand_get_device(mtd, FL_READING);

	ret = spinand_read_ops(mtd, from, ops);

	spinand_release_device(mtd);

	return ret;
}

static int spinand_write_oob(struct mtd_info *mtd, loff_t to,
			  struct mtd_oob_ops *ops)
{
	int ret;

	spinand_get_device(mtd, FL_WRITING);

	ret = spinand_write_ops(mtd, to, ops);
	
	spinand_release_device(mtd);
	
	return ret;
}

/**
 * spinand_erase - [MTD Interface] erase block(s)
 * @param mtd		MTD device structure
 * @param instr		erase instruction
 *
 * Erase one ore more blocks
 */
static int spinand_erase(struct mtd_info *mtd, struct erase_info *instr)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif
	struct spinand_chip *chip = mtd->priv;
	struct spi_device *spi_nand = chip->spi_nand;
	struct spinand_info *info = chip->info;
	u16 block_id, block_num, count;
	signed int retval=0;
	signed int errcode=0;

	DEBUG(MTD_DEBUG_LEVEL3, "spinand_erase: start = 0x%012llx, len = %llu\n",
	      (unsigned long long)instr->addr, (unsigned long long)instr->len);

	/* check address align on block boundary */
	if (instr->addr & (info->block_main_size - 1)) {
		DEBUG(MTD_DEBUG_LEVEL0, "spinand_erase: Unaligned address\n");
		return -EINVAL;
	}

	if (instr->len & (info->block_main_size - 1)) {
		DEBUG(MTD_DEBUG_LEVEL0, "spinand_erase: "
		      "Length not block aligned\n");
		return -EINVAL;
	}

	/* Do not allow erase past end of device */
	if ((instr->len + instr->addr) > info->usable_size) {
		DEBUG(MTD_DEBUG_LEVEL0, "spinand_erase: "
		      "Erase past end of device\n");
		return -EINVAL;
	}

	instr->fail_addr = MTD_FAIL_ADDR_UNKNOWN;	

	
	/* Grab the lock and see if the device is available */
	spinand_get_device(mtd, FL_ERASING);

	block_id = instr->addr >> info->block_shift;
	block_num = instr->len >> info->block_shift; 


    // Determine which die we should start on
    u8 db_start = ((u8)(instr->addr >> info->die_select_shift)) & 0x01; // get the topmost bit of addr    
    if(info->has_die_select){   // check if this device has die select
      chip->die_select(spi_nand, db_start); // set the appropriate die bit.
    } 


	count = 0; // number of blocks we have done
	
	while (count < block_num )
	{
		retval = chip->erase_block(spi_nand, info, block_id+count);

		if (retval!=0)
		{
			retval = chip->erase_block(spi_nand, info, block_id+count);
			if (retval!=0)
			{
				printk(KERN_INFO "spinand_erase: fail, block=%d!\n", block_id+count);
				errcode = -1;
			}
		}
		count++;
        // check that we haven't crossed the die boundary 
        u8 db_temp;
        if (db_start != (db_temp = ((count+block_id)>>(info->die_select_shift - info->block_shift))&0x01)){
            // crossed die boundary
            if(info->has_die_select){   // check if this device has die select
                chip->die_select(spi_nand, db_temp); // set the appropriate die bit.
            }                 
        }
	}
	
	if (errcode == 0)
	{
		instr->state = MTD_ERASE_DONE;
	}
	
	/* Deselect and wake up anyone waiting on the device */
	spinand_release_device(mtd);

	/* Do call back function */
	if(instr->callback)
		instr->callback(instr);
	
	return errcode;
}

/**
 * spinand_sync - [MTD Interface] sync
 * @param mtd		MTD device structure
 *
 * Sync is actually a wait for chip ready function
 */
static void spinand_sync(struct mtd_info *mtd)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif
	DEBUG(MTD_DEBUG_LEVEL3, "spinand_sync: called\n");

	/* Grab the lock and see if the device is available */
	spinand_get_device(mtd, FL_SYNCING);

	/* Release it and go back */
	spinand_release_device(mtd);
}

static int spinand_block_isbad(struct mtd_info *mtd, loff_t ofs)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif
	struct spinand_chip *chip = mtd->priv;
	struct spi_device *spi_nand = chip->spi_nand;
	struct spinand_info *info = chip->info;
	u16 block_id;
	u8 is_bad = 0x00;
	u8 ret = 0;

	spinand_get_device(mtd, FL_READING);

	block_id = ofs >> info->block_shift;

	chip->read_page(spi_nand, info, block_id*info->page_num_per_block, info->page_main_size, 1, &is_bad);

	if (is_bad != 0xFF)
	{
		ret =  1;	
	}

	spinand_release_device(mtd);

	return ret; 
}

/**
 * spinand_block_markbad - [MTD Interface] Mark bad block
 * @param mtd		MTD device structure
 * @param ofs       Bad block number
 */
static int spinand_block_markbad(struct mtd_info *mtd, loff_t ofs)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif
	struct spinand_chip *chip = mtd->priv;
	struct spi_device *spi_nand = chip->spi_nand;
	struct spinand_info *info = chip->info;
	u16 block_id;
	u8 is_bad = 0x00; 
	u8 ret = 0;

	spinand_get_device(mtd, FL_WRITING);

	block_id = ofs >> info->block_shift;

	chip->program_page(spi_nand, info, block_id*info->page_num_per_block, info->page_main_size, 1, &is_bad);

	spinand_release_device(mtd);

	return ret;
}


/**
 * spinand_suspend - [MTD Interface] Suspend the spinand flash
 * @param mtd		MTD device structure
 */
static int spinand_suspend(struct mtd_info *mtd)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif
	return spinand_get_device(mtd, FL_PM_SUSPENDED);
}

/**
 * spinand_resume - [MTD Interface] Resume the spinand flash
 * @param mtd		MTD device structure
 */
static void spinand_resume(struct mtd_info *mtd)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("SPINAND_MNT: %s\n\r", __func__);
    #endif
	struct spinand_chip *this = mtd->priv;

	if (this->state == FL_PM_SUSPENDED)
		spinand_release_device(mtd);
	else
		printk(KERN_ERR "resume() called for the chip which is not"
				"in suspended state\n");
}

/**
 * spinand_mtd - add MTD device with parameters
 * @param mtd		MTD device structure
 *
 * Add MTD device with parameters.
 */
int spinand_mtd(struct mtd_info *mtd)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif
	struct spinand_chip *chip = mtd->priv;
	struct spinand_info *info = chip->info;

	chip->state = FL_READY;
	init_waitqueue_head(&chip->wq);
	spin_lock_init(&chip->chip_lock);

	mtd->name = info->name;
	mtd->size = info->usable_size;
	mtd->erasesize = info->block_main_size;
	mtd->writesize = info->page_main_size;
	mtd->oobsize = info->ecclayout->oobavail;
	mtd->owner = THIS_MODULE;
	mtd->type = MTD_NANDFLASH;
	mtd->flags = MTD_CAP_NANDFLASH;

	mtd->ecclayout = info->ecclayout;

	mtd->erase = spinand_erase;
	mtd->point = NULL;
	mtd->unpoint = NULL;
	mtd->read = spinand_read;
	mtd->write = spinand_write;
	mtd->read_oob = spinand_read_oob;
	mtd->write_oob = spinand_write_oob;
	mtd->sync = spinand_sync;
	mtd->lock = NULL;
	mtd->unlock = NULL;
	mtd->suspend = spinand_suspend;
	mtd->resume = spinand_resume;
	mtd->block_isbad = spinand_block_isbad;
	mtd->block_markbad = spinand_block_markbad;
	
	return add_mtd_device(mtd) == 1 ? -ENODEV : 0;
}

void spinand_mtd_release(struct mtd_info *mtd)
{
    #ifdef DEBUG_PRINT_JACOBW_ADD
        printk("\n\rSPINAND_MTD: %s", __func__);
    #endif
	del_mtd_device(mtd);
}

EXPORT_SYMBOL_GPL(spinand_mtd);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Henry Pan <hspan@micron.com>");
MODULE_DESCRIPTION("SPI NAND driver code");
