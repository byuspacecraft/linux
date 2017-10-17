/*
 * Micron MT29F MTD driver for lightweight SPI framework
 *
 * Largely derived from mtd_mt29f.c:
 *  Copyright (C) 2010 ZIV
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/err.h>

#include <linux/spi/spi.h>
#include <linux/spi/flash.h>

#include <linux/mtd/mtd.h>
//#include <asm/nand.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>

//#include <asm/bfin5xx_spi.h>

#define DRV_NAME "mt29f"
//#define DEBUG_MT29F
//#define DEBUG_MT29F_EXAMINE

/* newer chips report JEDEC manufacturer and device IDs; chip
 * serial number and OTP bits; and per-sector writeprotect.
 */
#define OP_BLOCK_ERASE              0xD8
#define OP_GET_FEATURE              0x0F
#define OP_PAGE_READ                0x13
#define OP_PROGRAM_EXECUTE          0x10
#define OP_PROGRAM_LOAD             0x02
#define OP_PROGRAM_LOAD_RANDOM_DATA 0x84
#define OP_READ_FROM_CACHE1         0x03
#define OP_READ_FROM_CACHE2         0x0B
#define OP_READ_ID                 0x9F
#define OP_RESET                    0xFF
#define OP_SET_FEATURE              0x1F
#define OP_WRITE_DISABLE            0x04
#define OP_WRITE_ENABLE             0x06

#define REG_FEATURE_BLOCKLOCK      0xA0
#define REG_FEATURE_OTP            0xB0
#define REG_FEATURE_STATUS         0xC0

#define SPI_MAX_TRANSFER max(32,SMP_CACHE_BYTES)

struct mt29f_flash_info {
  char     *name;

  /* JEDEC id has a high byte of zero plus three data bytes:
   * the manufacturer id, then a two byte device id.
   */
  uint32_t jedec_id;

  /* The size listed here is what works with OP_ERASE_PAGE. */
  uint16_t nblocks;
  uint32_t npages;
  uint32_t writesize;
  uint32_t size;
};

struct mt29f_info {
  struct mt29f_flash_info *flash_info;
  struct spi_device *spi;
  struct flash_platform_data *platform;

  struct nand_hw_control  controller;
  struct mtd_info         mtd;
  struct nand_chip     chip;
  uint8_t auxbyte;
  uint8_t useauxbyte;
  uint8_t lastcommand;
  uint8_t status;
  uint16_t pageoffset;
  uint16_t byteoffset;
  uint8_t aleoffset;
  struct spi_message   message;
  struct spi_transfer  x[2];
};

static int mt29f_examine_features(struct spi_device *spi);

static struct mt29f_info *mtd_to_mt29f_info(struct mtd_info *mtd)
{
  return container_of(mtd, struct mt29f_info, mtd);
}

static void mt29f_error(struct mt29f_info *info, int tmp, int cmd)
{
  if (tmp < 0) {
     printk(KERN_ERR DRV_NAME "MT29F: error %d reading spi in command %02xn",
        tmp, cmd);
     return;
  }
}

static int mt29f_reset(struct mt29f_info *info)
{
  uint8_t dummy;
  uint8_t code = OP_RESET;
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " NAND_CMD_RESET\n");
  #endif
  return spi_write_then_read(info->spi, &code, 1, &dummy, 1);
}

static int mt29f_none(struct mt29f_info *info)
{
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " NAND_CMD_NONE\n");
  #ifdef DEBUG_MT29F_EXAMINE
  mt29f_examine_features(info->spi);
  #endif
  #endif
  return 0;
}

static int mt29f_read0(struct mt29f_info *info)
{
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " NAND_CMD_READ0\n");
  #endif
  return 0;
}

static int mt29f_get_features(struct spi_device *spi, uint8_t reg, 
           uint8_t *val)
{
  uint8_t code[2];
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " mt29f_get_features\n");
  #endif
  code[0] = OP_GET_FEATURE;
  code[1] = reg;
  return spi_write_then_read(spi, code, 2, val, 1);
}

static int mt29f_examine_features(struct spi_device *spi)
{
  uint8_t code[2];
  uint8_t val;
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " mt29f_examine_features\n");
  #endif
  code[0] = OP_GET_FEATURE;
  code[1] = REG_FEATURE_BLOCKLOCK;
  spi_write_then_read(spi, code, 2, &val, 1);
  printk(KERN_NOTICE DRV_NAME " REG_FEATURE_BLOCKLOCK %02x\n", val);

  code[1] = REG_FEATURE_OTP;
  spi_write_then_read(spi, code, 2, &val, 1);
  printk(KERN_NOTICE DRV_NAME " REG_FEATURE_OTP %02x\n", val);

  code[1] = REG_FEATURE_STATUS;
  spi_write_then_read(spi, code, 2, &val, 1);
  printk(KERN_NOTICE DRV_NAME " REG_FEATURE_STATUS %02x\n", val);

  return 0;
}

static int mt29f_set_features(struct spi_device *spi, uint8_t reg, 
           uint8_t val)
{
  uint8_t code[3];
  uint8_t dummy;
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " mt29f_set_features\n");
  #endif
  code[0] = OP_SET_FEATURE;
  code[1] = reg;
  code[2] = val;
  return spi_write_then_read(spi, code, 3, &dummy, 0);
}

static int mt29f_read_status(struct mt29f_info *info)
{
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " NAND_CMD_STATUS\n");
  #endif
  uint8_t aux;
  uint8_t returnvalue = NAND_STATUS_WP;
  int tmp = mt29f_get_features(info->spi, REG_FEATURE_STATUS, &aux);
  if (tmp < 0)
     return tmp;

  if (!(aux & 0x01))
     returnvalue |= NAND_STATUS_READY;

  if (aux & 0x2c) {
     #ifdef DEBUG_MT29F
     printk(KERN_ERR DRV_NAME " NAND_STATUS_FAIL %02x Page: %04x\n", aux,
info->pageoffset);
     #endif
     returnvalue |= NAND_STATUS_FAIL;
  }

  info->auxbyte = returnvalue;
  info->useauxbyte = 1;

  return returnvalue;
}

static int mt29f_write_enable(struct mt29f_info *info)
{
  uint8_t dummy;
  uint8_t code;
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " mt29f_write_enable\n");
  #endif
  code = OP_WRITE_ENABLE;
  return spi_write_then_read(info->spi, &code, 1, &dummy, 0);
}

static int mt29f_eraseblock(struct mt29f_info *info)
{
  int tmp;
  uint8_t dummy;
  uint8_t code[4];
  int i = 0;
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " NAND_CMD_ERASE2\n");
  printk(KERN_NOTICE DRV_NAME " mt29f_eraseblock. pageoffset %04x\n", 
info->pageoffset);
  #endif

  tmp = mt29f_write_enable(info);
  if (tmp < 0)
     return tmp;

  uint32_t aux = info->pageoffset /*<< 2*/;
  
  code[0] = OP_BLOCK_ERASE;
  code[1] = (aux >> 16) & 0xff;
  code[2] = (aux >> 8) & 0xff;
  code[3] = aux & 0xff;
  //printk(KERN_NOTICE DRV_NAME " MAC %02x %02x %02x\n", code[1],code[2], code[3]);
  tmp = spi_write_then_read(info->spi, code, 4, &dummy, 0);
  if (tmp < 0)
     return tmp;
  
  for (i = 0; i < 1000; i++) {
     if (mt29f_get_features(info->spi, REG_FEATURE_STATUS, &dummy) < 0)
        return -1;
     if (!(dummy & 0x01))
        break;
  }
  info->useauxbyte = 0;
  if (i == 1000) {
     #ifdef DEBUG_MT29F
     printk(KERN_NOTICE DRV_NAME " not read in 1000 cycles\n");
     #endif
     return -1;
  }
  return 0;
}

static int mt29f_readstart(struct mt29f_info *info)
{
  int tmp;
  uint8_t dummy;
  uint8_t code[4];
  int i = 0;
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " NAND_CMD_READSTART\n");
  printk(KERN_NOTICE DRV_NAME " mt29f_readstart. pageoffset %04x. \
byteoffset %04x\n", info->pageoffset, info->byteoffset);
  #endif
  code[0] = OP_PAGE_READ;
  code[1] = 0x00;
  code[2] = info->pageoffset >> 8;
  code[3] = info->pageoffset & 0xff;
  tmp = spi_write_then_read(info->spi, code, 4, &dummy, 0);
  if (tmp < 0)
     return tmp;
  
  for (i = 0; i < 1000; i++) {
     if (mt29f_get_features(info->spi, REG_FEATURE_STATUS, &dummy) < 0)
        return -1;
     if (!(dummy & 0x01))
        break;
  }
  info->useauxbyte = 0;
  if (i == 1000) {
     #ifdef DEBUG_MT29F
     printk(KERN_NOTICE DRV_NAME " not read in 1000 cycles\n");
     #endif
     return -1;
  }
  return 0;
}

static int mt29f_disable_hwecc(struct mt29f_info *info)
{
  uint8_t val;
  int tmp;
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " mt29f_disable_hwecc\n");
  #endif
  tmp = mt29f_get_features(info->spi, REG_FEATURE_OTP, &val);
  if (tmp < 0)
     return tmp;

  return mt29f_set_features(info->spi, REG_FEATURE_OTP, val & 0xef);
}

static int mt29f_unlockall(struct mt29f_info *info)
{
  uint8_t val;
  int tmp;
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " mt29f_unlockall\n");
  #endif
  tmp = mt29f_get_features(info->spi, REG_FEATURE_BLOCKLOCK, &val);
  if (tmp < 0)
     return tmp;

  return mt29f_set_features(info->spi, REG_FEATURE_BLOCKLOCK, 0);
}

static int mt29f_program_load(struct mt29f_info *info)
{
  int tmp;
  uint8_t dummy;
  uint8_t code[3];
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " mt29f_program_load\n");
  #endif
  code[0] = OP_PROGRAM_LOAD;
  code[1] = 0x00;
  code[2] = 0x00;
  tmp = spi_write_then_read(info->spi, code, 3, &dummy, 0);
  if (tmp < 0) {
//     printk(KERN_ERR DRV_NAME "%s: error %d reading JEDEC ID\n", 
//        info->spi->dev.bus_id, tmp);
      printk(KERN_ERR DRV_NAME "MT29F: error %d reading JEDEC ID\n",
            tmp);
     return -1;
  }
  return 0;
}

static int mt29f_program_execute(struct mt29f_info *info)
{
  uint8_t dummy;
  uint8_t code[4];
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " NAND_CMD_PAGEPROG\n");
  #endif
  code[0] = OP_PROGRAM_EXECUTE;
  code[1] = 0x00;
  code[2] = info->pageoffset >> 8;
  code[3] = info->pageoffset & 0xff;
  return spi_write_then_read(info->spi, code, 4, &dummy, 0);
}

static int mt29f_nand_calculate_ecc(struct mtd_info *mtd,
     const u_char *dat, u_char *ecc_code)
{
  return 0;
}

static int mt29f_nand_correct_data(struct mtd_info *mtd, u_char *dat,
              u_char *read_ecc, u_char *calc_ecc)
{
  return 0;
}

static void mt29f_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
  return;
}

static void mt29f_read_buf(struct mtd_info *mtd, uint8_t *buf, int len)
{
  struct mt29f_info *info = mtd_to_mt29f_info(mtd);
  int status;
  uint8_t code[4];
  code[0] = OP_READ_FROM_CACHE1;
  code[1] = info->byteoffset >> 8;
  code[2] = info->byteoffset & 0xff;
  code[3] = 0x00;
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " mt29f_read_buf %04x\n", len);
  #endif

  spi_message_init(&info->message);
  memset(info->x, 0, sizeof info->x);
  info->x[0].len = 4;
  spi_message_add_tail(&info->x[0], &info->message);
  info->x[1].len = len;
  spi_message_add_tail(&info->x[1], &info->message);

  info->x[0].tx_buf = code;
  info->x[1].rx_buf = buf;

  /* do the i/o */
  status = spi_sync(info->spi, &info->message);
  if (status != 0) {
     #ifdef DEBUG_MT29F
     printk(KERN_ERR DRV_NAME " mt29f_write_buf error\n");
     #endif
  }

  return;
}

static void mt29f_write_buf(struct mtd_info *mtd,
           const uint8_t *buf, int len)
{
  int status;
  uint8_t code[3];
  code[0] = OP_PROGRAM_LOAD;
  code[1] = 0x00;
  code[2] = 0x00;
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " mt29f_write_buf %04x\n", len);
  #endif
  if (len == 0x40)
     return;

  struct mt29f_info *info = mtd_to_mt29f_info(mtd);

  if (mt29f_write_enable(info) < 0)
     return;

  spi_message_init(&info->message);
  memset(info->x, 0, sizeof info->x);
  info->x[0].len = 3;
  spi_message_add_tail(&info->x[0], &info->message);
  info->x[1].len = len;
  spi_message_add_tail(&info->x[1], &info->message);

  info->x[0].tx_buf = code;
  info->x[1].tx_buf = buf;

  /* do the i/o */
  status = spi_sync(info->spi, &info->message);
  if (status != 0) {
     #ifdef DEBUG_MT29F
     printk(KERN_ERR DRV_NAME " mt29f_write_buf error\n");
     #endif
  }
  return;
}

static uint8_t mt29f_read_byte(struct mtd_info *mtd)
{
  int tmp;
  uint8_t useaux = 0;
  uint8_t  read;
  struct mt29f_info *info = mtd_to_mt29f_info(mtd);
  if (info->useauxbyte) {
     useaux = 1;
     if (info->lastcommand == NAND_CMD_READID) {
        if (info->useauxbyte == 1) {
           info->useauxbyte++;
           read = 0x2c;
        } else {
           info->useauxbyte = 0;
           read = 0x12;
        }
     } else {
        info->useauxbyte = 0;
        read = info->auxbyte;
     }
  } else 
     mt29f_read_buf(mtd, &read, 1);
  
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " mt29f_read_byte %02x %0d %02x\n", read,
useaux, info->lastcommand);
  #endif
  return read;
}

static void mt29f_hwcontrol(struct mtd_info *mtd, int cmd,
              unsigned int ctrl)
{
  int tmp = 0;
  uint8_t code;
  struct mt29f_info *info = mtd_to_mt29f_info(mtd);

  // ALE
  if (ctrl & NAND_ALE) {
     #ifdef DEBUG_MT29F
     printk(KERN_NOTICE DRV_NAME " NAND_ALE %02x %02x %02x %d\n", cmd,
ctrl,
info->lastcommand, info->aleoffset);
     #endif
     switch (info->lastcommand) {
        case NAND_CMD_READID:
           // Do not do anything
           break;
        case NAND_CMD_READ0:
        case NAND_CMD_SEQIN:
           if (info->aleoffset == 0) {
              info->byteoffset = cmd;
           } else if (info->aleoffset == 2) {
              info->pageoffset = cmd;
           }
           info->aleoffset++;
           break;
        case NAND_CMD_ERASE1:
           if (info->aleoffset == 0)
              info->pageoffset = cmd;
           info->aleoffset++;
        default:
           break;
     }
     return;
  }
  // NAND_CMD_NONE produces strange effects as it appears "inside"
  // other commands
  if (cmd == NAND_CMD_NONE) {
     tmp = mt29f_none(info);
     mt29f_error(info, tmp, cmd);
     return;
  }

  info->auxbyte = 0;
  info->useauxbyte = 0;
  info->aleoffset = 0;
  // CLE
  switch (cmd) {
     case NAND_CMD_READID:
        #ifdef DEBUG_MT29F
        printk(KERN_NOTICE DRV_NAME " NAND_CMD_READID\n");
        #endif
        info->useauxbyte = 1;
        // One detected, we unlock all blocks
        mt29f_unlockall(info);
        //mt29f_disable_hwecc(info);
        break;
     case NAND_CMD_RESET:
        tmp = mt29f_reset(info);
        udelay(1000);
        break;
     case NAND_CMD_NONE:
        tmp = mt29f_none(info);
        break;
     case NAND_CMD_READ0:
        tmp = mt29f_read0(info);
        break;
     case NAND_CMD_READSTART:
        tmp = mt29f_readstart(info);
        break;
     case NAND_CMD_STATUS:
        tmp = mt29f_read_status(info);
        // We should adapt the status bits to the NAND standard.
        /*info->auxbuffer[info->auxbufferwpos] = tmp;
        info->auxbufferwpos = (info->auxbufferwpos + 1) & AUXBUFFERMASK;
        tmp = 0;*/
        break;
     case NAND_CMD_SEQIN:
        /* Directly done in write_buf */
        break;
     case NAND_CMD_ERASE1:
        /* Just store page */
        break;
     case NAND_CMD_ERASE2:
        tmp = mt29f_eraseblock(info);
        break;
     case NAND_CMD_PAGEPROG:
        tmp = mt29f_program_execute(info);
        break;
     default:
        #ifdef DEBUG_MT29F
        printk(KERN_NOTICE DRV_NAME " Unknown command %02x, control %02x
\n", 
cmd, ctrl);
        #endif
        break;
  }
  mt29f_error(info, tmp, cmd);
  info->lastcommand = cmd;
}

// The nand_wait from nand_base.c does a read_byte after dev_ready that
// does not suit us.
static int mt29f_nand_wait(struct mtd_info *mtd, struct nand_chip
*chip)
{
  unsigned long timeo = jiffies;
  int status, state = chip->state;

  if (state == FL_ERASING)
     timeo += (HZ * 400) / 1000;
  else
     timeo += (HZ * 20) / 1000;

  /* Apply this short delay always to ensure that we do wait tWB in
   * any case on any machine. */
  ndelay(100);

  if ((state == FL_ERASING) && (chip->options & NAND_IS_AND))
     chip->cmdfunc(mtd, NAND_CMD_STATUS_MULTI, -1, -1);
  else
     chip->cmdfunc(mtd, NAND_CMD_STATUS, -1, -1);

  while (time_before(jiffies, timeo)) {
     status = chip->dev_ready(mtd);
     if (status)
        break;
     cond_resched();
  }

  return status;
}

static int mt29f_devready(struct mtd_info *mtd)
{
  uint8_t val;
  #ifdef DEBUG_MT29F
  printk(KERN_NOTICE DRV_NAME " mt29f_devready\n");
  #endif
  struct mt29f_info *info = mtd_to_mt29f_info(mtd);

  val = mt29f_read_status(info);
  // Workaround
  if (info->lastcommand != NAND_CMD_READID)
     info->useauxbyte = 0;
  return val;
}

/* ......................................................................... */

/*
 * Device management interface
 */
static int __devinit mt29f_add_partition(struct mt29f_info *info)
{
  struct mtd_info *mtd = &info->mtd;

#ifdef CONFIG_MTD_PARTITIONS
  struct mtd_partition *parts = info->platform->parts;
  int nr = info->platform->nr_parts;

  return add_mtd_partitions(mtd, parts, nr);
#else
  return add_mtd_device(mtd);
#endif
}

static struct mt29f_flash_info __devinitdata mt29f_data [] = {

  { "MT29F",  0x2C11, 1024, 1024*64, 2048, 0x8000000},
};

static struct mt29f_flash_info *__devinit jedec_probe(struct spi_device
*spi)
{
  int         tmp;
  uint8_t        code = OP_READ_ID;
  uint8_t        id[3];
  uint32_t    jedec;
  struct mt29f_flash_info *flash_info;

  /* JEDEC also defines an optional "extended device information"
   * string for after vendor-specific data, after the three bytes
   * we use here.  Supporting some chips might require using it.
   *
   * If the vendor ID isn't Micron's (0x2C), assume this call failed.
   */
  tmp = spi_write_then_read(spi, &code, 1, id, 3);
  if (tmp < 0) {
     printk(KERN_ERR DRV_NAME "%s: error %d reading JEDEC ID\n",
        /*spi->dev.bus_id*/"MT29F", tmp);
     return ERR_PTR(tmp);
  }
  /* First byte is a dummy byte */
  if (id[1] != 0x2c)
     return NULL;

  jedec = id[1];
  jedec = jedec << 8;
  if (id[2] == 0x11 || id[2] == 0x12 || id[2] == 0x13)
     jedec |= 0x11;

  for (tmp = 0, flash_info = mt29f_data; tmp < ARRAY_SIZE(mt29f_data);
        tmp++, flash_info++) {
     if (flash_info->jedec_id == jedec) {
        return flash_info;
     }
  }

  dev_err(&spi->dev, "unrecognized JEDEC id %06x\n", jedec);
  return ERR_PTR(-ENODEV);
}

/*
 * Detect and initialize mt29f device, using JEDEC IDs on newer chips
 * or else the ID code embedded in the status bits:
 *
 *   Device      Density         ID code   #Blocks #Pages  PageSize
 *   MT29F       1Gbit   (128M)  (0x12)    1024    1024*64 2048
 */
static int __devinit mt29f_probe(struct spi_device *spi)
{
  struct mt29f_flash_info *flash_info;
  struct flash_platform_data *data;
  unsigned       i;
  struct mt29f_info *info = NULL;
  struct nand_chip *chip = NULL;

  /* Platform data helps sort out which chip type we have, as
   * well as how this board partitions it.  If we don't have
   * a chip ID, try the JEDEC id commands; they'll work for most
   * newer chips, even if we don't recognize the particular chip.
   */
  data = spi->dev.platform_data;
  if (data && data->type) {
     for (i = 0, flash_info = mt29f_data; i < ARRAY_SIZE(mt29f_data);
        i++, flash_info++) {
        if (strcmp(data->type, flash_info->name) == 0)
           break;
     }

     /* unrecognized chip? */
     if (i == ARRAY_SIZE(mt29f_data)) {
        printk(KERN_ERR DRV_NAME "%s: unrecognized id %s\n",
              /*spi->dev.bus_id*/"MT29F", data->type);
        flash_info = NULL;
     /* recognized; is that chip really what's there? */
     } else if (flash_info->jedec_id) {
        struct mt29f_flash_info *chip = jedec_probe(spi);

        if (!chip || chip != flash_info) {
           dev_warn(&spi->dev, "found %s, expected %s\n",
              chip ? chip->name : "UNKNOWN",
              flash_info->name);
           flash_info = ERR_PTR(-ENODEV);
        }
     }
  } else {
     flash_info = jedec_probe(spi);
  }

  if (IS_ERR(flash_info)) {
     return PTR_ERR(flash_info);
  }

  info = kzalloc(sizeof *info, GFP_KERNEL);
  if (!info)
     return -ENOMEM;

  info->spi = spi;
  info->platform = spi->dev.platform_data;

  printk(KERN_NOTICE "%s found\n", flash_info->name);

  info->auxbyte = 0;
  info->useauxbyte = 0;
  info->lastcommand = 0;

  /* initialise chip data struct */
  chip = &info->chip;

  chip->options |= NAND_CACHEPRG | NAND_SKIP_BBTSCAN;

  chip->read_buf   = mt29f_read_buf;
  chip->write_buf  = mt29f_write_buf;
  chip->read_byte  = mt29f_read_byte;
  chip->cmd_ctrl   = mt29f_hwcontrol;
  chip->dev_ready  = mt29f_devready;
  chip->waitfunc   = mt29f_nand_wait;
  chip->priv       = &info->mtd;
  //chip->controller = &info->controller;
  chip->chip_delay = 0;
  chip->ecc.size      = flash_info->writesize;
  chip->ecc.calculate = mt29f_nand_calculate_ecc;
  chip->ecc.correct   = mt29f_nand_correct_data;
  chip->ecc.mode     = NAND_ECC_HW;
  chip->ecc.hwctl       = mt29f_nand_enable_hwecc;

  /* initialise mtd info data struct */
  info->mtd.priv = chip;
  info->mtd.name = flash_info->name;
  info->mtd.owner   = THIS_MODULE;
  info->mtd.size  = flash_info->size;

  /* scan hardware nand chip and setup mtd info data struct */
  if (nand_scan(&info->mtd, 1)) {
     return -1;
  }

  mt29f_add_partition(info);

  return 0;
}

static int __devexit mt29f_remove(struct spi_device *spi)
{
/*   struct mt29f   *flash = dev_get_drvdata(&spi->dev);

  DEBUG(MTD_DEBUG_LEVEL1, "%s: remove\n", spi->dev.bus_id);

  if (flash->mtd) {
     nand_release(flash->mtd);
     kfree(flash->mtd);
  }*/

  return 0;
}

static struct spi_driver mt29f_driver = {
  .driver = {
     .name    = DRV_NAME,
     .bus     = &spi_bus_type,
     .owner      = THIS_MODULE,
  },

  .probe      = mt29f_probe,
  .remove     = __devexit_p(mt29f_remove),
};

static int __init mt29f_init(void)
{
  return spi_register_driver(&mt29f_driver);
}
module_init(mt29f_init);

static void __exit mt29f_exit(void)
{
  spi_unregister_driver(&mt29f_driver);
}
module_exit(mt29f_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Miguel Ángel Álvarez");
MODULE_DESCRIPTION("MT29F driver");
