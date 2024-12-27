/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2021 Renesas Electronics Corporation
 */

#ifndef __SMARC_RZV2L_H
#define __SMARC_RZV2L_H

#include <asm/arch/rmobile.h>

#define CONFIG_REMAKE_ELF

#ifdef CONFIG_SPL
#define CONFIG_SPL_TARGET	"spl/u-boot-spl.scif"
#endif

/* boot option */

#define CONFIG_CMDLINE_TAG
#define CONFIG_SETUP_MEMORY_TAGS
#define CONFIG_INITRD_TAG

/* Generic Interrupt Controller Definitions */
/* RZ/V2L use GIC-v3 */
#define CONFIG_GICV3
#define GICD_BASE	0x11900000
#define GICR_BASE	0x11960000

/* console */
#define CONFIG_SYS_CBSIZE		2048
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE
#define CONFIG_SYS_MAXARGS		64
#define CONFIG_SYS_BAUDRATE_TABLE	{ 115200, 38400 }

/* PHY needs a longer autoneg timeout */
#define PHY_ANEG_TIMEOUT		20000

/* MEMORY */
#define CONFIG_SYS_INIT_SP_ADDR		CONFIG_SYS_TEXT_BASE

/* SDHI clock freq */
#define CONFIG_SH_SDHI_FREQ		133000000

#define DRAM_RSV_SIZE			0x08000000
#define CONFIG_SYS_SDRAM_BASE		(0x40000000 + DRAM_RSV_SIZE)
#define CONFIG_SYS_SDRAM_SIZE		(0x80000000u - DRAM_RSV_SIZE) //total 2GB
#define CONFIG_SYS_LOAD_ADDR		0x58000000
#define CONFIG_LOADADDR			CONFIG_SYS_LOAD_ADDR // Default load address for tfpt,bootp...
#define CONFIG_VERY_BIG_RAM
#define CONFIG_MAX_MEM_MAPPED		(0x80000000u - DRAM_RSV_SIZE)

#define CONFIG_SYS_MONITOR_BASE		0x00000000
#define CONFIG_SYS_MONITOR_LEN		(1 * 1024 * 1024)
#define CONFIG_SYS_MALLOC_LEN		(64 * 1024 * 1024)
#define CONFIG_SYS_BOOTM_LEN		(64 << 20)

/* The HF/QSPI layout permits up to 1 MiB large bootloader blob */
#define CONFIG_BOARD_SIZE_LIMIT		1048576

/* ENV setting */

#define CONFIG_EXTRA_ENV_SETTINGS \
	"usb_pgood_delay=2000\0" \
	"bootdelay=1\0" \
	"bootm_size=0x10000000 \0" \
	"prodemmcbootargs=setenv bootargs rw rootwait earlycon root=/dev/mmcblk0p2 \0" \
	"bootimage=booti 0x48080000 - 0x48000000 \0" \
	"emmcload=fatload mmc 0:1 0x48080000 Image;fatload mmc 0:1 0x48000000 ly-rzv2l-smarc.dtb;run prodemmcbootargs \0" \
	"bootcmd_check=if mmc dev 0; then run emmcload; fi \0" \
	"usbpart=0:1\0" \
	"usbload=usb start;fatload usb ${usbpart} 0x48080000 boot/Image;fatload usb ${usbpart} 0x48000000 boot/ly-rzv2l-smarc.dtb;fatload usb ${usbpart} 0x50000000 boot/RZV2L-LY.cpio.gz.u-boot;run prodemmcbootargs;booti 0x48080000 0x50000000 0x48000000 \0"

#define CONFIG_BOOTCOMMAND	"env default -a;run bootcmd_check;run bootimage"

/* For board */
/* Ethernet RAVB */
#define CONFIG_BITBANGMII_MULTI

#endif /* __SMARC_RZV2L_H */
