/*
 * Copyright (C) 2012-2015 Freescale Semiconductor, Inc.
 *
 * Configuration settings for the Freescale i.MX6SCM FreedomX board.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MX6SCMQSB_CONFIG_H
#define __MX6SCMQSB_CONFIG_H

#include <asm/arch/imx-regs.h>
#include <asm/imx-common/gpio.h>
#ifdef CONFIG_SPL
#define CONFIG_SPL_LIBCOMMON_SUPPORT
#define CONFIG_SPL_MMC_SUPPORT
#include "imx6_spl.h"
#endif

#define CONFIG_MACH_TYPE	3980
#ifdef CONFIG_SCMHVB
#define CONFIG_MXC_UART_BASE	UART4_BASE
#define CONFIG_CONSOLE_DEV		"ttymxc3"
#else
#define CONFIG_MXC_UART_BASE	UART1_BASE
#define CONFIG_CONSOLE_DEV		"ttymxc0"
#endif
#define CONFIG_MMCROOT			"/dev/mmcblk2p2"  /* SDHC3 */
#define CONFIG_DMFC				"dmfc=3"

#ifdef CONFIG_SCM_LPDDR2_512MB
#define PHYS_SDRAM_SIZE         (512 * 1024 * 1024)
#else
#define PHYS_SDRAM_SIZE         (1u * 1024 * 1024 * 1024)
#endif

#include "mx6sabre_common.h"

#define CONFIG_SYS_FSL_USDHC_NUM	3
#define CONFIG_SYS_MMC_ENV_DEV		1	/* SDHC3 */
#define CONFIG_SYS_MMC_ENV_PART                0       /* user partition */

#ifdef CONFIG_SYS_USE_SPINOR
#define CONFIG_SF_DEFAULT_CS   0
#endif

/* FEC NET ip addr */
#undef CONFIG_IPADDR
#define CONFIG_IPADDR			192.168.1.10
#define CONFIG_NETMASK			255.255.255.0
#undef CONFIG_SERVERIP
#define CONFIG_SERVERIP			192.168.1.12
#define CONFIG_ETHADDR                  00:11:22:33:44:55
#define CONFIG_GATEWAYIP                192.168.1.100

/* Physical Memory Map */
#if !defined(CONFIG_INTERLEAVING_MODE) && !defined(CONFIG_SCM_LPDDR2_512MB)
#undef CONFIG_NR_DRAM_BANKS
#define CONFIG_NR_DRAM_BANKS            2
#ifdef CONFIG_SCM_LPDDR2_2GB
#define PHYS_SDRAM_0            MMDC0_ARB_BASE_ADDR     /* SDRAM Bank #1 */
#define PHYS_SDRAM_1            MMDC1_ARB_BASE_ADDR     /* SDRAM Bank #2 */

#define PHYS_SDRAM_0_SIZE       (1024 * 1024 * 1024)      /* 1024MB */
#define PHYS_SDRAM_1_SIZE       (1024 * 1024 * 1024)      /* 1024MB */

#else

#define PHYS_SDRAM_0            MMDC0_ARB_BASE_ADDR     /* SDRAM Bank #1 */
#define PHYS_SDRAM_1            MMDC1_ARB_BASE_ADDR     /* SDRAM Bank #2 */

#define PHYS_SDRAM_0_SIZE       (512 * 1024 * 1024)      /* 512MB */
#define PHYS_SDRAM_1_SIZE       (512 * 1024 * 1024)      /* 512MB */
#endif
#endif
/*
 * imx6 q/dl/solo pcie would be failed to work properly in kernel, if
 * the pcie module is iniialized/enumerated both in uboot and linux
 * kernel.
 * rootcause:imx6 q/dl/solo pcie don't have the reset mechanism.
 * it is only be RESET by the POR. So, the pcie module only be
 * initialized/enumerated once in one POR.
 * Set to use pcie in kernel defaultly, mask the pcie config here.
 * Remove the mask freely, if the uboot pcie functions, rather than
 * the kernel's, are required.
 */
/* #define CONFIG_CMD_PCI */
#ifdef CONFIG_CMD_PCI
#define CONFIG_PCI
#define CONFIG_PCI_PNP
#define CONFIG_PCI_SCAN_SHOW
#define CONFIG_PCIE_IMX
#define CONFIG_PCIE_IMX_PERST_GPIO	IMX_GPIO_NR(7, 12)
#define CONFIG_PCIE_IMX_POWER_GPIO	IMX_GPIO_NR(3, 19)
#endif

/* PMIC */
#define CONFIG_POWER
#define CONFIG_POWER_I2C
#define CONFIG_POWER_PFUZE100
#define CONFIG_POWER_PFUZE100_I2C_ADDR	0x08

/* USB Configs */
#define CONFIG_CMD_USB
#ifdef CONFIG_CMD_USB
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_MX6
#define CONFIG_USB_STORAGE
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_USB_HOST_ETHER
#define CONFIG_USB_ETHER_ASIX
#define CONFIG_MXC_USB_PORTSC		(PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS		0
#define CONFIG_USB_MAX_CONTROLLER_COUNT	1 /* Enabled USB controller number */
#endif

/* RGMII */
#undef CONFIG_PHY_ATHEROS
#undef CONFIG_FEC_MXC_PHYADDR
#define CONFIG_PHY_MICREL
/* 3 */
#define CONFIG_FEC_MXC_PHYADDR          3

#define CONFIG_CMD_IRQ
#define CONFIG_USE_IRQ
/*
 * Stack sizes
 */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ            (4*1024)        /* IRQ stack */
#define CONFIG_STACKSIZE_FIQ            (4*1024)        /* FIQ stack */
#endif


/* Network */
/*
#undef CONFIG_PHY_ATHEROS
#undef CONFIG_FEC_MXC_PHYADDR
#define CONFIG_CMD_PING
#define CONFIG_CMD_DHCP
#define CONFIG_CMD_MII
#define CONFIG_CMD_NET
#define CONFIG_FEC_MXC
#define CONFIG_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_XCV_TYPE		RGMII
#define CONFIG_ETHPRIME			"FEC"
#define CONFIG_FEC_MXC_PHYADDR		6
#define CONFIG_PHYLIB
#define CONFIG_PHY_MICREL
*/

/*#define CONFIG_SPLASH_SCREEN*/
/*#define CONFIG_MXC_EPDC*/




/*
 * SPLASH SCREEN Configs
 */
#if defined(CONFIG_SPLASH_SCREEN) && defined(CONFIG_MXC_EPDC)
	/*
	 * Framebuffer and LCD
	 */
	#define CONFIG_CMD_BMP
	#define CONFIG_LCD
	#define CONFIG_SYS_CONSOLE_IS_IN_ENV
	#undef LCD_TEST_PATTERN
	/* #define CONFIG_SPLASH_IS_IN_MMC			1 */
	#define LCD_BPP					LCD_MONOCHROME
	/* #define CONFIG_SPLASH_SCREEN_ALIGN		1 */

	#define CONFIG_WAVEFORM_BUF_SIZE		0x200000
#endif /* CONFIG_SPLASH_SCREEN && CONFIG_MXC_EPDC */

#if defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#undef CONFIG_ENV_OFFSET
#define CONFIG_ENV_OFFSET              (448 * 1024)
#undef CONFIG_ENV_SECT_SIZE
#define CONFIG_ENV_SECT_SIZE           (64 * 1024)
#define CONFIG_INTERNAL_SPI_NOR_PARTITION "mtdparts=spi0.0:512k(uboot),64k(dtb),6m(linux),-(rootfs) "
#undef CONFIG_MFG_ENV_SETTINGS
#define CONFIG_MFG_ENV_SETTINGS \
	"mfgtool_args=setenv bootargs console=" CONFIG_CONSOLE_DEV ",115200 " \
		"rdinit=/linuxrc " \
		"g_mass_storage.stall=0 g_mass_storage.removable=1 " \
		"g_mass_storage.idVendor=0x066F g_mass_storage.idProduct=0x37FF "\
		"g_mass_storage.iSerialNumber=\"\" "\
		"enable_wait_mode=off "\
		CONFIG_INTERNAL_SPI_NOR_PARTITION \
		CONFIG_MFG_NAND_PARTITION \
		"\0" \
		"initrd_addr=0x12C00000\0" \
		"initrd_high=0xffffffff\0" \
		"bootcmd_mfg=run mfgtool_args;bootz ${loadaddr} ${initrd_addr} ${fdt_addr};\0" 

#undef CONFIG_EXTRA_ENV_SETTINGS 
#define CONFIG_EXTRA_ENV_SETTINGS \
	CONFIG_MFG_ENV_SETTINGS \
	"epdc_waveform=epdc_splash.bin\0" \
	"script=boot.scr\0" \
	"image=zImage\0" \
	"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" \
	"fdt_addr=0x18000000\0" \
	"boot_fdt=try\0" \
	"ip_dyn=yes\0" \
	"console=" CONFIG_CONSOLE_DEV "\0" \
	"fdt_high=0xffffffff\0"	  \
	"initrd_high=0xffffffff\0" \
	"mmcdev=" __stringify(CONFIG_SYS_MMC_ENV_DEV) "\0" \
	"mmcpart=" __stringify(CONFIG_SYS_MMC_IMG_LOAD_PART) "\0" \
	"mmcroot=" CONFIG_MMCROOT " rootwait rw\0" \
	"dmfc=" CONFIG_DMFC "\0" \
	"mmcautodetect=yes\0" \
	"update_sd_firmware=" \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"if mmc dev ${mmcdev}; then "	\
			"if ${get_cmd} ${update_sd_firmware_filename}; then " \
				"setexpr fw_sz ${filesize} / 0x200; " \
				"setexpr fw_sz ${fw_sz} + 1; "	\
				"mmc write ${loadaddr} 0x2 ${fw_sz}; " \
			"fi; "	\
		"fi\0" \
	EMMC_ENV	  \
	"smp=" CONFIG_SYS_NOSMP "\0"\
	"mmcargs=setenv bootargs console=${console},${baudrate} ${smp} " \
		"root=${mmcroot} ${dmfc} \0" \
	"loadbootscript=" \
		"fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${script};\0" \
	"bootscript=echo Running bootscript from mmc ...; " \
		"source\0" \
	"loadimage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} ${image}\0" \
	"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
	"mmcboot=echo Booting from mmc ...; " \
		"run mmcargs; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if run loadfdt; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"netargs=setenv bootargs console=${console},${baudrate} ${smp} " \
		"root=/dev/nfs " \
		"ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
	"netboot=echo Booting from net ...; " \
		"run netargs; " \
		"if test ${ip_dyn} = yes; then " \
			"setenv get_cmd dhcp; " \
		"else " \
			"setenv get_cmd tftp; " \
		"fi; " \
		"${get_cmd} ${image}; " \
		"if test ${boot_fdt} = yes || test ${boot_fdt} = try; then " \
			"if ${get_cmd} ${fdt_addr} ${fdt_file}; then " \
				"bootz ${loadaddr} - ${fdt_addr}; " \
			"else " \
				"if test ${boot_fdt} = try; then " \
					"bootz; " \
				"else " \
					"echo WARN: Cannot load the DT; " \
				"fi; " \
			"fi; " \
		"else " \
			"bootz; " \
		"fi;\0" \
	"spinorargs=setenv bootargs console=${console},${baudrate} root=/dev/mtdblock3 rootwait rw " \
		 CONFIG_INTERNAL_SPI_NOR_PARTITION "\0" \
	"spinorboot=echo Booting from internal SPI-NOR Flash...; " \
		"run spinorargs; sf probe; sf read 0x12000000 0x90000 0x600000; " \
		"sf read 0x18000000 0x80000 0x10000; bootz 0x12000000 - 0x18000000"

#undef CONFIG_BOOTCOMMAND
#define CONFIG_BOOTCOMMAND \
	"run spinorboot; " \
	"mmc dev ${mmcdev};" \
	"if mmc rescan; then " \
		"if run loadbootscript; then " \
		"run bootscript; " \
		"else " \
			"if run loadimage; then " \
				"run mmcboot; " \
			"else run netboot; " \
			"fi; " \
		"fi; " \
	"else run netboot; fi"

#endif
#endif                         /* __MX6SCMQSB_CONFIG_H */
