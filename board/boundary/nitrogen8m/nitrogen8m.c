// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2016 Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
 */

#include <common.h>
#include <env.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm-generic/gpio.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <asm/arch/imx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/video.h>
#include <video_fb.h>
#include <spl.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>

DECLARE_GLOBAL_DATA_PTR;

#define GPIRQ_ENET_PHY		IMX_GPIO_NR(1, 2)
#define GPIRQ_GT911 		IMX_GPIO_NR(3, 12)
#define GP_ARM_DRAM_VSEL	IMX_GPIO_NR(3, 24)
#define GP_CSI1_MIPI_PWDN	IMX_GPIO_NR(3, 3)
#define GP_CSI1_MIPI_RESET	IMX_GPIO_NR(3, 17)
#define GP_CSI2_MIPI_PWDN	IMX_GPIO_NR(3, 2)
#define GP_CSI2_MIPI_RESET	IMX_GPIO_NR(2, 19)
#define GP_DRAM_1P1_VSEL	IMX_GPIO_NR(2, 11)
#define GP_EMMC_RESET		IMX_GPIO_NR(2, 10)
#define GP_FASTBOOT_KEY		IMX_GPIO_NR(1, 7)
#define GP_GT911_RESET		IMX_GPIO_NR(3, 13)
#define GP_I2C1_PCA9546_RESET	IMX_GPIO_NR(1, 8)
#define GP_I2C4_SN65DSI83_EN	IMX_GPIO_NR(3, 15)
#define GP_LCM_JM430_BKL_EN	IMX_GPIO_NR(1, 1)
/* This enables 5V power on LTK080A60A004T mipi display */
#define GP_LTK08_MIPI_EN	IMX_GPIO_NR(1, 1)
#define GP_MIPI_RESET		IMX_GPIO_NR(3, 15)
#define GP_RGMII_PHY_RESET	IMX_GPIO_NR(1, 9)
#define GP_SOC_GPU_VPU_VSEL	IMX_GPIO_NR(2, 20)
#define GP_ST1633_RESET		IMX_GPIO_NR(3, 13)
#define GP_TC358762_EN		IMX_GPIO_NR(3, 15)

#define ENET_MDC_PAD_CTRL	(PAD_CTL_DSE3)
#define ENET_MDIO_PAD_CTRL	(PAD_CTL_DSE3 | PAD_CTL_ODE)
#define UART_PAD_CTRL		(PAD_CTL_DSE6 | PAD_CTL_FSEL1)
#define WDOG_PAD_CTRL		(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)
#define WEAK_PULLUP		(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)

static iomux_v3_cfg_t const init_pads[] = {
	IMX8MQ_PAD_GPIO1_IO01__GPIO1_IO1 | MUX_PAD_CTRL(0x16),
	IMX8MQ_PAD_GPIO1_IO02__WDOG1_WDOG_B | MUX_PAD_CTRL(WDOG_PAD_CTRL),
	IMX8MQ_PAD_GPIO1_IO07__GPIO1_IO7 | MUX_PAD_CTRL(WEAK_PULLUP),
	IMX8MQ_PAD_GPIO1_IO08__GPIO1_IO8 | MUX_PAD_CTRL(0x49),
	IMX8MQ_PAD_NAND_CE1_B__GPIO3_IO2 | MUX_PAD_CTRL(0x61),
	IMX8MQ_PAD_NAND_CE2_B__GPIO3_IO3 | MUX_PAD_CTRL(0x61),
	IMX8MQ_PAD_NAND_DATA06__GPIO3_IO12 | MUX_PAD_CTRL(0xd6),
	IMX8MQ_PAD_NAND_DATA07__GPIO3_IO13 | MUX_PAD_CTRL(0x49),
	IMX8MQ_PAD_NAND_RE_B__GPIO3_IO15 | MUX_PAD_CTRL(0x6),
	IMX8MQ_PAD_NAND_WE_B__GPIO3_IO17 | MUX_PAD_CTRL(0x61),
	IMX8MQ_PAD_SAI5_RXD3__GPIO3_IO24 | MUX_PAD_CTRL(0x16),
	IMX8MQ_PAD_SD1_RESET_B__GPIO2_IO10 | MUX_PAD_CTRL(0x41),
	IMX8MQ_PAD_SD1_STROBE__GPIO2_IO11 | MUX_PAD_CTRL(0x16),
	IMX8MQ_PAD_SD2_RESET_B__GPIO2_IO19 | MUX_PAD_CTRL(0x61),
	IMX8MQ_PAD_SD2_WP__GPIO2_IO20 | MUX_PAD_CTRL(0x16),
	IMX8MQ_PAD_UART1_RXD__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MQ_PAD_UART1_TXD__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	IMX8MQ_PAD_GPIO1_IO11__GPIO1_IO11 | MUX_PAD_CTRL(WEAK_PULLUP),
};

int board_early_init_f(void)
{
	struct wdog_regs *wdog = (struct wdog_regs *)WDOG1_BASE_ADDR;

	imx_iomux_v3_setup_multiple_pads(init_pads, ARRAY_SIZE(init_pads));
	set_wdog_reset(wdog);

	gpio_direction_output(GP_ARM_DRAM_VSEL, 0);
	gpio_direction_output(GP_DRAM_1P1_VSEL, 0);
	gpio_direction_output(GP_SOC_GPU_VPU_VSEL, 0);
	gpio_direction_output(GP_EMMC_RESET, 1);
	gpio_direction_output(GP_I2C1_PCA9546_RESET, 0);
	gpio_direction_output(GP_I2C4_SN65DSI83_EN, 0);
	gpio_direction_output(GP_CSI1_MIPI_PWDN, 1);
	gpio_direction_output(GP_CSI1_MIPI_RESET, 0);
	gpio_direction_output(GP_CSI2_MIPI_PWDN, 1);
	gpio_direction_output(GP_CSI2_MIPI_RESET, 0);

	return 0;
}

#define MAX_LOW_SIZE	(0x100000000ULL - CONFIG_SYS_SDRAM_BASE)
#define SDRAM_SIZE	((1ULL * CONFIG_DDR_MB) << 20)

#if SDRAM_SIZE > MAX_LOW_SIZE
#define MEM_SIZE	MAX_LOW_SIZE
#else
#define MEM_SIZE	SDRAM_SIZE
#endif

int dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
	gd->bd->bi_dram[0].size = SDRAM_SIZE;
	return 0;
}

int dram_init(void)
{
	/* rom_pointer[1] contains the size of TEE occupies */
	gd->ram_size = MEM_SIZE - rom_pointer[1];
	return 0;
}

#ifdef CONFIG_FEC_MXC
static iomux_v3_cfg_t const fec1_strap_pads[] = {
	IMX8MQ_PAD_GPIO1_IO09__GPIO1_IO9 | MUX_PAD_CTRL(WEAK_PULLUP),
	IMX8MQ_PAD_ENET_RD0__GPIO1_IO26 | MUX_PAD_CTRL(0xd1),
	IMX8MQ_PAD_ENET_RD1__GPIO1_IO27 | MUX_PAD_CTRL(0xd1),
	IMX8MQ_PAD_ENET_RD2__GPIO1_IO28 | MUX_PAD_CTRL(0xd1),
	IMX8MQ_PAD_ENET_RD3__GPIO1_IO29 | MUX_PAD_CTRL(0x91),
	IMX8MQ_PAD_ENET_RX_CTL__GPIO1_IO24 | MUX_PAD_CTRL(0x91),
	IMX8MQ_PAD_ENET_RXC__GPIO1_IO25 | MUX_PAD_CTRL(0xd1),
};

static iomux_v3_cfg_t const fec1_enet_pads[] = {
	IMX8MQ_PAD_GPIO1_IO09__GPIO1_IO9 | MUX_PAD_CTRL(WEAK_PULLUP),
	IMX8MQ_PAD_ENET_MDC__ENET_MDC | MUX_PAD_CTRL(ENET_MDC_PAD_CTRL),
	IMX8MQ_PAD_ENET_MDIO__ENET_MDIO | MUX_PAD_CTRL(ENET_MDIO_PAD_CTRL),
	IMX8MQ_PAD_ENET_RD0__ENET_RGMII_RD0 | MUX_PAD_CTRL(0x91),
	IMX8MQ_PAD_ENET_RD1__ENET_RGMII_RD1 | MUX_PAD_CTRL(0x91),
	IMX8MQ_PAD_ENET_RD2__ENET_RGMII_RD2 | MUX_PAD_CTRL(0x91),
	IMX8MQ_PAD_ENET_RD3__ENET_RGMII_RD3 | MUX_PAD_CTRL(0x91),
	IMX8MQ_PAD_ENET_RXC__ENET_RGMII_RXC | MUX_PAD_CTRL(0x91),
	IMX8MQ_PAD_ENET_RX_CTL__ENET_RGMII_RX_CTL | MUX_PAD_CTRL(0x91),
	IMX8MQ_PAD_ENET_TD0__ENET_RGMII_TD0 | MUX_PAD_CTRL(0x1f),
	IMX8MQ_PAD_ENET_TD1__ENET_RGMII_TD1 | MUX_PAD_CTRL(0x1f),
	IMX8MQ_PAD_ENET_TD2__ENET_RGMII_TD2 | MUX_PAD_CTRL(0x1f),
	IMX8MQ_PAD_ENET_TD3__ENET_RGMII_TD3 | MUX_PAD_CTRL(0x1f),
	IMX8MQ_PAD_ENET_TXC__ENET_RGMII_TXC | MUX_PAD_CTRL(0x1f),
	IMX8MQ_PAD_ENET_TX_CTL__ENET_RGMII_TX_CTL | MUX_PAD_CTRL(0x1f),
};

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *gpr =
		(struct iomuxc_gpr_base_regs *)IOMUXC_GPR_BASE_ADDR;

	/* Pull PHY into reset */
	gpio_request(IMX_GPIO_NR(1, 9), "fec1_rst");
	gpio_direction_output(IMX_GPIO_NR(1, 9), 0);

	/* Set bootstrap pins for AR8035 */
	gpio_request(IMX_GPIO_NR(1, 26), "fec1_rd0");
	gpio_direction_output(IMX_GPIO_NR(1, 26), 0);
	gpio_request(IMX_GPIO_NR(1, 27), "fec1_rd1");
	gpio_direction_output(IMX_GPIO_NR(1, 27), 0);
	gpio_request(IMX_GPIO_NR(1, 28), "fec1_rd2");
	gpio_direction_output(IMX_GPIO_NR(1, 28), 0);
	gpio_request(IMX_GPIO_NR(1, 29), "fec1_rd3");
	gpio_direction_output(IMX_GPIO_NR(1, 29), 1);
	gpio_request(IMX_GPIO_NR(1, 24), "fec1_rx_ctl");
	gpio_direction_output(IMX_GPIO_NR(1, 24), 0);
	gpio_request(IMX_GPIO_NR(1, 25), "fec1_rxc");
	gpio_direction_output(IMX_GPIO_NR(1, 25), 1);
	imx_iomux_v3_setup_multiple_pads(fec1_strap_pads,
					 ARRAY_SIZE(fec1_strap_pads));

	/* Use 125M anatop REF_CLK1 for ENET1, not from external */
	clrsetbits_le32(&gpr->gpr[1], BIT(13) | BIT(17), 0);
	set_clk_enet(ENET_125MHZ);
	udelay(1000);

	/* Release PHY from reset */
	gpio_direction_output(IMX_GPIO_NR(1, 9), 1);
	udelay(500);

	/* Setup pins for ethernet */
	imx_iomux_v3_setup_multiple_pads(fec1_enet_pads,
					 ARRAY_SIZE(fec1_enet_pads));

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	/* enable rgmii rxc skew and phy mode select to RGMII copper */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);
	return 0;
}
#endif

int board_init(void)
{
#ifdef CONFIG_FEC_MXC
	setup_fec();
#endif

	gpio_request(GP_I2C4_SN65DSI83_EN, "sn65dsi83_enable");
	gpio_request(GP_GT911_RESET, "gt911_reset");
	gpio_request(GPIRQ_GT911, "gt911_irq");
	gpio_request(GP_LTK08_MIPI_EN, "lkt08_mipi_en");
	gpio_direction_output(GP_GT911_RESET, 0);

	return 0;
}

int board_mmc_get_env_dev(int devno)
{
	return 0;
}

int board_late_init(void)
{
#ifdef CONFIG_ENV_VARS_UBOOT_RUNTIME_CONFIG
	env_set("board_name", "nitrogen8m");
#endif

	return 0;
}
