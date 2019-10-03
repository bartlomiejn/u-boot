// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2017 NXP
 */

#include <common.h>
#include <asm/io.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/arch/ddr.h>
#include <asm/arch/imx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc_imx.h>
#include <mmc.h>
#include <power/pmic.h>
#include <power/pfuze100_pmic.h>
#include <spl.h>

DECLARE_GLOBAL_DATA_PTR;

static void spl_dram_init(void)
{
	/* ddr init */
	ddr_init(&dram_timing);
}

#define I2C_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = IMX8MQ_PAD_I2C1_SCL__I2C1_SCL | PC,
		.gpio_mode = IMX8MQ_PAD_I2C1_SCL__GPIO5_IO14 | PC,
		.gp = IMX_GPIO_NR(5, 14),
	},
	.sda = {
		.i2c_mode = IMX8MQ_PAD_I2C1_SDA__I2C1_SDA | PC,
		.gpio_mode = IMX8MQ_PAD_I2C1_SDA__GPIO5_IO15 | PC,
		.gp = IMX_GPIO_NR(5, 15),
	},
};

#define USDHC1_PWR_GPIO	IMX_GPIO_NR(2, 10)

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret = 0;

	switch (cfg->esdhc_base) {
	case USDHC1_BASE_ADDR:
		ret = 1;
		break;
	}

	return ret;
}

#define USDHC_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | \
			 PAD_CTL_FSEL2)
#define USDHC_GPIO_PAD_CTRL (PAD_CTL_PUE | PAD_CTL_DSE1)

static iomux_v3_cfg_t const usdhc1_pads[] = {
	IMX8MQ_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA4__USDHC1_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA5__USDHC1_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA6__USDHC1_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA7__USDHC1_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_RESET_B__GPIO2_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC1_BASE_ADDR, 0, 8},
};

int board_mmc_init(bd_t *bis)
{
	int i, ret;
	/*
	 * According to the board_mmc_init() the following map is done:
	 * (U-Boot device node)    (Physical Port)
	 * mmc0                    USDHC1
	 * mmc1                    USDHC2
	 */
	for (i = 0; i < CONFIG_SYS_FSL_USDHC_NUM; i++) {
		switch (i) {
		case 0:
			init_clk_usdhc(0);
			usdhc_cfg[0].sdhc_clk = mxc_get_clock(USDHC1_CLK_ROOT);
			imx_iomux_v3_setup_multiple_pads(usdhc1_pads,
							 ARRAY_SIZE(usdhc1_pads));
			gpio_request(USDHC1_PWR_GPIO, "usdhc1_reset");
			gpio_direction_output(USDHC1_PWR_GPIO, 0);
			udelay(500);
			gpio_direction_output(USDHC1_PWR_GPIO, 1);
			break;
		default:
			printf("Warning: you configured more USDHC controllers"
				"(%d) than supported by the board\n", i + 1);
			return -EINVAL;
		}

		ret = fsl_esdhc_initialize(bis, &usdhc_cfg[i]);
		if (ret)
			return ret;
	}

	return 0;
}

#define GP_I2C1_PCA9546_RESET		IMX_GPIO_NR(1, 8)
#define GP_ARM_DRAM_VSEL		IMX_GPIO_NR(3, 24)
#define GP_DRAM_1P1_VSEL		IMX_GPIO_NR(2, 11)
#define GP_SOC_GPU_VPU_VSEL		IMX_GPIO_NR(2, 20)

#define I2C_MUX_ADDR		0x70
#define I2C_FAN53555_ADDR	0x60

static iomux_v3_cfg_t const i2c1_pads[] = {
	IMX8MQ_PAD_GPIO1_IO08__GPIO1_IO8 | MUX_PAD_CTRL(0x49),
};

int power_init_board(void)
{
	u8 val8;

	imx_iomux_v3_setup_multiple_pads(i2c1_pads, ARRAY_SIZE(i2c1_pads));

	gpio_set_value(GP_I2C1_PCA9546_RESET, 1);
	gpio_set_value(GP_ARM_DRAM_VSEL, 0);
	gpio_set_value(GP_DRAM_1P1_VSEL, 0);
	gpio_set_value(GP_SOC_GPU_VPU_VSEL, 0);

	/*
	 * 9e (1e = 30) default .9 V
	 * 0.6V to 1.23V in 10 MV steps
	 */

	/* Enable I2C1A, ARM/DRAM */
	i2c_write(I2C_MUX_ADDR, 1, 1, NULL, 0);
	/*
	 * .6 + .40 = 1.00
	 */
	val8 = 0x80 + 40;
	i2c_write(I2C_FAN53555_ADDR, 0, 1, &val8, 1);
	i2c_write(I2C_FAN53555_ADDR, 1, 1, &val8, 1);

	/* Enable I2C1B, DRAM 1.1V */
	i2c_write(I2C_MUX_ADDR, 2, 1, NULL, 0);
	/*
	 * .6 + .50 = 1.10
	 */
	val8 = 0x80 + 50;
	i2c_write(I2C_FAN53555_ADDR, 0, 1, &val8, 1);
	i2c_write(I2C_FAN53555_ADDR, 1, 1, &val8, 1);

	/* Enable I2C1C, soc/gpu/vpu */
	i2c_write(I2C_MUX_ADDR, 4, 1, NULL, 0);
	/*
	 * .6 + .30 = .90
	 */
	val8 = 0x80 + 30;
	i2c_write(I2C_FAN53555_ADDR, 0, 1, &val8, 1);
	i2c_write(I2C_FAN53555_ADDR, 1, 1, &val8, 1);

	/* Disable I2C1A-I2C1D */
	i2c_write(I2C_MUX_ADDR, 0, 1, NULL, 0);

	return 0;
}

void spl_board_init(void)
{
	/* Serial download mode */
	if (is_usb_boot()) {
		puts("Back to ROM, SDP\n");
		restore_boot_params();
	}
	puts("Normal Boot\n");
}

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* Just empty function now - can't decide what to choose */
	debug("%s: %s\n", __func__, name);

	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	int ret;

	/* Clear global data */
	memset((void *)gd, 0, sizeof(gd_t));

	arch_cpu_init();

	init_uart_clk(0);

	board_early_init_f();

	timer_init();

	preloader_console_init();

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	ret = spl_init();
	if (ret) {
		debug("spl_init() failed: %d\n", ret);
		hang();
	}

	enable_tzc380();

	/* Adjust pmic voltage to 1.0V for 800M */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);

	power_init_board();

	/* DDR initialization */
	spl_dram_init();

	board_init_r(NULL, 0);
}
