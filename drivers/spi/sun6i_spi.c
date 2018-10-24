/*
 * (C) Copyright 2018 Far South Networks
 * Michael Walton <mike@farsouthnet.com>
 *
 * Based on sun4i_spi.c and adapted to A64
 *
 * (C) Copyright 2017 Whitebox Systems / Northend Systems B.V.
 * S.J.R. van Schaik <stephan@whiteboxsystems.nl>
 * M.B.W. Wajer <merlijn@whiteboxsystems.nl>
 *
 * (C) Copyright 2017 Olimex Ltd..
 * Stefan Mavrodiev <stefan@olimex.com>
 *
 * Based on linux spi driver. Original copyright follows:
 * linux/drivers/spi/spi-sun4i.c
 *
 * Copyright (C) 2012 - 2014 Allwinner Tech
 * Pan Nan <pannan@allwinnertech.com>
 *
 * Copyright (C) 2014 Maxime Ripard
 * Maxime Ripard <maxime.ripard@free-electrons.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <dm.h>
#include <spi.h>
#include <errno.h>
#include <fdt_support.h>
#include <wait_bit.h>

#include <asm/bitops.h>
#include <asm/gpio.h>
#include <asm/io.h>

#include <asm/arch/clock.h>

//#define SUN6I_FIFO_DEPTH		128
// Specific to A64
#define SUN50I_FIFO_DEPTH			64
#define SUN50I_IOMUX				4
#define SUN50I_SPI1_ADDR			0x01c69000
#define AHB_GATE_OFFSET_SPI0        20
#define AHB_GATE_OFFSET_SPI1        21


#define SUN6I_GBL_CTL_REG		0x04
#define SUN6I_GBL_CTL_BUS_ENABLE		BIT(0)
#define SUN6I_GBL_CTL_MASTER			BIT(1)
#define SUN6I_GBL_CTL_TP			BIT(7)
#define SUN6I_GBL_CTL_RST			BIT(31)

#define SUN6I_TFR_CTL_REG		0x08
#define SUN6I_TFR_CTL_CPHA			BIT(0)
#define SUN6I_TFR_CTL_CPOL			BIT(1)
#define SUN6I_TFR_CTL_SPOL			BIT(2)
#define SUN6I_TFR_CTL_CS_MASK			0x30
#define SUN6I_TFR_CTL_CS(cs)			(((cs) << 4) & SUN6I_TFR_CTL_CS_MASK)
#define SUN6I_TFR_CTL_CS_MANUAL			BIT(6)
#define SUN6I_TFR_CTL_CS_LEVEL			BIT(7)
#define SUN6I_TFR_CTL_DHB			BIT(8)
#define SUN6I_TFR_CTL_FBS			BIT(12)
#define SUN6I_TFR_CTL_XCH			BIT(31)

#define SUN6I_INT_CTL_REG		0x10
#define SUN6I_INT_CTL_RF_RDY			BIT(0)
#define SUN6I_INT_CTL_TF_ERQ			BIT(4)
#define SUN6I_INT_CTL_RF_OVF			BIT(8)
#define SUN6I_INT_CTL_TC			BIT(12)

#define SUN6I_INT_STA_REG		0x14

#define SUN6I_FIFO_CTL_REG		0x18
#define SUN6I_FIFO_CTL_RF_RDY_TRIG_LEVEL_MASK	0xff
#define SUN6I_FIFO_CTL_RF_RDY_TRIG_LEVEL_BITS	0
#define SUN6I_FIFO_CTL_RF_RST			BIT(15)
#define SUN6I_FIFO_CTL_TF_ERQ_TRIG_LEVEL_MASK	0xff
#define SUN6I_FIFO_CTL_TF_ERQ_TRIG_LEVEL_BITS	16
#define SUN6I_FIFO_CTL_TF_RST			BIT(31)

#define SUN6I_FIFO_STA_REG		0x1c
#define SUN6I_FIFO_STA_RF_CNT_MASK		0x7f
#define SUN6I_FIFO_STA_RF_CNT_BITS		0
#define SUN6I_FIFO_STA_TF_CNT_MASK		0x7f
#define SUN6I_FIFO_STA_TF_CNT_BITS		16

#define SUN6I_CLK_CTL_REG		0x24
#define SUN6I_CLK_CTL_CDR2_MASK			0xff
#define SUN6I_CLK_CTL_CDR2(div)			(((div) & SUN6I_CLK_CTL_CDR2_MASK) << 0)
#define SUN6I_CLK_CTL_CDR1_MASK			0xf
#define SUN6I_CLK_CTL_CDR1(div)			(((div) & SUN6I_CLK_CTL_CDR1_MASK) << 8)
#define SUN6I_CLK_CTL_DRS			BIT(12)

#define SUN6I_MAX_XFER_SIZE		0xffffff

#define SUN6I_BURST_CNT_REG		0x30
#define SUN6I_BURST_CNT(cnt)			((cnt) & SUN6I_MAX_XFER_SIZE)

#define SUN6I_XMIT_CNT_REG		0x34
#define SUN6I_XMIT_CNT(cnt)			((cnt) & SUN6I_MAX_XFER_SIZE)

#define SUN6I_BURST_CTL_CNT_REG		0x38
#define SUN6I_BURST_CTL_CNT_STC(cnt)		((cnt) & SUN6I_MAX_XFER_SIZE)

#define SUN6I_TXDATA_REG		0x200
#define SUN6I_RXDATA_REG		0x300

#define SUN6I_SPI_MAX_RATE	24000000
#define SUN6I_SPI_MIN_RATE	3000
#define SUN6I_SPI_DEFAULT_RATE	1000000
#define SUN6I_SPI_TIMEOUT_US	1000000

#ifndef CONFIG_SPL_BUILD

struct sun6i_spi_platdata {
	u32 spin;
	u32 base_addr;
	u32 max_hz;
};

struct sun6i_spi_priv {
	//struct sun4i_spi_regs *regs;
	u32 spin;
	void *base_addr;
	u32 freq;
	u32 mode;

	const u8 *tx_buf;
	u8 *rx_buf;
};

DECLARE_GLOBAL_DATA_PTR;

static inline u32 sun6i_spi_read(struct sun6i_spi_priv *sspi, u32 reg)
{
	return readl(sspi->base_addr + reg);
}

static inline void sun6i_spi_write(struct sun6i_spi_priv *sspi, u32 reg, u32 value)
{
	writel(value, sspi->base_addr + reg);
}

static inline void sun6i_spi_drain_fifo(struct sun6i_spi_priv *priv, int len)
{
	u8 byte;

	while (len--) {
		byte = readb(priv->base_addr + SUN6I_RXDATA_REG);
		*priv->rx_buf++ = byte;
	}
}

static inline void sun6i_spi_fill_fifo(struct sun6i_spi_priv *priv, int len)
{
	u8 byte;

	while (len--) {
		byte = priv->tx_buf ? *priv->tx_buf++ : 0;
		writeb(byte, priv->base_addr + SUN6I_TXDATA_REG);
	}
}


static void sun6i_spi_set_cs(struct udevice *bus, u8 cs, bool enable)
{
	struct sun6i_spi_priv *priv = dev_get_priv(bus);
	u32 reg;

	reg = sun6i_spi_read(priv, SUN6I_TFR_CTL_REG);
	reg &= ~SUN6I_TFR_CTL_CS_MASK;
	reg |= SUN6I_TFR_CTL_CS(cs);

	/* Active low */
	if (!enable)
		reg |= SUN6I_TFR_CTL_CS_LEVEL;
	else
		reg &= ~SUN6I_TFR_CTL_CS_LEVEL;

	sun6i_spi_write(priv, SUN6I_TFR_CTL_REG, reg);
}

static int sun6i_spi_parse_pins(struct udevice *dev)
{
	const void *fdt = gd->fdt_blob;
	const char *pin_name;
	const fdt32_t *list;
	u32 phandle;
	int drive, pull = 0, pin, i;
	int offset;
	int size;

	list = fdt_getprop(fdt, dev_of_offset(dev), "pinctrl-0", &size);
	if (!list) {
		printf("WARNING: sun6i_spi: cannot find pinctrl-0 node\n");
		return -EINVAL;
	}

	while (size) {
		phandle = fdt32_to_cpu(*list++);
		size -= sizeof(*list);

		offset = fdt_node_offset_by_phandle(fdt, phandle);
		if (offset < 0)
			return offset;

		drive = fdt_getprop_u32_default_node(fdt, offset, 0,
						     "drive-strength", 0);
		if (drive) {
			if (drive <= 10)
				drive = 0;
			else if (drive <= 20)
				drive = 1;
			else if (drive <= 30)
				drive = 2;
			else
				drive = 3;
		} else {
			drive = fdt_getprop_u32_default_node(fdt, offset, 0,
							     "allwinner,drive",
							      0);
			drive = min(drive, 3);
		}

		if (fdt_get_property(fdt, offset, "bias-disable", NULL))
			pull = 0;
		else if (fdt_get_property(fdt, offset, "bias-pull-up", NULL))
			pull = 1;
		else if (fdt_get_property(fdt, offset, "bias-pull-down", NULL))
			pull = 2;
		else
			pull = fdt_getprop_u32_default_node(fdt, offset, 0,
							    "allwinner,pull",
							     0);
		pull = min(pull, 2);

		// SPI0 - PC0 PC1 PC2 PC3
		// SPI1 - PD0 PD1 PD2 PD3
		for (i = 0; ; i++) {
			pin_name = fdt_stringlist_get(fdt, offset,
						      "pins", i, NULL);
			if (!pin_name) {
				pin_name = fdt_stringlist_get(fdt, offset,
							      "allwinner,pins",
							       i, NULL);
				if (!pin_name)
					break;
			}

			pin = name_to_gpio(pin_name);
			if (pin < 0)
				break;

			sunxi_gpio_set_cfgpin(pin, SUN50I_IOMUX);
			sunxi_gpio_set_drv(pin, drive);
			sunxi_gpio_set_pull(pin, pull);
		}
	}
	return 0;
}

static inline void sun6i_spi_enable_clock(struct sun6i_spi_priv *priv)
{
	struct sunxi_ccm_reg *const ccm =
		(struct sunxi_ccm_reg *const)SUNXI_CCM_BASE;

	u32 bit = AHB_GATE_OFFSET_SPI0; // also applicable to reset
	if (priv->spin == 1) {
		bit = AHB_GATE_OFFSET_SPI1;
	}

	/* Deassert SPI reset */
	setbits_le32(&ccm->ahb_reset0_cfg, (1 << bit));

	/* Enable clock */
	setbits_le32(&ccm->ahb_gate0, (1 << bit));
	writel((1 << 31), priv->spin == 1 ? &ccm->spi1_clk_cfg : &ccm->spi0_clk_cfg);
}

static int sun6i_spi_ofdata_to_platdata(struct udevice *bus)
{
	struct sun6i_spi_platdata *plat = dev_get_platdata(bus);
	int node = dev_of_offset(bus);

	plat->base_addr = devfdt_get_addr(bus);
	plat->max_hz = fdtdec_get_int(gd->fdt_blob, node,
				      "spi-max-frequency",
				      SUN6I_SPI_DEFAULT_RATE);
	// Ugly, but no easy way to determine which SPI?
	plat->spin = (plat->base_addr == SUN50I_SPI1_ADDR) ? 1 : 0;

	if (plat->max_hz > SUN6I_SPI_MAX_RATE)
		plat->max_hz = SUN6I_SPI_MAX_RATE;

	return 0;
}

static int sun6i_spi_probe(struct udevice *bus)
{
	struct sun6i_spi_platdata *plat = dev_get_platdata(bus);
	struct sun6i_spi_priv *priv = dev_get_priv(bus);

	sun6i_spi_parse_pins(bus);

	priv->spin = plat->spin;
	priv->base_addr = (void*)(uintptr_t)plat->base_addr;
	priv->freq = plat->max_hz;

	sun6i_spi_enable_clock(priv);

	/* Let's reset again */
	sun6i_spi_write(priv, SUN6I_GBL_CTL_REG,
			SUN6I_GBL_CTL_BUS_ENABLE | SUN6I_GBL_CTL_MASTER |
			/*SUN6I_GBL_CTL_TP |*/ SUN6I_GBL_CTL_RST);

	/* wait for reset to complete */
	wait_for_bit_le32(priv->base_addr + SUN6I_GBL_CTL_REG, SUN6I_GBL_CTL_RST,
				false, SUN6I_SPI_TIMEOUT_US, false);
	return 0;
}

static int sun6i_spi_claim_bus(struct udevice *dev)
{
	struct sun6i_spi_priv *priv = dev_get_priv(dev->parent);

	sun6i_spi_write(priv, SUN6I_GBL_CTL_REG,
			SUN6I_GBL_CTL_BUS_ENABLE | SUN6I_GBL_CTL_MASTER |
			/*SUN6I_GBL_CTL_TP |*/ SUN6I_GBL_CTL_RST);

	/* wait for reset to complete */
	wait_for_bit_le32(priv->base_addr + SUN6I_GBL_CTL_REG, SUN6I_GBL_CTL_RST,
				false, SUN6I_SPI_TIMEOUT_US, false);

	return 0;
}

static int sun6i_spi_release_bus(struct udevice *dev)
{
	struct sun6i_spi_priv *priv = dev_get_priv(dev->parent);
	//u32 reg;

	sun6i_spi_write(priv, SUN6I_GBL_CTL_REG, 0);

	return 0;
}

static int sun6i_spi_xfer(struct udevice *dev, unsigned int bitlen,
			  const void *dout, void *din, unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct sun6i_spi_priv *priv = dev_get_priv(bus);
	struct dm_spi_slave_platdata *slave_plat = dev_get_parent_platdata(dev);

	u32 len = bitlen / 8;
	u32 reg;
	u8 nbytes;
	int ret;

	priv->tx_buf = dout;
	priv->rx_buf = din;

	if (bitlen % 8) {
		debug("%s: non byte-aligned SPI transfer.\n", __func__);
		return -ENAVAIL;
	}

	if (flags & SPI_XFER_BEGIN)
		sun6i_spi_set_cs(bus, slave_plat->cs, true);

	reg = sun6i_spi_read(priv, SUN6I_FIFO_CTL_REG);

	/* Reset FIFOs */
	sun6i_spi_write(priv, SUN6I_FIFO_CTL_REG, reg | SUN6I_FIFO_CTL_RF_RST | SUN6I_FIFO_CTL_TF_RST);

	while (len) {
		/* Setup the transfer now... */
		nbytes = min(len, (u32)(SUN50I_FIFO_DEPTH - 1));

		/* Setup the counters */
		sun6i_spi_write(priv, SUN6I_BURST_CNT_REG, SUN6I_BURST_CNT(nbytes));
		sun6i_spi_write(priv, SUN6I_XMIT_CNT_REG, SUN6I_XMIT_CNT(nbytes));
		sun6i_spi_write(priv, SUN6I_BURST_CTL_CNT_REG,
				SUN6I_BURST_CTL_CNT_STC(nbytes));


		/* Fill the TX FIFO */
		sun6i_spi_fill_fifo(priv, nbytes);

		/* Start the transfer */
		reg = sun6i_spi_read(priv, SUN6I_TFR_CTL_REG);
		sun6i_spi_write(priv, SUN6I_TFR_CTL_REG, reg | SUN6I_TFR_CTL_XCH);

		/* Wait for transfer to complete */
		ret = wait_for_bit_le32(priv->base_addr + SUN6I_TFR_CTL_REG, SUN6I_TFR_CTL_XCH,
					false, SUN6I_SPI_TIMEOUT_US, false);
		if (ret) {
			printf("ERROR: sun6i_spi: Timeout transferring data\n");
			sun6i_spi_set_cs(bus, slave_plat->cs, false);
			return ret;
		}

		/* Drain the RX FIFO */
		sun6i_spi_drain_fifo(priv, nbytes);

		len -= nbytes;
	}

	if (flags & SPI_XFER_END)
		sun6i_spi_set_cs(bus, slave_plat->cs, false);

	return 0;
}

static int sun6i_spi_set_speed(struct udevice *dev, uint speed)
{
	struct sun6i_spi_platdata *plat = dev_get_platdata(dev);
	struct sun6i_spi_priv *priv = dev_get_priv(dev);
	unsigned int div;
	u32 reg;

	if (speed > plat->max_hz)
		speed = plat->max_hz;

	if (speed < SUN6I_SPI_MIN_RATE)
		speed = SUN6I_SPI_MIN_RATE;
	/*
	 * Setup clock divider.
	 *
	 * We have two choices there. Either we can use the clock
	 * divide rate 1, which is calculated thanks to this formula:
	 * SPI_CLK = MOD_CLK / (2 ^ (cdr + 1))
	 * Or we can use CDR2, which is calculated with the formula:
	 * SPI_CLK = MOD_CLK / (2 * (cdr + 1))
	 * Whether we use the former or the latter is set through the
	 * DRS bit.
	 *
	 * First try CDR2, and if we can't reach the expected
	 * frequency, fall back to CDR1.
	 */

	div = SUN6I_SPI_MAX_RATE / (2 * speed);
	reg = sun6i_spi_read(priv, SUN6I_CLK_CTL_REG);

	if (div <= (SUN6I_CLK_CTL_CDR2_MASK + 1)) {
		if (div > 0)
			div--;

		reg &= ~(SUN6I_CLK_CTL_CDR2_MASK | SUN6I_CLK_CTL_DRS);
		reg |= SUN6I_CLK_CTL_CDR2(div) | SUN6I_CLK_CTL_DRS;
	} else {
		div = __ilog2(SUN6I_SPI_MAX_RATE) - __ilog2(speed);
		reg &= ~((SUN6I_CLK_CTL_CDR1_MASK << 8) | SUN6I_CLK_CTL_DRS);
		reg |= SUN6I_CLK_CTL_CDR1(div);
	}

	priv->freq = speed;
	sun6i_spi_write(priv, SUN6I_CLK_CTL_REG, reg);

	/* Let's reset again */
	sun6i_spi_write(priv, SUN6I_GBL_CTL_REG,
			SUN6I_GBL_CTL_BUS_ENABLE | SUN6I_GBL_CTL_MASTER |
			/*SUN6I_GBL_CTL_TP |*/ SUN6I_GBL_CTL_RST);

	/* wait for reset to complete */
	wait_for_bit_le32(priv->base_addr + SUN6I_GBL_CTL_REG, SUN6I_GBL_CTL_RST,
				false, SUN6I_SPI_TIMEOUT_US, false);

	return 0;
}

static int sun6i_spi_set_mode(struct udevice *dev, uint mode)
{
	struct sun6i_spi_priv *priv = dev_get_priv(dev);
	u32 reg;

	reg = sun6i_spi_read(priv, SUN6I_TFR_CTL_REG);
	reg &= ~(SUN6I_TFR_CTL_CPOL | SUN6I_TFR_CTL_CPHA);

	if (mode & SPI_CPOL)
		reg |= SUN6I_TFR_CTL_CPOL;

	if (mode & SPI_CPHA)
		reg |= SUN6I_TFR_CTL_CPHA;

	reg |= SUN6I_TFR_CTL_CS_MANUAL | SUN6I_TFR_CTL_CS_LEVEL;

	priv->mode = mode;
	//printf("set tfr_ctl=0x%08X\n", reg);
	sun6i_spi_write(priv, SUN6I_TFR_CTL_REG, reg);

	return 0;
}

static const struct dm_spi_ops sun6i_spi_ops = {
	.claim_bus		= sun6i_spi_claim_bus,
	.release_bus	= sun6i_spi_release_bus,
	.xfer			= sun6i_spi_xfer,
	.set_speed		= sun6i_spi_set_speed,
	.set_mode		= sun6i_spi_set_mode,
};

static const struct udevice_id sun6i_spi_ids[] = {
	{ .compatible = "allwinner,sun6i-a31-spi"  },
	{ }
};

U_BOOT_DRIVER(sun6i_spi) = {
	.name	= "sun6i_spi",
	.id	= UCLASS_SPI,
	.of_match	= sun6i_spi_ids,
	.ops	= &sun6i_spi_ops,
	.ofdata_to_platdata	= sun6i_spi_ofdata_to_platdata,
	.platdata_auto_alloc_size	= sizeof(struct sun6i_spi_platdata),
	.priv_auto_alloc_size	= sizeof(struct sun6i_spi_priv),
	.probe	= sun6i_spi_probe,
};

#endif
