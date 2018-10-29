/*
 * Microchip KSZ8794 4-port managed switch
 *
 * SPI control
 *
 */

#include <common.h>
#include <errno.h>
#include <malloc.h>
#include <mapmem.h>
#include <phy.h>
#include <miiphy.h>
#include <spi.h>

#define KSZ8794_REG_ID0		0x00    /* Chip ID0 */
#define KSZ8794_REG_ID1		0x01    /* Chip ID1 */

#define KSZ8794_REG_GC0		0x02    /* Global Control 0 */
#define KSZ8794_REG_GC1		0x03    /* Global Control 1 */
#define KSZ8794_REG_GC2		0x04    /* Global Control 2 */
#define KSZ8794_REG_GC3		0x05    /* Global Control 3 */
#define KSZ8794_REG_GC4		0x06    /* Global Control 4 */
#define KSZ8794_REG_GC5		0x07    /* Global Control 5 */
#define KSZ8794_REG_GC6		0x08    /* Global Control 6 */
#define KSZ8794_REG_GC7		0x09    /* Global Control 7 */
#define KSZ8794_REG_GC8		0x0a    /* Global Control 8 */
#define KSZ8794_REG_GC9		0x0b    /* Global Control 9 */

#define KSZ8794_REG_PC(p, r)	((0x10 * p) + r)	 /* Port Control, p = 1,2,3,5 */
#define KSZ8794_REG_PS(p, r)	((0x10 * p) + r + 0xe)  /* Port Status */

#define KSZ8794_REG_STATUS0(p)  	(KSZ8794_REG_PC(p, 0x08))
#define KSZ8794_REG_STATUS2(p)  	(KSZ8794_REG_PC(p, 0x0e))
#define KSZ8794_REG_CONTROL9(p) 	(KSZ8794_REG_PC(p, 0x0c))
#define KSZ8794_REG_CONTROL10(p) 	(KSZ8794_REG_PC(p, 0x0d))
#define KSZ8794_REG_CONTROL11(p) 	(KSZ8794_REG_PC(p, 0x0f))

#define KSZ8794_REG_TPC0		0x60    /* TOS Priority Control 0 */
#define KSZ8794_REG_TPC1		0x61    /* TOS Priority Control 1 */
#define KSZ8794_REG_TPC2		0x62    /* TOS Priority Control 2 */
#define KSZ8794_REG_TPC3		0x63    /* TOS Priority Control 3 */
#define KSZ8794_REG_TPC4		0x64    /* TOS Priority Control 4 */
#define KSZ8794_REG_TPC5		0x65    /* TOS Priority Control 5 */
#define KSZ8794_REG_TPC6		0x66    /* TOS Priority Control 6 */
#define KSZ8794_REG_TPC7		0x67    /* TOS Priority Control 7 */

#define KSZ8794_REG_MAC0		0x68    /* MAC address 0 */
#define KSZ8794_REG_MAC1		0x69    /* MAC address 1 */
#define KSZ8794_REG_MAC2		0x6a    /* MAC address 2 */
#define KSZ8794_REG_MAC3		0x6b    /* MAC address 3 */
#define KSZ8794_REG_MAC4		0x6c    /* MAC address 4 */
#define KSZ8794_REG_MAC5		0x6d    /* MAC address 5 */

#define KSZ8794_REG_IAC0		0x6e    /* Indirect Access Control 0 */
#define KSZ8794_REG_IAC1		0x6f    /* Indirect Access Control 0 */
#define KSZ8794_REG_IAD7		0x70    /* Indirect Access Data 7 */
#define KSZ8794_REG_IAD6		0x71    /* Indirect Access Data 6 */
#define KSZ8794_REG_IAD5		0x72    /* Indirect Access Data 5 */
#define KSZ8794_REG_IAD4		0x73    /* Indirect Access Data 4 */
#define KSZ8794_REG_IAD3		0x74    /* Indirect Access Data 3 */
#define KSZ8794_REG_IAD2		0x75    /* Indirect Access Data 2 */
#define KSZ8794_REG_IAD1		0x76    /* Indirect Access Data 1 */
#define KSZ8794_REG_IAD0		0x77    /* Indirect Access Data 0 */

#define KSZ8864_REG_ID1		0xfe	/* Chip ID in bit 7 */

#define KSZ8794_REGS_SIZE	0x80
#define KSZ8864_REGS_SIZE	0x100
#define KSZ8795_REGS_SIZE	0x100

#define ID1_CHIPID_M		0xf
#define ID1_CHIPID_S		4
#define ID1_REVISION_M		0x7
#define ID1_REVISION_S		1
#define ID1_START_SW		1	/* start the switch */

#define FAMILY_KSZ8795		0x87
#define CHIPID_M			0
#define KSZ8794_CHIP_ID		0x06

#define KSZ8794_CMD_WRITE	0x02
#define KSZ8794_CMD_READ	0x03

#define KSZ8794_RESET_DELAY	10 /* usec */

/* Sadly... */
#define KSZ8794_SPI_BUS 	1
#define KSZ8794_SPI_CS		0
#define KSZ8794_SPI_SPEED	2000000
#define KSZ8794_SPI_MODE    0

struct ksz8794_dev {
	struct spi_slave *spi;

};

static struct ksz8794_dev KSZ8794_DEV;

static int ksz8794_spi_read_write(struct spi_slave *spi,
				const u16 cmd,
				u8 *data_out, u8 *data_in,
				size_t data_len)
{
	unsigned long flags = SPI_XFER_BEGIN;
	int ret;

	if (data_len == 0)
		flags |= SPI_XFER_END;

	ret = spi_xfer(spi, 16, &cmd, NULL, flags);
	if (ret) {
		debug("Failed to send command (2 bytes): %d\n", ret);
	} else if (data_len != 0) {
		ret = spi_xfer(spi, data_len * 8, data_out, data_in,
					SPI_XFER_END);
		if (ret)
			debug("Failed to transfer %zu bytes of data: %d\n",
			      data_len, ret);
	}

	return ret;
}

static inline u16 create_spi_cmd(struct ksz8794_dev *priv, u16 cmd,
				    u16 address)
{
	u16 result = cmd << 13;

	/* add address, leave turnaround (TR) bit in lsb */
	result |= address << 1;
	/* SPI protocol needs big endian */
	return cpu_to_be16(result);
}

static int ksz8794_read(struct ksz8794_dev *priv, u8 *buf,
		 unsigned offset, size_t count)
{
	u16 cmd;

	cmd = create_spi_cmd(priv, KSZ8794_CMD_READ, offset);
	return ksz8794_spi_read_write(priv->spi, cmd, NULL, buf, count);
}

static int ksz8794_write(struct ksz8794_dev *priv, u8 *buf,
		 unsigned offset, size_t count)
{
	u16 cmd;

	cmd = create_spi_cmd(priv, KSZ8794_CMD_WRITE, offset);
	return ksz8794_spi_read_write(priv->spi, cmd, buf, NULL, count);
}

static inline int ksz8794_read_reg(struct ksz8794_dev *priv, u8 addr, u8 *buf)
{
	int ret = ksz8794_read(priv, buf, addr, 1);
	//printf("ksz8794_read_reg: addr=0x%02x res=0x%02x\n", addr, *buf);
	return ret;
}

static inline int ksz8794_write_reg(struct ksz8794_dev *priv, u8 addr, u8 val)
{
	u8 buf = val;

	//printf("ksz8794_write_reg: addr=%d val=%02X\n", addr, val);
	return ksz8794_write(priv, &buf, addr, 1);
}

static int ksz8794_mdio_read(struct mii_dev *bus, int addr, int devad, int reg)
{
	struct ksz8794_dev *priv = bus->priv;
	int p = addr;
	u8 s0, s1, s2;
	int r = 0;

	//printf("ksz8794_mdio_read: addr=%d devad=%d reg=%d\n", addr, devad, reg);
	/* phyaddr is 1 - 3 */
	if (p < 1 || p > 3) {
		return 0xffff;
	}
	/* Simulate MIIM of this device */
	if (reg == MII_PHYSID1) return 0x0022;
	if (reg == MII_PHYSID2) return 0x1550;
	if (reg == MII_BMCR) {
		ksz8794_read_reg(priv, KSZ8794_REG_CONTROL9(p), &s0);
		ksz8794_read_reg(priv, KSZ8794_REG_CONTROL10(p), &s1);
		ksz8794_read_reg(priv, KSZ8794_REG_CONTROL11(p), &s2);
		if (!(s0 & 0x80)) r |= BMCR_ANENABLE;
		if (s0 & 0x40) r |= BMCR_SPEED100;
		if (s0 & 0x20) r |= BMCR_FULLDPLX;
		if (s1 & 0x80) r |= 0x0001; // led off
		if (s1 & 0x40) r |= 0x0002; // disable tx
		if (s1 & 0x20) r |= BMCR_ANRESTART;
		if (s1 & 0x08) r |= BMCR_PDOWN;
		if (s1 & 0x04) r |= 0x0008; // disable MDI/MDIX
		if (s1 & 0x02) r |= 0x0010; // force MDI
		if (s1 & 0x01) r |= BMCR_LOOPBACK;
		if (s2 & 0x20) r |= BMCR_ISOLATE;
		if (s2 & 0x10) r |= BMCR_RESET;
	} else if (reg == MII_BMSR) {
		r = BMSR_100FULL | BMSR_100HALF | BMSR_10FULL | BMSR_10HALF | BMSR_ANEGCAPABLE;
		ksz8794_read_reg(priv, KSZ8794_REG_STATUS2(p), &s0);
		if (s0 & 0x40) r |= BMSR_ANEGCOMPLETE;
		if (s0 & 0x20) r |= BMSR_LSTATUS;
	} else if (reg == MII_ADVERTISE) {
		/* Just hard code it */
		r = ADVERTISE_PAUSE_CAP | ADVERTISE_100FULL | ADVERTISE_10FULL | ADVERTISE_100HALF | ADVERTISE_10HALF | ADVERTISE_CSMA;
	} else if (reg == MII_LPA) {
		ksz8794_read_reg(priv, KSZ8794_REG_STATUS0(p), &s0);
		if (s0 & 0x01) r |= LPA_10HALF;
		if (s0 & 0x02) r |= LPA_10FULL;
		if (s0 & 0x04) r |= LPA_100HALF;
		if (s0 & 0x08) r |= LPA_100FULL;
		if (s0 & 0x20) r |= LPA_PAUSE_ASYM;
	} else if (reg == 0x1f) { /* PHY special control/status */
		/* TODO? */
	} else {
		return 0xffff;
	}

	return r;

}

static int ksz8794_mdio_write(struct mii_dev *bus, int addr, int devad, int reg, u16 val)
{
	struct ksz8794_dev *priv = bus->priv;
	int p = addr;
	u8 s0, s1, s2;

	//printf("ksz8794_mdio_write: addr=%d devad=%d reg=%d val=0x%04X\n", addr, devad, reg, val);
	/* phyaddr is 1 - 3 */
	if (p < 1 || p > 3) {
		return 0xffff;
	}
	/* Simulate MIIM of this device */
	if (reg == MII_BMCR) {
		ksz8794_read_reg(priv, KSZ8794_REG_CONTROL9(p), &s0);
		ksz8794_read_reg(priv, KSZ8794_REG_CONTROL11(p), &s2);
		s0 &= ~(0x80|0x40|0x20);
		s1 = 0;
		s2 &= ~(0x20|0x10);
		if (!(val & BMCR_ANENABLE)) s0 |= 0x80; // this bit is opposite in polarity
		if (val & BMCR_SPEED100) s0 |= 0x40;
		if (val & BMCR_FULLDPLX) s0 |= 0x20;
		if (val & 0x0001) s1 |= 0x80;
		if (val & 0x0002) s1 |= 0x40;
		if (val & BMCR_ANRESTART) s1 |= 0x20;
		if (val & BMCR_PDOWN) s1 |= 0x08;
		if (val & 0x0008) s1 |= 0x04;
		if (val & 0x0010) s1 |= 0x02;
		if (val & BMCR_LOOPBACK) s1 |= 0x01;
		if (val & BMCR_ISOLATE) s2 |= 0x20;
		if (val & BMCR_RESET) s2 |= 0x10;

		/* reset (if enabled) first */
		ksz8794_write_reg(priv, KSZ8794_REG_CONTROL11(p), s2);
		// Hang on
		udelay(KSZ8794_RESET_DELAY);
		ksz8794_write_reg(priv, KSZ8794_REG_CONTROL9(p), s0);
		ksz8794_write_reg(priv, KSZ8794_REG_CONTROL10(p), s1);
	}
	return 0;
}


static int ksz8794_mdio_init(const char *name, struct ksz8794_dev *priv)
{
	struct mii_dev *bus = mdio_alloc();
	u8 id0, id1;
	int ret;
	char *str;
	struct udevice *dev;
	struct spi_slave *slave;


	if (!bus) {
		debug("Failed to allocate MDIO bus\n");
		return -ENOMEM;
	}

	bus->read = ksz8794_mdio_read;
	bus->write = ksz8794_mdio_write;
	snprintf(bus->name, sizeof(bus->name), name);
	bus->priv = (void *)priv;


	//snprintf(name, sizeof(name), "ksz8794_spi_%d:%d", KSZ8794_SPI_BUS, KSZ8794_SPI_CS);
	str = strdup(name);
	if (!str)
		return -ENOMEM;

	ret = spi_get_bus_and_cs(KSZ8794_SPI_BUS, KSZ8794_SPI_CS, KSZ8794_SPI_SPEED, KSZ8794_SPI_MODE, "spi_generic_drv",
				 str, &dev, &slave);
	if (ret) {
		mdio_free(bus);
		return ret;
	}
	ret = spi_claim_bus(slave);
	if (ret) {
		mdio_free(bus);
		return ret;
	}
	priv->spi = slave;

	/* Now check for device */
	ksz8794_read_reg(priv, KSZ8794_REG_ID0, &id0);
	ksz8794_read_reg(priv, KSZ8794_REG_ID1, &id1);
	if (id0 != FAMILY_KSZ8795 || (id1 >> 4) != KSZ8794_CHIP_ID) {
		//printf("ksz8975_spi: invalid chip id0=%02X id1=%02X\n", id0, id1);
		mdio_free(bus);
		return -ENODEV;
	}

	if (mdio_register(bus) != 0) {
		spi_release_bus(priv->spi);
		mdio_free(bus);
		return -ENOENT;
	}

	/* Enable switch, set GMII ingress clock delay */
	ksz8794_write_reg(priv, 0x56, 0xff);
	ksz8794_write_reg(priv, KSZ8794_REG_ID1, ID1_START_SW);

	return 0;
}

static int ksz8794_probe(struct phy_device *phydev)
{
	//printf("ksz8794_probe\n");
	return 0;
}

static int ksz8794_config(struct phy_device *phydev)
{
	//printf("ksz8794_config\n");
	phy_write(phydev, MDIO_DEVAD_NONE, MII_BMCR, BMCR_RESET);

	genphy_config_aneg(phydev);

	return 0;
}

static int ksz8794_startup(struct phy_device *phydev)
{
	//printf("ksz8794_startup\n");
	// TODO: fetch and parse status?
	return 0;
}


static struct phy_driver ksz8794_spi_driver = {
	.name = "KSZ8794 Switch/PHY",
	.uid = 0x221550,
	.mask = 0xfffff0,
	.features = PHY_BASIC_FEATURES,
	.probe = &ksz8794_probe,
	.config = &ksz8794_config,
	.startup = &ksz8794_startup,
	.shutdown = &genphy_shutdown,
};

/* Called from phy.c during system init */
int phy_ksz8794_spi_init(void)
{
	int ret = 0;
	//printf("ksz8794_init\n");

	ret = ksz8794_mdio_init("ksz8794_spi", &KSZ8794_DEV);

	if (ret != 0) {
		return ret;
	}

	phy_register(&ksz8794_spi_driver);

	return 0;
}
