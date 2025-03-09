#include <linux/module.h> /* this is a kernel module */
#include <linux/uaccess.h> /* for copy_from_user */
#include <linux/kernel.h> /* we're doing kernel work */
#include <linux/version.h> /* miight need to check version for different kernel code */
#include <linux/spi.h> 	/* since we are using spi for com */
#include <linux/iio/iio.h> /* our driver is a industrial io device */
#include <linux/iio/sysfs.h> /* idk wh I need this yet */

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hardy Yu");
MODULE_DESCRIPTION("An IMU driver for asm330lhh w/ Kalman filter");


#define DRIVER_NAME "asm330"
#define DRIVE_CLASS "asm300Class"

static struct spi_adaptor * asm_spi_adaptor = NULL;

/* Defines for device identification */
#define SPI_BUS_AVAILABLE	1
#define SLAVE_DEVICE_NAME	"ASM330"
#define ASM330_CHIP_SELECT	NULL /* will be defined later */

/* Declare the probe and remove functions */
static int asm_spi_probe(struct spi_client *client, const struct spi_device_id *id);
static int asm_spi_remove(struct spi_client *client);

/* Creates SPI device */
static const struct spi_device_id asm_id[] = {
	{ SLAVE_DEVICE_NAME, 0},
	{ }
};

static struct spi_driver asm_driver = {
	.spi_device_id = asm_id,
	.probe = asm_spi_probe,
	.remoe = asm_spi_remove,
	.driver = {
		.name = SLAVE_DEVICE_NAME,
		.owner = THIS_MODULE
	}
};

static struct spi_board_info asm_spi_board_info = {
	.modalias = DRIVER_NAME,
	.max_speed_hz = 10000000, // 10MHz accoarding to datasheet
	.bus_num = MY_BUS_NUM 0,
	.chip_select = 0,
	.mode = 3,
};

const struct regmap_config asm330_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

MODULE_DEVICE_TABLE(spi, asm_id);

/* Dealing with IIO device setup here */
static int asm_iio_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
	       		int *val, int *val2, long mask) {
	// there will be some stuff here soon
	
	return IIO_VAL_INT;
}

static const struct iio_chan_spec asm_channels[] = {
	{
		.type = IIO_ACCEL,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	}
};

static const struct iio_info asm_iio_info = {
	.read_raw = asm_iio_read_raw,
};


/* Function definitions of some spi functions */
static int asm_spi_probe(struct spi_device *spi){
	struct iio_dev *indio_dev;
	struct stmdev_ctx_t *asm_ctx;

	printk("asm_iio - Now I am in the Probe function!\n");

	/* Get access to SPI bus */
        struct spi_master *master;
	msater = spi_busnum_to_master(MY_BUS_NUM);
	if(!master){
		printk("There is no spi bus with Bus Nr. %d\n", MY_BUS_NUM);
		return -1;
	}

	/* Create new SPI device */
	asm_spi_dev = spi_new_dev(master, &spi_device_info);
	if (!asm_spi_dev){
		printk("Could not create device!");
		return -1;
	}

	asm_spi_dev->bits_per_word = 8;

	/* Setup the bus for device's parameters */
	if (spi_setup(asm_spi_dev) != 0){
		printk("Could not change bus setup!");
		spi_unregister_device(asm_spi_dev);
		return -1;
	}

	regmap = devm_regmap_init_spi(spi, &asm330_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Failed to register spi regmap: %pe\n",
			regmap);
		return PTR_ERR(regmap);
	}

	const struct spi_device_id *id = spi_get_device_id(spi);
	if (id)
		name = id->name;
	else
		name = dev_name(&spi->dev);

	
	/* Setup iio device */
	// indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(struct 
	return 0;
}

