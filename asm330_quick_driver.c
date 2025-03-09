#include <linux/module.h> /* this is a kernel module */
#include <linux/uaccess.h> /* for copy_from_user */
#include <linux/kernel.h> /* we're doing kernel work */
#include <linux/version.h> /* miight need to check version for different kernel code */
#include <linux/spi/spi.h> 	/* since we are using spi for com */
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "asm330lhh_reg.h" /* the asm330 sensor interaction file */

/* Meta Information */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hardy Yu");
MODULE_DESCRIPTION("An IMU driver for asm330 with minimal code");


/* Defines for device identification */
#define SLAVE_DEVICE_NAME	"my_asm330"

stmdev_ctx_t asm330_ctx;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
    struct spi_transfer xfer[2];
    struct spi_message msg;
    struct spi_device *spi = (struct spi_device *)handle;
    
    spi_message_init(&msg);

    // First transfer: Register address
    memset(&xfer[0], 0, sizeof(xfer[0]));
    xfer[0].tx_buf = &reg;
    xfer[0].len = 1;
    spi_message_add_tail(&xfer[0], &msg);

    // Second transfer: Data payload
    memset(&xfer[1], 0, sizeof(xfer[1]));
    xfer[1].tx_buf = bufp;
    xfer[1].len = len;
    spi_message_add_tail(&xfer[1], &msg);

    // Perform SPI transaction
    if (spi_sync(spi, &msg)) {
        pr_err("ASM330 Driver: SPI write failed\n");
        return -1;
    }
    return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
    struct spi_transfer xfer;
    struct spi_message msg;
    struct spi_device *spi = (struct spi_device *)handle;
    
    uint8_t txrx_buffer[len + 1]; // First byte for register address, rest for received data
    memset(txrx_buffer, 0, sizeof(txrx_buffer));

    txrx_buffer[0] = reg | 0x80; // Set read bit

    spi_message_init(&msg);

    memset(&xfer, 0, sizeof(xfer));
    xfer.tx_buf = txrx_buffer; // Send register address, receive response in same transaction
    xfer.rx_buf = txrx_buffer;
    xfer.len = len + 1; // One byte for register, rest for received data
    spi_message_add_tail(&xfer, &msg);

    if (spi_sync(spi, &msg)) {
        pr_err("ASM330 Driver: SPI read failed\n");
        return -1;
    }

    // Copy received data into bufp (skip first byte which was the sent register)
    memcpy(bufp, &txrx_buffer[1], len);

    return 0;
}

static int asm330_read_raw(struct iio_dev * indio_dev, struct iio_chan_spec const * chan,
                             int *val, int *val2, long mask) {
    if (mask == IIO_CHAN_INFO_RAW){
        asm330lhh_status_reg_t status_reg;
        int16_t data_raw_acceleration[3];
        int16_t data_raw_angular_rate[3];
    
    
        asm330lhh_status_reg_get(&asm330_ctx, &status_reg);
        if (status_reg.xlda) {
            asm330lhh_acceleration_raw_get(&asm330_ctx, data_raw_acceleration);
        }
        if (status_reg.gda) {
            asm330lhh_angular_rate_raw_get(&asm330_ctx, data_raw_angular_rate);
        }

        pr_info("ASM330 read raw: accel[0] %hd\n", data_raw_acceleration[0]);
        pr_info("ASM330 read raw: accel[1] %hd\n", data_raw_acceleration[1]);
        pr_info("ASM330 read raw: accel[2] %hd\n", data_raw_acceleration[2]);
        pr_info("ASM330 read raw: gyro[0] %hd\n", data_raw_angular_rate[0]);
        pr_info("ASM330 read raw: gyro[1] %hd\n", data_raw_angular_rate[1]);
        pr_info("ASM330 read raw: gyro[2] %hd\n", data_raw_angular_rate[2]);

    } else {
        return -EINVAL;
    }
	return IIO_VAL_INT;
}

static const struct iio_chan_spec asm_channels[] = {
	{
		.type = IIO_ACCEL,
		.modified = 1,
        .channel2 = IIO_MOD_X,
        .scan_index = 0,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
				BIT(IIO_CHAN_INFO_SAMP_FREQ),
        .scan_type = {
            .sign = 's',
            .realbits = 16,
            .storagebits = 16,
            .shift = 0,
            .endianness = IIO_LE,
        },
	}, 
    {
        .type = IIO_ACCEL,
		.modified = 1,
        .channel2 = IIO_MOD_Y,
        .scan_index = 1,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
				BIT(IIO_CHAN_INFO_SAMP_FREQ),
        .scan_type = {
            .sign = 's',
            .realbits = 16,
            .storagebits = 16,
            .shift = 0,
            .endianness = IIO_LE,
        },
    },
    {
        .type = IIO_ACCEL,
		.modified = 1,
        .channel2 = IIO_MOD_Z,
        .scan_index = 2,
        .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |
				BIT(IIO_CHAN_INFO_SAMP_FREQ),
        .scan_type = {
            .sign = 's',
            .realbits = 16,
            .storagebits = 16,
            .shift = 0,
            .endianness = IIO_LE,
        },
    },
    IIO_CHAN_SOFT_TIMESTAMP(3),
};

static const struct iio_info asm330_iio_info = {
	.read_raw = asm330_read_raw,
};

static int asm330_probe(struct spi_device *spi) {
    /* Setting up IIO dev */
    struct iio_dev *indio_dev;

	printk("dt_iio - Now I am in the Probe function!\n");

	indio_dev = iio_device_alloc(&spi->dev, sizeof(struct iio_dev));
	if(!indio_dev) {
		printk("dt_iio - Error! Out of memory\n");
		return -ENOMEM;
	}

	indio_dev->name = "asm330_iio";
	indio_dev->info = &asm330_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = asm_channels;
	indio_dev->num_channels = ARRAY_SIZE(asm_channels);

	spi_set_drvdata(spi, indio_dev);
    int ret = iio_device_register(indio_dev);
    if (ret < 0 ){
        pr_err("Unable to register iio device for asm330\n");
        return ret;
    }

    /* Interact with IMU*/
    pr_info("ASM330 Driver: Probing device\n");
    asm330_ctx.write_reg = platform_write;
    asm330_ctx.read_reg = platform_read;
    asm330_ctx.handle = spi;
    uint8_t asm330_wai;
    asm330lhh_device_id_get(&asm330_ctx, &asm330_wai);
    pr_info("Read ASM330 WHOAMI reg: 0x%x\n", asm330_wai);
    uint8_t rst;
    asm330lhh_reset_set(&asm330_ctx, PROPERTY_ENABLE);
    asm330lhh_reset_get(&asm330_ctx, &rst);
    pr_info("Read ASM330 RESET reg: 0x%x\n", rst);

    // Configure IMU
	asm330lhh_device_conf_set(&asm330_ctx, PROPERTY_ENABLE); 			// Set device configuration (no clue what this does)
	asm330lhh_block_data_update_set(&asm330_ctx, PROPERTY_ENABLE);		// Enable block data update
	asm330lhh_xl_data_rate_set(&asm330_ctx, ASM330LHH_XL_ODR_417Hz);	// TODO: determine the correct output rates
	asm330lhh_gy_data_rate_set(&asm330_ctx, ASM330LHH_GY_ODR_417Hz);
	asm330lhh_xl_full_scale_set(&asm330_ctx, ASM330LHH_2g);			// TODO: determine the correct scaling
	asm330lhh_gy_full_scale_set(&asm330_ctx, ASM330LHH_2000dps);
	asm330lhh_timestamp_set(&asm330_ctx, PROPERTY_ENABLE);				// Enable timestamping

    /* Configure filtering chain(No aux interface)
	 * Accelerometer - LPF1 + LPF2 path
	 */
	asm330lhh_xl_hp_path_on_out_set(&asm330_ctx, ASM330LHH_LP_ODR_DIV_100);
	asm330lhh_xl_filter_lp2_set(&asm330_ctx, PROPERTY_ENABLE);

    return 0; // Return 0 on success
}

static void asm330_remove(struct spi_device *spi) {
    pr_info("ASM330 Driver: Removing device\n");
    struct iio_dev * indio_dev = spi_get_drvdata(spi);
    iio_device_unregister(indio_dev);
    iio_device_free(indio_dev);
}

static const struct spi_device_id spi_asm330_ids[] = {
    { SLAVE_DEVICE_NAME, 0},
    {},
};

MODULE_DEVICE_TABLE(spi, spi_asm330_ids);

static struct of_device_id of_asm_ids[] = {
	{
		.compatible = "my_asm330",
	}, { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_asm_ids);

static struct spi_driver asm330_driver = {
	.driver = {
        .name = SLAVE_DEVICE_NAME,
        .of_match_table = of_asm_ids,
    },
    .probe = asm330_probe,
    .remove = asm330_remove,
    .id_table = spi_asm330_ids,
};


// static int __init ModuleInit(void){
//     pr_info("Just to print something\n");
// 	int ret = spi_register_driver(&asm330_driver);
// 	if (ret != 0) {
// 		pr_err("ASM330 Driver: Failed to register driver\n");
// 		return ret;
// 	}

    
// 	return 0;
// }

// static void __exit ModuleExit(void) {
// 	spi_unregister_driver(&asm330_driver);
// 	printk("Goodbye, Kernel\n");
// }

// module_init(ModuleInit);
// module_exit(ModuleExit);

module_spi_driver(asm330_driver)
