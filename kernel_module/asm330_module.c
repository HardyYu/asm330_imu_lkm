/*
 * ASM330LHH IMU Driver
 * Author: Hardy Yu
 * Date: Mar 8, 2025
 * Description: An IIO-based driver for the ASM330LHH 6-DOF sensor using SPI.
 */

#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "asm330lhh_reg.h"


/* ---------------------- Meta Information ---------------------- */
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hardy Yu");
MODULE_DESCRIPTION("An IMU driver for ASM330LHH 6-DOF sensor");


/* ---------------------- Device Definitions ---------------------- */
#define SLAVE_DEVICE_NAME	"my_asm330"

/* ---------------------- Data Structures ---------------------- */
struct asm330_iio_t {
    struct spi_device *asm330_spi;
    struct mutex mtx;
    stmdev_ctx_t asm330_ctx;
    asm330lhh_status_reg_t status_reg;
    uint32_t    timestamp;
    int16_t     accel[3];
    int16_t     gyro[3];
};

/* ---------------------- SPI Communication ---------------------- */
/**
 * @brief Write to ASM330 via SPI.
 * @param handle Pointer to SPI device.
 * @param reg Register address.
 * @param bufp Buffer containing data to write.
 * @param len Number of bytes to write.
 * @return 0 on success, -1 on failure.
 */
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


/**
 * @brief Read from ASM330 via SPI.
 * @param handle Pointer to SPI device.
 * @param reg Register address.
 * @param bufp Buffer to store received data.
 * @param len Number of bytes to read.
 * @return 0 on success, -1 on failure.
 */
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

/* ---------------------- IIO Read Function ---------------------- */
static int asm330_read_raw(struct iio_dev * indio_dev, struct iio_chan_spec const * chan,
                             int *val, int *val2, long mask) {
    struct asm330_iio_t *data = iio_priv(indio_dev);
    if (mask == IIO_CHAN_INFO_RAW){

        mutex_lock(&data->mtx);
        asm330lhh_status_reg_get(&data->asm330_ctx, &data->status_reg);
        if (data->status_reg.xlda) {
            asm330lhh_acceleration_raw_get(&data->asm330_ctx, data->accel);
        }
        if (data->status_reg.gda) {
            asm330lhh_angular_rate_raw_get(&data->asm330_ctx, data->gyro);
        }
        mutex_unlock(&data->mtx);
        switch (chan->scan_index){
            case 0: { *val = data->accel[0]; break;}
            case 1: {*val = data->accel[1];  break;}
            case 2: {*val = data->accel[2];  break;}
            case 3: {*val = data->gyro[0];  break;}
            case 4: {*val = data->gyro[1];  break;}
            case 5: {*val = data->gyro[2];  break;}
            default: {return -EINVAL;}
        }

    } else {
        return -EINVAL;
    }
	return IIO_VAL_INT;
}

/* ---------------------- Sensor Configuration ---------------------- */
static void asm330_configure_sensor(stmdev_ctx_t *asm330_ctx, struct spi_device *spi) {
    pr_info("ASM330 Driver: Probing device\n");
    
    asm330_ctx->write_reg = platform_write;
    asm330_ctx->read_reg = platform_read;
    asm330_ctx->handle = spi;
    uint8_t asm330_wai;
    asm330lhh_device_id_get(asm330_ctx, &asm330_wai);
    pr_info("Read ASM330 WHOAMI reg: 0x%x\n", asm330_wai);
    uint8_t rst;
    asm330lhh_reset_set(asm330_ctx, PROPERTY_ENABLE);
    asm330lhh_reset_get(asm330_ctx, &rst);
    pr_info("Read ASM330 RESET reg: 0x%x\n", rst);

    // Configure IMU
	asm330lhh_device_conf_set(asm330_ctx, PROPERTY_ENABLE); 			// Set device configuration (no clue what this does)
	asm330lhh_block_data_update_set(asm330_ctx, PROPERTY_ENABLE);		// Enable block data update
	asm330lhh_xl_data_rate_set(asm330_ctx, ASM330LHH_XL_ODR_417Hz);	// TODO: determine the correct output rates
	asm330lhh_gy_data_rate_set(asm330_ctx, ASM330LHH_GY_ODR_417Hz);
	asm330lhh_xl_full_scale_set(asm330_ctx, ASM330LHH_2g);			// TODO: determine the correct scaling
	asm330lhh_gy_full_scale_set(asm330_ctx, ASM330LHH_2000dps);
	asm330lhh_timestamp_set(asm330_ctx, PROPERTY_ENABLE);				// Enable timestamping

    /* Configure filtering chain(No aux interface)
	 * Accelerometer - LPF1 + LPF2 path
	 */
	asm330lhh_xl_hp_path_on_out_set(asm330_ctx, ASM330LHH_LP_ODR_DIV_100);
	asm330lhh_xl_filter_lp2_set(asm330_ctx, PROPERTY_ENABLE);
}

/* ---------------------- IIO Channel Specification ---------------------- */
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
    {
        .type = IIO_ANGL_VEL,
		.modified = 1,
        .channel2 = IIO_MOD_X,
        .scan_index = 3,
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
        .type = IIO_ANGL_VEL,
		.modified = 1,
        .channel2 = IIO_MOD_Y,
        .scan_index = 4,
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
        .type = IIO_ANGL_VEL,
		.modified = 1,
        .channel2 = IIO_MOD_Z,
        .scan_index = 5,
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
    IIO_CHAN_SOFT_TIMESTAMP(6),
};

static const struct iio_info asm330_iio_info = {
	.read_raw = asm330_read_raw,
};

/* ---------------------- SPI Probe Function ---------------------- */
static int asm330_probe(struct spi_device *spi) {
    /* Setting up IIO dev */
    struct iio_dev *indio_dev;
    struct asm330_iio_t *data;

	printk("dt_iio - Now I am in the Probe function!\n");

	indio_dev = iio_device_alloc(&spi->dev, sizeof(struct asm330_iio_t));
	if(!indio_dev) {
		printk("dt_iio - Error! Out of memory\n");
		return -ENOMEM;
	}

    data = iio_priv(indio_dev);

	indio_dev->name = "asm330_iio";
	indio_dev->info = &asm330_iio_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = asm_channels;
	indio_dev->num_channels = ARRAY_SIZE(asm_channels);

    mutex_init(&data->mtx);

	spi_set_drvdata(spi, indio_dev);
    int ret = iio_device_register(indio_dev);
    if (ret < 0 ){
        pr_err("Unable to register iio device for asm330\n");
        return ret;
    }

    /* configure initial setup for IMU*/
    asm330_configure_sensor(&data->asm330_ctx, spi);

    return 0; // Return 0 on success
}

static void asm330_remove(struct spi_device *spi) {
    pr_info("ASM330 Driver: Removing device\n");
    struct iio_dev * indio_dev = spi_get_drvdata(spi);
    iio_device_unregister(indio_dev);
    iio_device_free(indio_dev);
}


/* ---------------------- SPI Configurations ---------------------- */
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

module_spi_driver(asm330_driver)
