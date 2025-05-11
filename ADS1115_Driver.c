#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>

#define DRIVER_NAME "ADS1115_driver"
#define CLASS_NAME "ADS1115"
#define DEVICE_NAME "ADS1115"

static struct i2c_client *ADS1115_client;
static struct class* ADS1115_class = NULL;
static struct device* ADS1115_device = NULL;
static int major_number;
static int device_open_count = 0;

// Pointer register 
#define ADS1115_REG_POINTER_CONVERSION  0x00
#define ADS1115_REG_POINTER_CONFIG      0x01
#define ADS1115_REG_POINTER_LOW_THRESH  0x02
#define ADS1115_REG_POINTER_HIGH_THRESH 0x03
#define ADS1115_REG_POINTER_ALERT_MASK  0x04
#define ADS1115_REG_POINTER_ALERT_STATUS 0x05

// Config register masks 
#define ADS1115_REG_CONFIG_OS_IDLE      0x8000
#define ADS1115_REG_CONFIG_MUX_SHIFT    12
#define ADS1115_REG_CONFIG_PGA_MASK     0x0E00
#define ADS1115_REG_CONFIG_MODE_MASK    0x0100
#define ADS1115_REG_CONFIG_DR_MASK      0x00E0
#define ADS1115_REG_CONFIG_COMP_QUE_MASK 0x0003

//* MUX channel values 
#define ADS1115_MUX_AIN0_GND  (0x4 << ADS1115_REG_CONFIG_MUX_SHIFT)  //AIN0 - GND
#define ADS1115_MUX_AIN1_GND  (0x5 << ADS1115_REG_CONFIG_MUX_SHIFT)
#define ADS1115_MUX_AIN2_GND  (0x6 << ADS1115_REG_CONFIG_MUX_SHIFT)
#define ADS1115_MUX_AIN3_GND  (0x7 << ADS1115_REG_CONFIG_MUX_SHIFT)
#define ADS1115_MUX_AIN0_AIN1  (0x0 << ADS1115_REG_CONFIG_MUX_SHIFT)  // AIN0 - AIN1
#define ADS1115_MUX_AIN0_AIN3  (0x1 << ADS1115_REG_CONFIG_MUX_SHIFT)  // AIN0 - AIN3
#define ADS1115_MUX_AIN1_AIN3  (0x2 << ADS1115_REG_CONFIG_MUX_SHIFT)  // AIN1 - AIN3
#define ADS1115_MUX_AIN2_AIN3  (0x3 << ADS1115_REG_CONFIG_MUX_SHIFT)  // AIN2 - AIN3


// PGA values 
#define ADS1115_PGA_6_144V    0x0000  // ±6.144V (tương ứng với 1 bit = 0.1875mV)
#define ADS1115_PGA_4_096V    0x0200  // ±4.096V (tương ứng với 1 bit = 0.125mV)
#define ADS1115_PGA_2_048V    0x0400  // ±2.048V (tương ứng với 1 bit = 0.0625mV)
#define ADS1115_PGA_1_024V    0x0600  // ±1.024V (tương ứng với 1 bit = 0.03125mV)
#define ADS1115_PGA_0_512V    0x0800  // ±0.512V (tương ứng với 1 bit = 0.015625mV)
#define ADS1115_PGA_0_256V    0x0A00  // ±0.256V (tương ứng với 1 bit = 0.0078125mV)


//Mode
#define ADS1115_MODE_CONTINUOUS 0x0000
#define ADS1115_MODE_SINGLESHOT 0x0100

// Data rate 
#define ADS1115_DR_8SPS      0x0000  // 8 Samples Per Second
#define ADS1115_DR_16SPS     0x0020  // 16 Samples Per Second
#define ADS1115_DR_32SPS     0x0040  // 32 Samples Per Second
#define ADS1115_DR_64SPS     0x0060  // 64 Samples Per Second
#define ADS1115_DR_128SPS    0x0080  // 128 Samples Per Second
#define ADS1115_DR_250SPS    0x00A0  // 250 Samples Per Second
#define ADS1115_DR_475SPS    0x00C0  // 475 Samples Per Second
#define ADS1115_DR_860SPS    0x00E0  // 860 Samples Per Second

// ALERT Pin Configuration Masks 
#define ADS1115_REG_CONFIG_CMP_PIN_MASK   0x1000  // Alert Pin (Enabled/Disabled)
#define ADS1115_REG_CONFIG_CMP_MODE_MASK  0x2000  // Comparator Mode (Latching/Non-Latching)
#define ADS1115_REG_CONFIG_CMP_POL_MASK   0x4000  // Comparator Polarity (Active Low/High)


/* IOCTL */
#define ADS1115_IOCTL_MAGIC        'a'
#define ADS1115_IOCTL_READ_CONFIG  _IOWR(ADS1115_IOCTL_MAGIC, 1, struct ADS1115_read_config)
#define ADS1115_IOCTL_SET_THRESHOLDS _IOW(ADS1115_IOCTL_MAGIC, 2, struct ADS1115_thresh_config)
#define ADS1115_IOCTL_READ_ALERT   _IOWR(ADS1115_IOCTL_MAGIC, 3, uint8_t) 

struct ADS1115_read_config {
    uint8_t channel;
    uint16_t pga;
    uint16_t mode;
    uint16_t datarate;
    int16_t result;
};

struct ADS1115_thresh_config {
    int16_t low_threshold;
    int16_t high_threshold;
};

static uint16_t get_channel_mux(uint8_t channel)
{
    switch (channel) {
        case 0: return ADS1115_MUX_AIN0_GND;
        case 1: return ADS1115_MUX_AIN1_GND;
        case 2: return ADS1115_MUX_AIN2_GND;
        case 3: return ADS1115_MUX_AIN3_GND;
        case 4: return ADS1115_MUX_AIN0_AIN1;
        case 5: return ADS1115_MUX_AIN0_AIN3;
        case 6: return ADS1115_MUX_AIN1_AIN3;
        case 7: return ADS1115_MUX_AIN2_AIN3;
        default: return ADS1115_MUX_AIN0_GND;
    }
}
static int ADS1115_configure_and_read(struct ADS1115_read_config *cfg)
{
    uint16_t config = 0;
    uint8_t config_data[3];
    uint8_t conv_data[2];
    int ret;
    uint16_t raw_val;

    config = ADS1115_REG_CONFIG_OS_IDLE |
         get_channel_mux(cfg->channel) |
         (cfg->pga & ADS1115_REG_CONFIG_PGA_MASK) |
         (cfg->mode & ADS1115_REG_CONFIG_MODE_MASK) |
         (cfg->datarate & ADS1115_REG_CONFIG_DR_MASK) |  
         ADS1115_REG_CONFIG_COMP_QUE_MASK;


    config_data[0] = ADS1115_REG_POINTER_CONFIG;
    config_data[1] = (config >> 8) & 0xFF;
    config_data[2] = config & 0xFF;

    ret = i2c_master_send(ADS1115_client, config_data, sizeof(config_data));
    if (ret < 0) {
        printk(KERN_ERR "ADS1115: Failed to write config\n");
        return ret;
    }

    if (cfg->mode == ADS1115_MODE_SINGLESHOT)
        msleep(10);

    config_data[0] = ADS1115_REG_POINTER_CONVERSION;
    ret = i2c_master_send(ADS1115_client, config_data, 1);
    if (ret < 0) {
        printk(KERN_ERR "ADS1115: Failed to set pointer to conversion register\n");
        return ret;
    }

    ret = i2c_master_recv(ADS1115_client, conv_data, 2);
    if (ret < 0) {
        printk(KERN_ERR "ADS1115: Failed to read conversion\n");
        return ret;
    }

    raw_val = (conv_data[0] << 8) | conv_data[1];
    cfg->result = (int16_t)raw_val;

    return 0;
}
static int ADS1115_set_thresholds(int16_t low_threshold, int16_t high_threshold)
{
    uint8_t low_data[3];
    uint8_t high_data[3];
    int ret;

    // Ghi ngưỡng thấp vào thanh ghi LOW_THRESHOLD
    low_data[0] = ADS1115_REG_POINTER_LOW_THRESH;
    low_data[1] = (low_threshold >> 8) & 0xFF;
    low_data[2] = low_threshold & 0xFF;

    ret = i2c_master_send(ADS1115_client, low_data, 3);
    if (ret < 0) {
        printk(KERN_ERR "ADS1115: Failed to set low threshold\n");
        return ret;
    }

    // Ghi ngưỡng cao vào thanh ghi HIGH_THRESHOLD
    high_data[0] = ADS1115_REG_POINTER_HIGH_THRESH;
    high_data[1] = (high_threshold >> 8) & 0xFF;
    high_data[2] = high_threshold & 0xFF;

    ret = i2c_master_send(ADS1115_client, high_data, 3);
    if (ret < 0) {
        printk(KERN_ERR "ADS1115: Failed to set high threshold\n");
        return ret;
    }

    return 0;
}

static long ADS1115_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct ADS1115_read_config cfg;
    struct ADS1115_thresh_config thresholds;

    switch (cmd) {
        case ADS1115_IOCTL_READ_CONFIG:
            if (copy_from_user(&cfg, (void __user *)arg, sizeof(cfg)))
                return -EFAULT;

            if (cfg.channel > 3)
                return -EINVAL;

            if (ADS1115_configure_and_read(&cfg) < 0)
                return -EIO;

            if (copy_to_user((void __user *)arg, &cfg, sizeof(cfg)))
                return -EFAULT;

            break;
        case ADS1115_IOCTL_SET_THRESHOLDS:
            if (copy_from_user(&thresholds, (void __user *)arg, sizeof(thresholds)))
                return -EFAULT;

            if (ADS1115_set_thresholds(thresholds.low_threshold, thresholds.high_threshold) < 0)
                return -EIO;
            break;
        default:
            return -EINVAL;
    }

    return 0;
}

static int ADS1115_open(struct inode *inodep, struct file *filep)
{
    if (device_open_count > 0)
        return -EBUSY;
    device_open_count++;
    try_module_get(THIS_MODULE);
    printk(KERN_INFO "ADS1115 device opened\n");
    return 0;
}

static int ADS1115_release(struct inode *inodep, struct file *filep)
{
    device_open_count--;
    module_put(THIS_MODULE);
    printk(KERN_INFO "ADS1115 device closed\n");
    return 0;
}

static struct file_operations fops = {
    .open = ADS1115_open,
    .unlocked_ioctl = ADS1115_ioctl,
    .release = ADS1115_release,
};

static int ADS1115_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    ADS1115_client = client;


    // Create a char device
    major_number = register_chrdev(0, DEVICE_NAME, &fops);
    if (major_number < 0) {
        printk(KERN_ERR "Failed to register a major number\n");
        return major_number;
    }

    ADS1115_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(ADS1115_class)) {
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ERR "Failed to register device class\n");
        return PTR_ERR(ADS1115_class);
    }

    ADS1115_device = device_create(ADS1115_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
    if (IS_ERR(ADS1115_device)) {
        class_destroy(ADS1115_class);
        unregister_chrdev(major_number, DEVICE_NAME);
        printk(KERN_ERR "Failed to create the device\n");
        return PTR_ERR(ADS1115_device);
    }

    printk(KERN_INFO "ADS1115 driver installed\n");
    return 0;


}

static void ADS1115_remove(struct i2c_client *client)
{
    device_destroy(ADS1115_class, MKDEV(major_number, 0));
    class_unregister(ADS1115_class);
    class_destroy(ADS1115_class);
    unregister_chrdev(major_number, DEVICE_NAME);

    printk(KERN_INFO "ADS1115 driver removed\n");
}

static const struct of_device_id ADS1115_of_match[] = {
    { .compatible = "TexasInstrument,ADS1115", },
    { },
};
MODULE_DEVICE_TABLE(of, ADS1115_of_match);

static struct i2c_driver ADS1115_driver = {
    .driver = {
        .name   = DRIVER_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = of_match_ptr(ADS1115_of_match),
    },
    .probe      = ADS1115_probe,
    .remove     = ADS1115_remove,
};

static int __init ADS1115_init(void)
{
    printk(KERN_INFO "Initializing ADS1115 driver\n");
    return i2c_add_driver(&ADS1115_driver);
}

static void __exit ADS1115_exit(void)
{
    printk(KERN_INFO "Exiting ADS1115 driver\n");
    i2c_del_driver(&ADS1115_driver);
}

module_init(ADS1115_init);
module_exit(ADS1115_exit);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("ADS1115 I2C Client Driver with IOCTL Interface");
MODULE_LICENSE("GPL");