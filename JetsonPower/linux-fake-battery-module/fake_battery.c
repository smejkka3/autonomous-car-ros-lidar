#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/vermagic.h>
#include <linux/uaccess.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/fs.h>      // Needed by filp
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/unistd.h>
#include <linux/fcntl.h>
#include <linux/syscalls.h>



#define MAX_BATTERY_VOLTAGE		20000 //in mV
#define MIN_BATTERY_VOLTAGE		18000 //in mV
#define BATTERY_VOLTAGE_INTERVAL 2000 //in mV just an shorthand for the difference between Max and Min
#define BATTERY_CAPACITY		//in mAh
#define TIMER_INTERVAL	1000	//in ms

const char PATH_TO_I2C_DEVICE_VOLTAGE[] = "/sys/devices/3160000.i2c/i2c-0/0-0041/iio_device1/in_voltage0_input";
const char PATH_TO_I2C_DEVICE_CURRENT[] = "/sys/devices/3160000.i2c/i2c-0/0-0041/iio:device1/in_current0_input";
const char Workspace_Name[] = "Battery Work Queue";

void read_value(const char *);
void work_poll_func(struct work_struct *);
ssize_t show_io_data_voltage(struct device * dev,struct device_attribute *attr,char * buf );
ssize_t store_io_data_voltage(struct device * dev,  struct device_attribute * arrr,const char * buf,size_t count);

static int battery_status		= POWER_SUPPLY_STATUS_CHARGING;
static int battery_health		= POWER_SUPPLY_HEALTH_GOOD;
static int battery_present		= 1;
static int battery_technology	= POWER_SUPPLY_TECHNOLOGY_LIPO;
static int battery_capacity		= 50;
static int battery_capacity_lvl	= POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
static long battery_value_voltage = 17000;
static long battery_value_current = 80;




static bool module_initialized;
struct device *io_device;
struct power_supply *battery_device;
struct attribute attr_struct;
struct device_attribute io_attr;
struct kobject *dev_kobj;
struct timer_list poll_timer;
char buf[128];



ssize_t show_io_data_voltage(struct device * dev,struct device_attribute *attr,char *buff){
	scnprintf(buff,10 * sizeof(char),"%ld\0",battery_value_voltage);
	return 10 * sizeof(char);
}

ssize_t store_io_data_voltage(struct device * dev,  struct device_attribute * arrr,const char * buff,size_t count){
	kstrtol(buff,10,&battery_value_voltage);
	printk("Value:%ld",battery_value_voltage);
	power_supply_changed(battery_device);
	return count;
}

struct workqueue_struct *my_workqueue;
DECLARE_DELAYED_WORK(poll_work, work_poll_func);

void work_poll_func(struct work_struct *data){
	//battery_value_voltage = read_value(PATH_TO_I2C_DEVICE_VOLTAGE);
	//battery_value_current = read_value(PATH_TO_I2C_DEVICE_CURRENT);
	printk(KERN_DEBUG "Hello From Work Handler:%ld %ld",battery_value_voltage,battery_value_current);
	queue_delayed_work(my_workqueue, &poll_work, msecs_to_jiffies(2000));
}


void read_value(const char *path)
{
// Create variables
	/*int i;
    struct file *f;
	long value;
    mm_segment_t fs;
    // To see in /var/log/messages that the module is operating
    // I am using Fedora and for the test I have chosen following file
    // Obviously it is much smaller than the 128 bytes, but hell with it =)
    for(i = 0;i < 128; i++){
		buf[i] = 0;
	}
	
	f = filp_open(path, O_RDONLY, 0);
    if(f == NULL)
        printk(KERN_ALERT "filp_open error!!.\n");
    else{
		printk(KERN_DEBUG "File Opened");
        // Get current segment descriptor
        fs = get_fs();
        // Set segment descriptor associated to kernel space
        set_fs(KERNEL_DS);
        // Read the file
        f->f_op->read(f, buf, 128, &f->f_pos);
        // Restore segment descriptor
		
        set_fs(fs);
        // See what we read from file
        //printk(KERN_DEBUG "buf:%s\n",buf);
    }
    filp_close(f,NULL);*/
}

void test_battery_external_power_changed(struct power_supply *psy){
    battery_capacity++;
    power_supply_changed(psy);
}


void onTimeInterrupt(unsigned long data){
	printk(KERN_DEBUG "Timer Called");
	//queue_work(my_workqueue, &poll_work);
	mod_timer( &poll_timer, jiffies + msecs_to_jiffies(TIMER_INTERVAL));
    //battery_value_voltage =	read_voltage(PATH_TO_I2C_DEVICE_VOLTAGE);
	//battery_value_current = read_voltage(PATH_TO_I2C_DEVICE_CURRENT);
	//power_supply_changed(battery_device);
	//schedule_work(&poll_work);
	
}


static int test_power_get_battery_property(struct power_supply *psy,
					   enum power_supply_property psp,
					   union power_supply_propval *val)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "DAI-Labor Tx2 Battery";
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "DAI-LABOR";
		break;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = UTS_RELEASE;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = battery_status;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = battery_health;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = battery_present;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = battery_technology;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = battery_capacity_lvl;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = 100 * (battery_value_voltage - MIN_BATTERY_VOLTAGE) / (BATTERY_VOLTAGE_INTERVAL);
		battery_capacity = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = 100;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		val->intval = 3300;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = 26;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = battery_value_voltage;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = battery_value_current;
		break;
	default:
		pr_info("%s: some properties deliberately report errors.\n",
			__func__);
		return -EINVAL;
	}
	return 0;
}



static enum power_supply_property test_power_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};


static const struct power_supply_desc test_power_desc = {
		.name = "test_battery",
		.type = POWER_SUPPLY_TYPE_BATTERY,
		.properties = test_power_battery_props,
		.num_properties = ARRAY_SIZE(test_power_battery_props),
		.get_property = test_power_get_battery_property,
        .external_power_changed = test_battery_external_power_changed,
};

static const struct power_supply_config test_power_config = {};

static int __init test_power_init(void)
{
	
	printk(KERN_DEBUG "Started Initializing Battery Driver Module");
	battery_device = power_supply_register(NULL,&test_power_desc,&test_power_config);

	if(IS_ERR(battery_device)){
		printk(KERN_ERR "ERROR REGISTERING POWER MODULE,EXITING...");
		return -1;
	}
	
	io_device = root_device_register("battery_IO_device");
	//my_workqueue = create_singlethread_workqueue(Workspace_Name);
	//queue_delayed_work(my_workqueue, &poll_work, msecs_to_jiffies(2000));
	
	//setup_timer(&poll_timer, onTimeInterrupt, 0);
	//mod_timer( &poll_timer, jiffies + msecs_to_jiffies(TIMER_INTERVAL));
	//read_voltage(PATH_TO_I2C_DEVICE_CURRENT);
	//read_value(PATH_TO_I2C_DEVICE_VOLTAGE);
	//printk(KERN_DEBUG "Value is %ld",*value_holder);
	//struct device batt_dev = battery_device->dev;
	attr_struct.name = "Voltage";
	attr_struct.mode = 0777;
	dev_kobj = &io_device->kobj;

	io_attr.attr = attr_struct;
	io_attr.show = show_io_data_voltage;
	io_attr.store = store_io_data_voltage;

	if(PTR_ERR(sysfs_create_file( dev_kobj, &io_attr.attr ))) {
		printk(KERN_ERR "ERROR CREATING SYSFS FILE");
		return -1;
	}else{
		printk(KERN_DEBUG "SUCCESSFULLY CREATED");
	}

	//printk(KERN_DEBUG "Dev Name %s",batt_dev.init_name);


	module_initialized = true;
	return 0;
}
module_init(test_power_init);

static void __exit test_power_exit(void)
{
	printk(KERN_WARNING "Tx2 Battery Driver is exiting... Battery may get damaged");
	power_supply_unregister(battery_device);
	sysfs_remove_file( dev_kobj, &io_attr.attr);
	//kobject_put(dev_kobj);
	root_device_unregister(io_device);
	//del_timer(&poll_timer);
	//flush_workqueue(my_workqueue);
	//destroy_workqueue(my_workqueue);
	//int i;

	/* Let's see how we handle changes... */
	//ac_online = 0;
	//usb_online = 0;
	//battery_status = POWER_SUPPLY_STATUS_DISCHARGING;
	//for (i = 0; i < ARRAY_SIZE(test_power_supplies); i++)
		//power_supply_changed(test_power_supplies[i]);
	//pr_info("%s: 'changed' event sent, sleeping for 10 seconds...\n",
		//__func__);
	//ssleep(10);

	//for (i = 0; i < ARRAY_SIZE(test_power_supplies); i++)
		//power_supply_unregister(test_power_supplies[i]);

	printk(KERN_DEBUG "Module has successfully Exited");
	module_initialized = false;
}
module_exit(test_power_exit);

#define param_check_ac_online(name, p) __param_check(name, p, void);
#define param_check_usb_online(name, p) __param_check(name, p, void);
#define param_check_battery_status(name, p) __param_check(name, p, void);
#define param_check_battery_present(name, p) __param_check(name, p, void);
#define param_check_battery_technology(name, p) __param_check(name, p, void);
#define param_check_battery_health(name, p) __param_check(name, p, void);
#define param_check_battery_capacity(name, p) __param_check(name, p, void);
#define param_check_battery_voltage(name, p) __param_check(name, p, void);

MODULE_DESCRIPTION("Power supply driver for testing");
MODULE_AUTHOR("Anton Vorontsov <cbouatmailru@gmail.com>");
MODULE_LICENSE("GPL");
