
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/vermagic.h>









static int __init battery_module_init(void){
    struct power_supply *dev;
    printk(KERN_DEBUG "Initializing");
    dev = power_supply_get_by_name("test_battery");
    if(dev != NULL){
        printk(KERN_DEBUG "Device is found succesfully");
        power_supply_external_power_changed(dev);
        printk("Issued External power change");
    }else{
        printk(KERN_DEBUG "Device is not found");
    }


    printk("End of Initialization");
    return 0;
}



static void __exit battery_module_exit(void){

}

module_init(battery_module_init);
module_exit(battery_module_exit);

MODULE_DESCRIPTION("Power supply driver for testing");
MODULE_AUTHOR("Anton Vorontsov <cbouatmailru@gmail.com>");
MODULE_LICENSE("GPL");