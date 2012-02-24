/* Copyright (c) 2012, Daniel Toussaint ,  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <asm/delay.h>
#include <asm/io.h>
#include <acpi/acpi_bus.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/delay.h>

#include "f75111.h"

#define PMBASE 0x40 /* Offset of the power management base register in Intel ICH8 */
#define INTGPE 0x11 /* GPE number for the interrupt routine ( 0x10 + 1 (GPIO 1 ) ) */

static const unsigned short normal_i2c[] = { 0x37, 0x17, I2C_CLIENT_END };


struct f75111_chip {
        struct gpio_chip                 gpio_chip;
        struct i2c_client               *client;
};


static u32 pmbaseaddress;

static int eventflag = 0;

static int f75111_write_value(struct i2c_client *client, u8 reg, u8 value);


/* Kernel Thread to check for GPE events */
struct task_struct *checkthread;
int gpe_flag_check_thread(void *data) {
	
	struct i2c_client *client = data; 
	struct device dev = client->dev;
	char *envp[2];
	envp[0] = "GPIODATA=input"; 
        envp[1] = NULL;
	
	while(1) {

		if (eventflag !=0 ) {
	
			/* send event to userspace */
       			 kobject_uevent_env( &dev.kobj, KOBJ_CHANGE, envp) ; 

			/* Clear the  irq input pins */
			f75111_write_value( client, EDGE_DETECT_STATUS, 0xff);
			eventflag =0;
		}

		msleep(20);
		if (kthread_should_stop()) break;

	}
}


static const struct pci_device_id ich8_lpc_pci_id[] = 
{
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_ICH8_4) },
}; 


static u32 event_handler ( acpi_handle gpe_device, u32 gpe_number, void *context)  {
	
	eventflag = 1;
#if 0
	char *envp[2];
	
	struct i2c_client *client = context;
	struct device dev = client->dev;

	envp[0] = "GPIODATA=input";
	envp[1] = NULL;

	kobject_uevent_env( &dev.kobj, KOBJ_CHANGE, envp) ; 
#endif

	return ACPI_INTERRUPT_HANDLED | ACPI_REENABLE_GPE;

}


static int __devinit ich8_lpc_probe(struct pci_dev *dev, const struct pci_device_id *id) {

	int status;
	status = pci_enable_device(dev); 
	if (status) {	
		dev_err( &dev->dev, "pci_enable_device failed \n");	
		return -EIO;
	}

	/* Get PMBASE */ 
	status = pci_read_config_dword( dev,PMBASE, &pmbaseaddress);
	pmbaseaddress &= 0x00000ff80;
	
	/* Set the GPE route */
	pci_write_config_dword( dev, 0xb8, 0x00000208);

	if (status) goto out;

	/* set pmbase gpe registers */
	outl(  inl(pmbaseaddress + 0x2c ) | 0x00020000 , pmbaseaddress + 0x2c );


out:
	if (status) {
		pci_disable_device(dev);
	}

	return status;

}

static void ich8_lpc_remove(struct pci_dev *dev) {

	pci_disable_device(dev);

}

static u8 f75111_read_value(struct i2c_client *client, u8 reg) {
        return i2c_smbus_read_byte_data(client, reg);
}

static int f75111_write_value(struct i2c_client *client, u8 reg, u8 value) {
        return i2c_smbus_write_byte_data(client, reg, value);
}

static int f75111_gpio_get(struct gpio_chip *gc, unsigned offset)
{
        struct f75111_chip *chip;
        int status = -EINVAL; 
	
	u8 byte; 

	if  (offset > 7 ) return -EINVAL;

        chip = container_of(gc, struct f75111_chip, gpio_chip);
	
	byte = f75111_read_value( chip->client , GPIO2_ISR);
	status = ( byte >> offset ) & 0x01; 

        return status;
}

static void f75111_gpio_set(struct gpio_chip *gc, unsigned offset, int val)
{
        struct f75111_chip *chip;
	u8 byte;

	if ( ( offset < 8 ) || ( offset > 15))  return -EINVAL; 
	
	offset = offset -8; // second byte lane

        chip = container_of(gc, struct f75111_chip, gpio_chip);
	
	byte = f75111_read_value(chip->client, GPIO1_ODR); 
	
	printk("toussi 1 %x \n", byte);
	if ( val == 0 ) byte = ( byte & ~(1  << offset) ); 
	else byte = ( byte | (1 << offset) );
	printk("toussi 2 %x \n", byte);
	
	f75111_write_value( chip->client, GPIO1_ODR, byte); 

}

static int f75111_gpio_direction_input(struct gpio_chip *gc, unsigned offset)
{
        struct f75111_chip *chip;

        chip = container_of(gc, struct f75111_chip, gpio_chip);

	if (offset < 8 ) return 0; else return -EINVAL;

}

static int f75111_gpio_direction_output(struct gpio_chip *gc,
                                        unsigned offset,
                                        int val)
{
        struct f75111_chip *chip;
        int status = -EINVAL;

        chip = container_of(gc, struct f75111_chip, gpio_chip);
	
	if (offset > 7 ) return 0; else return -EINVAL;

        return status;
}


static void f75111_init_chip(struct f75111_chip *chip,
                        struct i2c_client *client,
                        kernel_ulong_t driver_data) 
{

	// hardcode irq's functionality for now.
	
	f75111_write_value( client, CONFIG_FUNCTION, 0x50);
	f75111_write_value( client, SMI_ENABLE, 0xff ); /* Set all inputs to cause an interrupt */
	f75111_write_value( client, EDGE_DETECT_ENABLE, 0xff); /* Interrupts are EDGE */

	f75111_write_value ( client, GPIO1_OCR, 0xff);
	f75111_write_value( client, GPIO1_ODR, 0x00);
	
	
	/* Setup the GPIO object */
        chip->client                     = client;
        chip->gpio_chip.label            = client->name;
        chip->gpio_chip.direction_input  = f75111_gpio_direction_input;
        chip->gpio_chip.direction_output = f75111_gpio_direction_output;
        chip->gpio_chip.get              = f75111_gpio_get;
        chip->gpio_chip.set              = f75111_gpio_set;
        chip->gpio_chip.can_sleep        = 1;
        chip->gpio_chip.ngpio            = 16; //TODO : sometimes only 8 pins are routed in the h/w

}

static int f75111_detect(struct i2c_client *client, struct i2c_board_info *info) {

	strlcpy(info->type, "f75111", I2C_NAME_SIZE);
	return 0;
}


static int f75111_probe( struct i2c_client *client, const struct i2c_device_id *id) { 


	static const u32 i2c_funcs = I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WRITE_WORD_DATA;
	int rc;
	acpi_status status;

	struct f75111_chip *chip;


	if (!i2c_check_functionality(client->adapter, i2c_funcs))
                return -ENOSYS;
	
	chip = kzalloc(sizeof(struct f75111_chip), GFP_KERNEL);
        if (!chip)
                return -ENOMEM;
	
	f75111_init_chip(chip, client, id->driver_data);

	chip->client = client;

	rc = gpiochip_add(&chip->gpio_chip);
	if (rc < 0 ) goto addchip_fail;

	i2c_set_clientdata(client, chip);
	
	/* Attach to the GPE */
        status = acpi_install_gpe_handler( NULL, INTGPE, ACPI_GPE_LEVEL_TRIGGERED, &event_handler, client);

        if (status != AE_OK) {
                printk(" Couldn't claim the GPE \n");
        }
        acpi_enable_gpe( NULL, INTGPE);

	checkthread=kthread_run(gpe_flag_check_thread,client ,"GPE FLAG check thread");

	
	return 0;
	
addchip_fail:
	kfree(chip); 
	return rc; 
}


static int __devexit f75111_remove ( struct i2c_client *client) {

	struct f75111_chip *chip;
        int rc;

        chip = i2c_get_clientdata(client);
        rc = gpiochip_remove(&chip->gpio_chip);
        if (rc < 0)
                return rc;

        kfree(chip);

	return 0;
}

static struct pci_driver fmb290_pci_driver = {
	.name 		= KBUILD_MODNAME,
	.id_table 	= ich8_lpc_pci_id,
	.probe 		= ich8_lpc_probe,
	.remove 	= ich8_lpc_remove,

}; 

static const struct i2c_device_id f75111_id[] = {
        {"f75111", 0},
        {}
};


static struct i2c_driver f75111 = {
	.class          = I2C_CLASS_HWMON,
        .driver = {
                .name = "f75111",
                .owner = THIS_MODULE
        },
        .probe    = f75111_probe,
        .remove   = __devexit_p(f75111_remove),
        .id_table = f75111_id,
	.detect   = f75111_detect,
	.address_list = normal_i2c,
};

static int __init f75111_init(void) {

	int ret;

	/* Initialize the PCI registers */
	ret = pci_register_driver(&fmb290_pci_driver);
	if (ret) return ret;

	/* Register the i2c device */
        ret = i2c_add_driver( &f75111 );
        if (ret) return ret;

	return 0;
}

static void __exit f75111_exit(void) {

	i2c_del_driver(&f75111);
	pci_unregister_driver(&fmb290_pci_driver);
	acpi_remove_gpe_handler(NULL, INTGPE, &event_handler);
	kthread_stop(checkthread);
}

subsys_initcall(f75111_init);
module_exit(f75111_exit);

MODULE_ALIAS("i2c:f75111");
MODULE_AUTHOR("Daniel Toussaint <daniel@toucware.com>");
MODULE_DESCRIPTION("Fintek F75111 GPIO driver");
MODULE_LICENSE("GPL v2");


