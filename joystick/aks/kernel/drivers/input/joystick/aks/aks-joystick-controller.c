#include <linux/module.h>  
#include <linux/kernel.h>  
#include <linux/cdev.h>  
#include <linux/device.h>  
#include <linux/fs.h>            
#include <linux/slab.h>  
#include <asm/uaccess.h>
#include "aks_joystick_controller_dev.h"
    
dev_t id;  
struct cdev cdev;  
struct class *class;  
struct device *dev;  
    
char user_flag[100]; 

int work_mode_flag = 1;
    
#define DEVICE_NAME "aks_js_ctl"  
    
    
int aks_joystick_controller_open (struct inode *inode, struct file *filp)  
{  
    printk( "[AKSYS] open\n" );  
    memset( user_flag, 0, 0 );  
    
    return 0;  
}  
    
int aks_joystick_controller_close (struct inode *inode, struct file *filp)  
{  
    printk( "[AKSYS] close\n" );  
    return 0;  
}  
    
ssize_t aks_joystick_controller_read(struct file *filp, char *buf, size_t size, loff_t *offset)  
{  
    printk( "[AKSYS] aks_joystick_controller_read\n" );  
    printk( "[AKSYS] DEV : write something\n" );  
    printk( "[AKSYS] %s %dbytes\n", user_flag, strlen(user_flag) );  
    //int ret = copy_to_user( buf, user_flag, strlen(user_flag)+1 );  
    
    return size;  
}  
    
ssize_t aks_joystick_controller_write (struct file *filp, const char *buf, size_t size, loff_t *offset)  
{  

	printk( "[AKSYS] aks_joystick_controller_write\n" );

	if(size == 4){
		printk( "[AKSYS] GamePad mode\n");
		work_mode_flag = 1;
	}

	else if(size == 5){
		printk( "[AKSYS] Touchpad mode\n");
		work_mode_flag = 2;
	}

	printk( "[AKSYS] %d\n", work_mode_flag);
	
    return size;  
}  
    
long aks_joystick_controller_ioctl ( struct file *filp, unsigned int cmd, unsigned long arg)  
{  
    
    printk( "[AKSYS] ioctl\n" );  
    return 0;  
}  
    
    
struct file_operations aks_joystick_controller_fops =  
{  
    .owner           = THIS_MODULE,  
    .read            = aks_joystick_controller_read,       
    .write           = aks_joystick_controller_write,      
    .unlocked_ioctl  = aks_joystick_controller_ioctl,      
    .open            = aks_joystick_controller_open,       
    .release         = aks_joystick_controller_close,    
};  
    
int aks_joystick_controller_init(void)  
{  
    int ret;  
    
    ret = alloc_chrdev_region( &id, 0, 1, DEVICE_NAME );  
    if ( ret ){  
        printk( "alloc_chrdev_region error %d\n", ret );  
        return ret;  
    }  
    
    cdev_init( &cdev, &aks_joystick_controller_fops );  
    cdev.owner = THIS_MODULE;  
    
    ret = cdev_add( &cdev, id, 1 );  
    if (ret){  
        printk( "cdev_add error %d\n", ret );  
        unregister_chrdev_region( id, 1 );  
        return ret;  
    }  
    
    class = class_create( THIS_MODULE, DEVICE_NAME );  
    if ( IS_ERR(class)){  
        ret = PTR_ERR( class );  
        printk( "class_create error %d\n", ret );  
    
        cdev_del( &cdev );  
        unregister_chrdev_region( id, 1 );  
        return ret;  
    }  
    
    dev = device_create( class, NULL, id, NULL, DEVICE_NAME );  
    if ( IS_ERR(dev) ){  
        ret = PTR_ERR(dev);  
        printk( "device_create error %d\n", ret );  
    
        class_destroy(class);  
        cdev_del( &cdev );  
        unregister_chrdev_region( id, 1 );  
        return ret;  
    }  
    
    
    return 0;  
}  
    
void aks_joystick_controller_exit(void)  
{  
    device_destroy(class, id );  
    class_destroy(class);  
    cdev_del( &cdev );  
    unregister_chrdev_region( id, 1 );  
}

EXPORT_SYMBOL(work_mode_flag);
    
module_init(aks_joystick_controller_init);  
module_exit(aks_joystick_controller_exit);  
    
MODULE_LICENSE("GPL  2.0");
