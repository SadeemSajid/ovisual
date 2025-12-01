#include <linux/module.h>
#include <linux/init.h>
#include <linux/usb.h>

/** DRIVER MODULE INFO *****/
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sadeem Sajid <sadeem.sajid@dreambigsemi.com>");
MODULE_DESCRIPTION("USB driver for the OVisual hardware.");
/****************************/

/** DEFINES *****/
#define KEYVIS_VID 0xDEAD
#define KEYVIS_PID 0xBEEF
/****************/

/** LOCAL PROTOTYPES *****/
static int keyvis_probe(struct usb_interface *intf, const struct usb_device_id *id);
static void keyvis_disconnect(struct usb_interface *intf);

static void keyvis_delete(struct kref *kref);
static int keyvis_open(struct inode *inode, struct file *file);
static ssize_t keyvis_write(struct file *file, const char *user_buffer,
			  size_t count, loff_t *ppos);
static void keyvis_write_bulk_callback(struct urb *urb);
static int keyvis_release(struct inode *inode, struct file *file);
/*************************/

/** LOCAL VARIABLES *****/
struct keyvis_usb {
	struct usb_device	*udev;			/* the usb device for this device */
	struct usb_interface	*interface;		/* the interface for this device */
	unsigned char           *bulk_in_buffer;	/* the buffer to receive data */
	size_t			bulk_in_size;		/* the size of the receive buffer */
	size_t			bulk_in_filled;		/* number of bytes in the buffer */
	size_t			bulk_in_copied;		/* already copied to user space */
	__u8			int_in_endpointAddr;	/* the address of the bulk in endpoint */
	__u8			int_out_endpointAddr;	/* the address of the bulk out endpoint */
	int			errors;			/* the last request tanked */
	bool			ongoing_read;		/* a read is going on */
	spinlock_t		err_lock;		/* lock for errors */
	struct kref		kref;
	struct mutex		io_mutex;		/* synchronize I/O with disconnect */
	unsigned long		disconnected:1;
	wait_queue_head_t	bulk_in_wait;		/* to wait for an ongoing read */
};
#define to_keyvis_dev(d) container_of(d, struct keyvis_usb, kref)

static const struct file_operations keyvis_fops = {
	.owner =	THIS_MODULE,
	.open =		keyvis_open,
	.write =	keyvis_write,
	.release =	keyvis_release,
	// .read =		skel_read,
	// .flush =	skel_flush,
	// .llseek =	noop_llseek,
};

/*
 * usb class driver info in order to get a minor number from the usb core,
 * and to have the device registered with the driver core
 */
static struct usb_class_driver keyvis_class = {
	.name =		"ovis%d",
	.fops =		&keyvis_fops,
	.minor_base =	192, /* standard */
};

static struct usb_device_id keyvis_ids[] = {
	{ USB_DEVICE(KEYVIS_VID, KEYVIS_PID) },
	{},
};
MODULE_DEVICE_TABLE(usb, keyvis_ids);

static const char driver_name[] = "ovis";

static struct usb_driver keyvis_driver = {
	.name = driver_name,
	.id_table = keyvis_ids,
	.probe = keyvis_probe,
	.disconnect = keyvis_disconnect,
};
// module_usb_driver(keyvis_driver);
/************************/

/** FUNCTION IMPLEMENTATIONS *****/
static int keyvis_probe(struct usb_interface *intf, const struct usb_device_id *id)
{
	printk(KERN_DEBUG "probing...\n");

	struct keyvis_usb *dev;
	struct usb_endpoint_descriptor *int_in, *int_out;
	int retval;

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	kref_init(&dev->kref);
	mutex_init(&dev->io_mutex);
	spin_lock_init(&dev->err_lock);
	init_waitqueue_head(&dev->bulk_in_wait);

	dev->udev = usb_get_dev(interface_to_usbdev(intf));
	dev->interface = usb_get_intf(intf);

	/* set up the endpoint information */
	/* use only the first bulk-in and bulk-out endpoints */
	retval = usb_find_common_endpoints(intf->cur_altsetting,
			NULL, NULL, &int_in, &int_out);
	if (retval) {
		dev_err(&intf->dev,
			"Could not find both bulk-in and bulk-out endpoints\n");
		goto error;
	}

	dev->bulk_in_size = usb_endpoint_maxp(int_in);
	dev->int_in_endpointAddr = int_in->bEndpointAddress;
	dev->bulk_in_buffer = kmalloc(dev->bulk_in_size, GFP_KERNEL);
	if (!dev->bulk_in_buffer) {
		retval = -ENOMEM;
		goto error;
	}

	dev->int_out_endpointAddr = int_out->bEndpointAddress;

	/* save our data pointer in this interface device */
	usb_set_intfdata(intf, dev);

	/* we can register the device now, as it is ready */
	retval = usb_register_dev(intf, &keyvis_class);
	if (retval) {
		/* something prevented us from registering this driver */
		dev_err(&intf->dev,
			"Not able to get a minor for this device.\n");
		usb_set_intfdata(intf, NULL);
		goto error;
	}

	/* let the user know what node this device is now attached to */
	dev_info(&intf->dev,
		 "USB device now attached to ovis%d",
		 intf->minor);
	return 0;

error:
	/* this frees allocated memory */
	kref_put(&dev->kref, keyvis_delete);

	return retval;
}

static void keyvis_disconnect(struct usb_interface *intf)
{
	printk(KERN_DEBUG "disconnecting...\n");

	struct keyvis_usb *dev;
	int minor = intf->minor;

	dev = usb_get_intfdata(intf);

	/* give back our minor */
	usb_deregister_dev(intf, &keyvis_class);

	/* prevent more I/O from starting */
	mutex_lock(&dev->io_mutex);
	dev->disconnected = 1;
	mutex_unlock(&dev->io_mutex);

	/* decrement our usage count */
	kref_put(&dev->kref, keyvis_delete);

	dev_info(&intf->dev, "OVisual USB #%d now disconnected", minor);
}

static void keyvis_delete(struct kref *kref)
{
	struct keyvis_usb *dev = to_keyvis_dev(kref);

	// usb_free_urb(dev->bulk_in_urb);
	usb_put_intf(dev->interface);
	usb_put_dev(dev->udev);
	kfree(dev->bulk_in_buffer);
	kfree(dev);
}

static int keyvis_open(struct inode *inode, struct file *file)
{
	struct keyvis_usb *dev;
	struct usb_interface *interface;
	int subminor;
	int retval = 0;

	subminor = iminor(inode);

	interface = usb_find_interface(&keyvis_driver, subminor);
	if (!interface) {
		pr_err("%s - error, can't find device for minor %d\n",
			__func__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	dev = usb_get_intfdata(interface);
	if (!dev) {
		retval = -ENODEV;
		goto exit;
	}

	/* increment our usage count for the device */
	kref_get(&dev->kref);

	/* save our object in the file's private structure */
	file->private_data = dev;

exit:
	return retval;
}

static ssize_t keyvis_write(struct file *file, const char *user_buffer,
			  size_t count, loff_t *ppos)
{
	struct keyvis_usb *dev;
	int retval = 0;
	struct urb *urb = NULL;
	char *buf = NULL;
	size_t writesize = min_t(size_t, count, (PAGE_SIZE - 512));

	dev = file->private_data;

	/* verify that we actually have some data to write */
	if (count == 0)
		goto exit;

	spin_lock_irq(&dev->err_lock);
	retval = dev->errors;
	if (retval < 0) {
		/* any error is reported once */
		dev->errors = 0;
		/* to preserve notifications about reset */
		retval = (retval == -EPIPE) ? retval : -EIO;
	}
	spin_unlock_irq(&dev->err_lock);
	if (retval < 0)
		goto error;

	/* create a urb, and a buffer for it, and copy the data to the urb */
	urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!urb) {
		retval = -ENOMEM;
		goto error;
	}

	buf = usb_alloc_coherent(dev->udev, writesize, GFP_KERNEL,
				 &urb->transfer_dma);
	if (!buf) {
		retval = -ENOMEM;
		goto error;
	}

	if (copy_from_user(buf, user_buffer, writesize)) {
		retval = -EFAULT;
		goto error;
	}

	/* this lock makes sure we don't submit URBs to gone devices */
	mutex_lock(&dev->io_mutex);
	if (dev->disconnected) {		/* disconnect() was called */
		mutex_unlock(&dev->io_mutex);
		retval = -ENODEV;
		goto error;
	}

	/* initialize the urb properly */
	usb_fill_int_urb(urb, dev->udev, usb_sndintpipe(dev->udev, dev->int_out_endpointAddr), buf, writesize,
			keyvis_write_bulk_callback, dev, 5);
	urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	/* send the data out the bulk port */
	retval = usb_submit_urb(urb, GFP_KERNEL);
	mutex_unlock(&dev->io_mutex);
	if (retval) {
		dev_err(&dev->interface->dev,
			"%s - failed submitting write urb, error %d\n",
			__func__, retval);
		goto error_unanchor;
	}

	/*
	 * release our reference to this urb, the USB core will eventually free
	 * it entirely
	 */
	usb_free_urb(urb);

	return writesize;

error_unanchor:
	usb_unanchor_urb(urb);
error:
	if (urb) {
		usb_free_coherent(dev->udev, writesize, buf, urb->transfer_dma);
		usb_free_urb(urb);
	}

exit:
	return retval;
}

static void keyvis_write_bulk_callback(struct urb *urb)
{
	struct keyvis_usb *dev;
	unsigned long flags;

	dev = urb->context;

	/* sync/async unlink faults aren't errors */
	if (urb->status) {
		if (!(urb->status == -ENOENT ||
		    urb->status == -ECONNRESET ||
		    urb->status == -ESHUTDOWN))
			dev_err(&dev->interface->dev,
				"%s - nonzero write bulk status received: %d\n",
				__func__, urb->status);

		spin_lock_irqsave(&dev->err_lock, flags);
		dev->errors = urb->status;
		spin_unlock_irqrestore(&dev->err_lock, flags);
	}

	/* free up our allocated buffer */
	usb_free_coherent(urb->dev, urb->transfer_buffer_length,
			  urb->transfer_buffer, urb->transfer_dma);
}

static int keyvis_release(struct inode *inode, struct file *file)
{
	struct keyvis_usb *dev;

	dev = file->private_data;
	if (dev == NULL)
		return -ENODEV;

	/* decrement the count on our device */
	kref_put(&dev->kref, keyvis_delete);
	return 0;
}
/*********************************/

static int __init entry(void)
{
	printk(KERN_INFO "OVisual P-Series v0.1 by Sadeem Sajid <sadeem.sajid@dreambigsemi.com>\n");
	int ret = 0;
	ret = usb_register(&keyvis_driver);

	if (ret) {
		printk(KERN_ERR "Could not register driver!\n");
	}

	return ret;
}

static void __exit dentry(void)
{
	printk(KERN_INFO "Unloading kernel...\n");
	usb_deregister(&keyvis_driver);
}

module_init(entry);
module_exit(dentry);
