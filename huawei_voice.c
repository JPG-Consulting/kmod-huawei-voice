/*
  USB Driver for GSM modems

  Copyright (C) 2005  Matthias Urlichs <smurf@smurf.noris.de>

  This driver is free software; you can redistribute it and/or modify
  it under the terms of Version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  Portions copied from the Keyspan driver by Hugh Blemings <hugh@blemings.org>

  History: see the git log.

  Work sponsored by: Sigos GmbH, Germany <info@sigos.de>

  This driver exists because the "normal" serial driver doesn't work too well
  with GSM modems. Issues:
  - data loss -- one single Receive URB is not nearly enough
  - nonstandard flow (Option devices) control
  - controlling the baud rate doesn't make sense

  This driver is named "option" because the most common device it's
  used for is a PC-Card (with an internal OHCI-USB interface, behind
  which the GSM interface sits), made by Option Inc.

  Some of the "one port" devices actually exhibit multiple USB instances
  on the USB bus. This is not a bug, these ports are used for different
  device features.
*/

#define DRIVER_AUTHOR "Matthias Urlichs <smurf@smurf.noris.de>"
#define DRIVER_DESC "USB Driver for GSM modems"

#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include "usb-wwan.h"

/* Function prototypes */
static int  huawei_voice_probe(struct usb_serial *serial,
			const struct usb_device_id *id);
static int huawei_voice_attach(struct usb_serial *serial);
static void huawei_voice_release(struct usb_serial *serial);
static int huawei_voice_send_setup(struct usb_serial_port *port);
static void huawei_voice_instat_callback(struct urb *urb);
static int huawei_voice_write(struct tty_struct *tty, struct usb_serial_port *port,
		   const unsigned char *buf, int count);

/* Vendor and product IDs */
#define HUAWEI_VENDOR_ID			0x12D1
#define HUAWEI_PRODUCT_E600			0x1001

/* some devices interfaces need special handling due to a number of reasons */
enum huawei_voice_blacklist_reason {
		OPTION_BLACKLIST_NONE = 0,
		OPTION_BLACKLIST_SENDSETUP = 1,
		OPTION_BLACKLIST_RESERVED_IF = 2
};

#define MAX_BL_NUM  11
struct huawei_voice_blacklist_info {
	/* bitfield of interface numbers for OPTION_BLACKLIST_SENDSETUP */
	const unsigned long sendsetup;
	/* bitfield of interface numbers for OPTION_BLACKLIST_RESERVED_IF */
	const unsigned long reserved;
};

static const struct huawei_voice_blacklist_info four_g_w14_blacklist = {
	.sendsetup = BIT(0) | BIT(1),
};

static const struct usb_device_id huawei_voice_ids[] = {
	{ USB_DEVICE_AND_INTERFACE_INFO(HUAWEI_VENDOR_ID, HUAWEI_PRODUCT_E600, 0xff, 0xff, 0xff) },
	{ } /* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, huawei_voice_ids);

/* The card has three separate interfaces, which the serial driver
 * recognizes separately, thus num_port=1.
 */

static struct usb_serial_driver huawei_voice_1port_device = {
	.driver = {
		.owner =	THIS_MODULE,
		.name =		"huawei_voice1",
	},
	.description       = "Huawei GSM modem (1-port)",
	.id_table          = huawei_voice_ids,
	.num_ports         = 1,
	.probe             = huawei_voice_probe,
	.open              = usb_wwan_open,
	.close             = usb_wwan_close,
	.dtr_rts	       = usb_wwan_dtr_rts,
	.write             = huawei_voice_write,
	.write_room        = usb_wwan_write_room,
	.chars_in_buffer   = usb_wwan_chars_in_buffer,
	.tiocmget          = usb_wwan_tiocmget,
	.tiocmset          = usb_wwan_tiocmset,
	.ioctl             = usb_wwan_ioctl,
	.attach            = huawei_voice_attach,
	.release           = huawei_voice_release,
	.port_probe        = usb_wwan_port_probe,
	.port_remove	   = usb_wwan_port_remove,
	.read_int_callback = huawei_voice_instat_callback,
#ifdef CONFIG_PM
	.suspend           = usb_wwan_suspend,
	.resume            = usb_wwan_resume,
#endif
};

static struct usb_serial_driver * const serial_drivers[] = {
	&huawei_voice_1port_device, NULL
};

struct huawei_voice_private {
	u8 bInterfaceNumber;
};

module_usb_serial_driver(serial_drivers, huawei_voice_ids);

static bool is_blacklisted(const u8 ifnum, enum huawei_voice_blacklist_reason reason,
			   const struct huawei_voice_blacklist_info *blacklist)
{
	unsigned long num;
	const unsigned long *intf_list;

	if (blacklist) {
		if (reason == OPTION_BLACKLIST_SENDSETUP)
			intf_list = &blacklist->sendsetup;
		else if (reason == OPTION_BLACKLIST_RESERVED_IF)
			intf_list = &blacklist->reserved;
		else {
			BUG_ON(reason);
			return false;
		}

		for_each_set_bit(num, intf_list, MAX_BL_NUM + 1) {
			if (num == ifnum)
				return true;
		}
	}
	return false;
}

static int huawei_voice_probe(struct usb_serial *serial,
			const struct usb_device_id *id)
{
	struct usb_interface_descriptor *iface_desc =
				&serial->interface->cur_altsetting->desc;
	struct usb_device_descriptor *dev_desc = &serial->dev->descriptor;

	__u8 nintf;
	__u8 ifnum;
	
	/* Never bind to the CD-Rom emulation interface	*/
	if (iface_desc->bInterfaceClass == 0x08)
		return -ENODEV;

	/*
	 * Don't bind reserved interfaces (like network ones) which often have
	 * the same class/subclass/protocol as the serial interfaces.  Look at
	 * the Windows driver .INF files for reserved interface numbers.
	 */
	if (is_blacklisted(
		iface_desc->bInterfaceNumber,
		OPTION_BLACKLIST_RESERVED_IF,
		(const struct huawei_voice_blacklist_info *) id->driver_info))
		return -ENODEV;

	/*
	 * Don't bind network interface on Huawei E600, it is handled by
	 * a separate module.
	 */
	/*if (dev_desc->idVendor == cpu_to_le16(HUAWEI_VENDOR_ID) &&
	    dev_desc->idProduct == cpu_to_le16(HUAWEI_PRODUCT_E600) &&
	    serial->interface->cur_altsetting->desc.bInterfaceNumber != 1)
		return -ENODEV;	*/
    
	/* Store device id so we can use it during attach. */
	usb_set_serial_data(serial, (void *)id);

	return 0;
}

static int huawei_voice_attach(struct usb_serial *serial)
{
	struct usb_interface_descriptor *iface_desc;
	const struct usb_device_id *id;
	struct usb_wwan_intf_private *data;
	struct huawei_voice_private *priv;

	data = kzalloc(sizeof(struct usb_wwan_intf_private), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		kfree(data);
		return -ENOMEM;
	}

	/* Retrieve device id stored at probe. */
	id = usb_get_serial_data(serial);
	iface_desc = &serial->interface->cur_altsetting->desc;

	priv->bInterfaceNumber = iface_desc->bInterfaceNumber;
	data->private = priv;

	if (!is_blacklisted(iface_desc->bInterfaceNumber,
			OPTION_BLACKLIST_SENDSETUP,
			(struct huawei_voice_blacklist_info *)id->driver_info)) {
		data->send_setup = huawei_voice_send_setup;
	}
	spin_lock_init(&data->susp_lock);

	usb_set_serial_data(serial, data);

	return 0;
}

static void huawei_voice_release(struct usb_serial *serial)
{
	struct usb_wwan_intf_private *intfdata = usb_get_serial_data(serial);
	struct huawei_voice_private *priv = intfdata->private;

	kfree(priv);
	kfree(intfdata);
}

static void huawei_voice_instat_callback(struct urb *urb)
{
	int err;
	int status = urb->status;
	struct usb_serial_port *port = urb->context;
	struct device *dev = &port->dev;
	struct usb_wwan_port_private *portdata =
					usb_get_serial_port_data(port);

	dev_dbg(dev, "%s: urb %p port %p has data %p\n", __func__, urb, port, portdata);

	if (status == 0) {
		struct usb_ctrlrequest *req_pkt =
				(struct usb_ctrlrequest *)urb->transfer_buffer;

		if (!req_pkt) {
			dev_dbg(dev, "%s: NULL req_pkt\n", __func__);
			return;
		}
		if ((req_pkt->bRequestType == 0xA1) &&
				(req_pkt->bRequest == 0x20)) {
			int old_dcd_state;
			unsigned char signals = *((unsigned char *)
					urb->transfer_buffer +
					sizeof(struct usb_ctrlrequest));

			dev_dbg(dev, "%s: signal x%x\n", __func__, signals);

			old_dcd_state = portdata->dcd_state;
			portdata->cts_state = 1;
			portdata->dcd_state = ((signals & 0x01) ? 1 : 0);
			portdata->dsr_state = ((signals & 0x02) ? 1 : 0);
			portdata->ri_state = ((signals & 0x08) ? 1 : 0);

			if (old_dcd_state && !portdata->dcd_state)
				tty_port_tty_hangup(&port->port, true);
		} else {
			dev_dbg(dev, "%s: type %x req %x\n", __func__,
				req_pkt->bRequestType, req_pkt->bRequest);
		}
	} else if (status == -ENOENT || status == -ESHUTDOWN) {
		dev_dbg(dev, "%s: urb stopped: %d\n", __func__, status);
	} else
		dev_err(dev, "%s: error %d\n", __func__, status);

	/* Resubmit urb so we continue receiving IRQ data */
	if (status != -ESHUTDOWN && status != -ENOENT) {
		usb_mark_last_busy(port->serial->dev);
		err = usb_submit_urb(urb, GFP_ATOMIC);
		if (err)
			dev_dbg(dev, "%s: resubmit intr urb failed. (%d)\n",
				__func__, err);
	}
}

/** send RTS/DTR state to the port.
 *
 * This is exactly the same as SET_CONTROL_LINE_STATE from the PSTN
 * CDC.
*/
static int huawei_voice_send_setup(struct usb_serial_port *port)
{
	struct usb_serial *serial = port->serial;
	struct usb_wwan_intf_private *intfdata = usb_get_serial_data(serial);
	struct huawei_voice_private *priv = intfdata->private;
	struct usb_wwan_port_private *portdata;
	int val = 0;
	int res;

	portdata = usb_get_serial_port_data(port);

	if (portdata->dtr_state)
		val |= 0x01;
	if (portdata->rts_state)
		val |= 0x02;

	res = usb_autopm_get_interface(serial->interface);
	if (res)
		return res;

	res = usb_control_msg(serial->dev, usb_sndctrlpipe(serial->dev, 0),
				0x22, 0x21, val, priv->bInterfaceNumber, NULL,
				0, USB_CTRL_SET_TIMEOUT);

	usb_autopm_put_interface(serial->interface);

	return res;
}

static int huawei_voice_write(struct tty_struct *tty, struct usb_serial_port *port,
		   const unsigned char *buf, int count)
{
	struct usb_wwan_port_private *portdata;
	struct usb_wwan_intf_private *intfdata;
	struct huawei_voice_private *priv;
	int i;
	int left, todo;
	struct urb *this_urb = NULL;	/* spurious */
	int err;
	unsigned long flags;

	portdata = usb_get_serial_port_data(port);
	intfdata = usb_get_serial_data(port->serial);
	priv = intfdata->private;

	dev_dbg(&port->dev, "%s: write (%d chars)\n", __func__, count);

	i = 0;
	left = count;
	for (i = 0; left > 0 && i < N_OUT_URB; i++) {
		todo = left;
		if (todo > OUT_BUFLEN)
			todo = OUT_BUFLEN;

		this_urb = portdata->out_urbs[i];
		if (test_and_set_bit(i, &portdata->out_busy)) {
			if (time_before(jiffies,
					portdata->tx_start_time[i] + 10 * HZ))
				continue;
			usb_unlink_urb(this_urb);
			continue;
		}
		dev_dbg(&port->dev, "%s: endpoint %d buf %d\n", __func__,
			usb_pipeendpoint(this_urb->pipe), i);

		err = usb_autopm_get_interface_async(port->serial->interface);
		if (err < 0) {
			clear_bit(i, &portdata->out_busy);
			break;
		}

		/* send the data */
		memcpy(this_urb->transfer_buffer, buf, todo);
		this_urb->transfer_buffer_length = todo;

		spin_lock_irqsave(&intfdata->susp_lock, flags);
		if (intfdata->suspended) {
			usb_anchor_urb(this_urb, &portdata->delayed);
			spin_unlock_irqrestore(&intfdata->susp_lock, flags);
		} else {
			intfdata->in_flight++;
			spin_unlock_irqrestore(&intfdata->susp_lock, flags);
			err = usb_submit_urb(this_urb, GFP_ATOMIC);
			if (err) {
				dev_err(&port->dev,
					"%s: submit urb %d failed: %d\n",
					__func__, i, err);
				clear_bit(i, &portdata->out_busy);
				spin_lock_irqsave(&intfdata->susp_lock, flags);
				intfdata->in_flight--;
				spin_unlock_irqrestore(&intfdata->susp_lock,
						       flags);
				usb_autopm_put_interface_async(port->serial->interface);
				break;
			}
		}

		portdata->tx_start_time[i] = jiffies;
		buf += todo;
		left -= todo;
	}

	count -= left;
	dev_dbg(&port->dev, "%s: wrote (did %d)\n", __func__, count);

	/* E150 require write zero length frame after each 320 bytes 20ms of voice data written. */
	if (priv->bInterfaceNumber == 1) {

	}
	
	return count;
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
