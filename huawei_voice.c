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
static int huawei_voice_port_probe(struct usb_serial_port *port);
static void huawei_voice_indat_callback(struct urb *urb);
static void huawei_voice_outdat_callback(struct urb *urb);
static struct urb *huawei_voice_setup_urb(struct usb_serial_port *port,
				      int endpoint,
				      int dir, void *ctx, char *buf, int len,
				      void (*callback) (struct urb *));
static int huawei_voice_port_remove(struct usb_serial_port *port);
static void huawei_voice_close(struct usb_serial_port *port);

					
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
	.close             = huawei_voice_close,
	.dtr_rts	       = usb_wwan_dtr_rts,
	.write             = huawei_voice_write,
	.write_room        = usb_wwan_write_room,
	.chars_in_buffer   = usb_wwan_chars_in_buffer,
	.tiocmget          = usb_wwan_tiocmget,
	.tiocmset          = usb_wwan_tiocmset,
	.ioctl             = usb_wwan_ioctl,
	.attach            = huawei_voice_attach,
	.release           = huawei_voice_release,
	.port_probe        = huawei_voice_port_probe,
	.port_remove	   = huawei_voice_port_remove,
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

/* struct usb_wwan_port_private { */
struct huawei_voice_port_private {
	/* Input endpoints and buffer for this port */
	struct urb *in_urbs[N_IN_URB];
	u8 *in_buffer[N_IN_URB];
	/* Output endpoints and buffer for this port */
	struct urb *out_urbs[N_OUT_URB + 1];
	u8 *out_buffer[N_OUT_URB + 1];
	unsigned long out_busy;	/* Bit vector of URBs in use */
	struct usb_anchor delayed;

	/* Settings for the port */
	int rts_state;		/* Handshaking pins (outputs) */
	int dtr_state;
	int cts_state;		/* Handshaking pins (inputs) */
	int dsr_state;
	int dcd_state;
	int ri_state;

	unsigned long tx_start_time[N_OUT_URB + 1];
};

static void unbusy_queued_urb(struct urb *urb,
					struct huawei_voice_port_private *portdata)
{
	int i;

	for (i = 0; i < N_OUT_URB; i++) {
		if (urb == portdata->out_urbs[i]) {
			clear_bit(i, &portdata->out_busy);
			break;
		}
	}
	
	if (urb == portdata->out_urbs[N_OUT_URB]) {
		clear_bit(N_OUT_URB, &portdata->out_busy);
	}
}

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

static void huawei_voice_indat_callback(struct urb *urb)
{
	int err;
	int endpoint;
	struct usb_serial_port *port;
	struct device *dev;
	unsigned char *data = urb->transfer_buffer;
	int status = urb->status;

	endpoint = usb_pipeendpoint(urb->pipe);
	port = urb->context;
	dev = &port->dev;

	if (status) {
		dev_dbg(dev, "%s: nonzero status: %d on endpoint %02x.\n",
			__func__, status, endpoint);
	} else {
		if (urb->actual_length) {
			tty_insert_flip_string(&port->port, data,
					urb->actual_length);
			tty_flip_buffer_push(&port->port);
		} else
			dev_dbg(dev, "%s: empty read urb received\n", __func__);
	}
	/* Resubmit urb so we continue receiving */
	err = usb_submit_urb(urb, GFP_ATOMIC);
	if (err) {
		if (err != -EPERM) {
			dev_err(dev, "%s: resubmit read urb failed. (%d)\n",
				__func__, err);
			/* busy also in error unless we are killed */
			usb_mark_last_busy(port->serial->dev);
		}
	} else {
		usb_mark_last_busy(port->serial->dev);
	}
}

static void huawei_voice_outdat_callback(struct urb *urb)
{
	struct usb_serial_port *port;
	struct huawei_voice_port_private *portdata;
	struct usb_wwan_intf_private *intfdata;
	int i;

	port = urb->context;
	intfdata = usb_get_serial_data(port->serial);

	usb_serial_port_softint(port);
	usb_autopm_put_interface_async(port->serial->interface);
	portdata = usb_get_serial_port_data(port);
	spin_lock(&intfdata->susp_lock);
	intfdata->in_flight--;
	spin_unlock(&intfdata->susp_lock);

	for (i = 0; i < N_OUT_URB; ++i) {
		if (portdata->out_urbs[i] == urb) {
			smp_mb__before_atomic();
			clear_bit(i, &portdata->out_busy);
			break;
		}
	}
}

static int huawei_voice_probe(struct usb_serial *serial,
			const struct usb_device_id *id)
{
	struct usb_interface_descriptor *iface_desc =
				&serial->interface->cur_altsetting->desc;
	/*struct usb_device_descriptor *dev_desc = &serial->dev->descriptor;*/

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

static int huawei_voice_port_remove(struct usb_serial_port *port)
{
	int i;
	struct huawei_voice_port_private *portdata;

	portdata = usb_get_serial_port_data(port);
	usb_set_serial_port_data(port, NULL);

	for (i = 0; i < N_IN_URB; i++) {
		usb_free_urb(portdata->in_urbs[i]);
		free_page((unsigned long)portdata->in_buffer[i]);
	}
	for (i = 0; i < N_OUT_URB; i++) {
		usb_free_urb(portdata->out_urbs[i]);
		kfree(portdata->out_buffer[i]);
	}

	usb_free_urb(portdata->out_urbs[N_OUT_URB]);
	kfree(portdata->out_buffer[N_OUT_URB]);
	
	kfree(portdata);

	return 0;
}

static struct urb *huawei_voice_setup_urb(struct usb_serial_port *port,
				      int endpoint,
				      int dir, void *ctx, char *buf, int len,
				      void (*callback) (struct urb *))
{
	struct usb_serial *serial = port->serial;
	struct urb *urb;

	urb = usb_alloc_urb(0, GFP_KERNEL);	/* No ISO */
	if (!urb)
		return NULL;

	usb_fill_bulk_urb(urb, serial->dev,
			  usb_sndbulkpipe(serial->dev, endpoint) | dir,
			  buf, len, callback, ctx);

	return urb;
}

static int huawei_voice_port_probe(struct usb_serial_port *port)
{
	struct huawei_voice_port_private *portdata;
	struct urb *urb;
	u8 *buffer;
	int i;

	if (!port->bulk_in_size || !port->bulk_out_size)
		return -ENODEV;

	portdata = kzalloc(sizeof(*portdata), GFP_KERNEL);
	if (!portdata)
		return -ENOMEM;

	init_usb_anchor(&portdata->delayed);

	for (i = 0; i < N_IN_URB; i++) {
		buffer = (u8 *)__get_free_page(GFP_KERNEL);
		if (!buffer)
			goto bail_out_error;
		portdata->in_buffer[i] = buffer;

		urb = huawei_voice_setup_urb(port, port->bulk_in_endpointAddress,
						USB_DIR_IN, port,
						buffer, IN_BUFLEN,
						huawei_voice_indat_callback);
		portdata->in_urbs[i] = urb;
	}

	for (i = 0; i < N_OUT_URB; i++) {
		buffer = kmalloc(OUT_BUFLEN, GFP_KERNEL);
		if (!buffer)
			goto bail_out_error2;
		portdata->out_buffer[i] = buffer;

		urb = huawei_voice_setup_urb(port, port->bulk_out_endpointAddress,
						USB_DIR_OUT, port,
						buffer, OUT_BUFLEN,
						huawei_voice_outdat_callback);
		portdata->out_urbs[i] = urb;
	}
	
	/* Extra byte */
	buffer = kmalloc(OUT_BUFLEN, GFP_KERNEL);
	if (!buffer)
		goto bail_out_error2;
	portdata->out_buffer[N_OUT_URB] = buffer;

	urb = huawei_voice_setup_urb(port, port->bulk_out_endpointAddress,
					USB_DIR_OUT, port,
					buffer, OUT_BUFLEN,
					huawei_voice_outdat_callback);
					
	portdata->out_urbs[N_OUT_URB] = urb;
	/* End Extra byte */

	usb_set_serial_port_data(port, portdata);

	return 0;

bail_out_error2:
	for (i = 0; i < N_OUT_URB; i++) {
		usb_free_urb(portdata->out_urbs[i]);
		kfree(portdata->out_buffer[i]);
	}
bail_out_error:
	for (i = 0; i < N_IN_URB; i++) {
		usb_free_urb(portdata->in_urbs[i]);
		free_page((unsigned long)portdata->in_buffer[i]);
	}
	kfree(portdata);

	return -ENOMEM;
}

static void huawei_voice_close(struct usb_serial_port *port)
{
	int i;
	struct usb_serial *serial = port->serial;
	struct huawei_voice_port_private *portdata;
	struct usb_wwan_intf_private *intfdata = usb_get_serial_data(serial);
	struct urb *urb;

	portdata = usb_get_serial_port_data(port);

	/*
	 * Need to take susp_lock to make sure port is not already being
	 * resumed, but no need to hold it due to ASYNC_INITIALIZED.
	 */
	spin_lock_irq(&intfdata->susp_lock);
	if (--intfdata->open_ports == 0)
		serial->interface->needs_remote_wakeup = 0;
	spin_unlock_irq(&intfdata->susp_lock);

	for (;;) {
		urb = usb_get_from_anchor(&portdata->delayed);
		if (!urb)
			break;
		unbusy_queued_urb(urb, portdata);
		usb_autopm_put_interface_async(serial->interface);
	}

	for (i = 0; i < N_IN_URB; i++)
		usb_kill_urb(portdata->in_urbs[i]);
	for (i = 0; i < N_OUT_URB; i++)
		usb_kill_urb(portdata->out_urbs[i]);
		
	usb_kill_urb(portdata->out_urbs[N_OUT_URB]);
	
	usb_kill_urb(port->interrupt_in_urb);

	usb_autopm_get_interface_no_resume(serial->interface);
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
	struct huawei_voice_port_private *portdata =
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
	struct huawei_voice_port_private *portdata;
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
	struct huawei_voice_port_private *portdata;
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
		this_urb = portdata->out_urbs[N_OUT_URB];
		if (test_and_set_bit(N_OUT_URB, &portdata->out_busy)) {
			if (time_before(jiffies,
					portdata->tx_start_time[N_OUT_URB] + 10 * HZ))
				return count;
			usb_unlink_urb(this_urb);
			return count;
		}
		dev_dbg(&port->dev, "%s: endpoint %d buf %d\n", __func__,
			usb_pipeendpoint(this_urb->pipe), i);

		err = usb_autopm_get_interface_async(port->serial->interface);
		if (err < 0) {
			clear_bit(N_OUT_URB, &portdata->out_busy);
			return count;
		}

		 /* send the data */
		this_urb->transfer_buffer_length = 0;

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
					__func__, N_OUT_URB, err);
				clear_bit(N_OUT_URB, &portdata->out_busy);
				spin_lock_irqsave(&intfdata->susp_lock, flags);
				intfdata->in_flight--;
				spin_unlock_irqrestore(&intfdata->susp_lock,
						       flags);
				usb_autopm_put_interface_async(port->serial->interface);
				return count;
			}
		}

		portdata->tx_start_time[N_OUT_URB] = jiffies;
	}
	
	return count;
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
