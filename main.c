/*
 * Innodisk EMUC-B202 CAN interface driver (using tty line discipline)
 *
 * This file is derived from linux/drivers/net/slip/slip.c
 *
 * slip.c Authors  : Laurence Culhane <loz@holmes.demon.co.uk>
 *                   Fred N. van Kempen <waltje@uwalt.nl.mugnet.org>
 * slcan.c Author  : Oliver Hartkopp <socketcan@hartkopp.net>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307. You can also get it
 * at http://www.gnu.org/licenses/gpl.html
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/tty.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/version.h>
#include <linux/rtnetlink.h>
#include <linux/if_arp.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
  #include <linux/can/dev.h>
#endif

#include "transceive.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35)
  #define N_EMUC (NR_LDISCS - 1)
#else
  #define N_EMUC N_SLCAN
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 4, 0)
/* pcan_netdev_register() use alloc_candev() instead of alloc_netdev() */
#define USES_ALLOC_CANDEV

/* using alloc_candev() also means:
 * - don't care about LINUX_CAN_RESTART_TIMER (restart is handled by can_restart_work)
 */
#endif

#define INNO_XMIT_DELAY_CMD 0x14A9 /* in decimal: 5289+0 */
#define INNO_XMIT_MODEL_CMD 0x14AA /* in decimal: 5289+1 */
#define VER_LEN 16

/*
 *  v2.1: Joey modify first steady version
 *  v2.2: Fix Tx & Rx packet bug (command "ifconfig")
 *  v2.3: kernel version >= 4.11.9: dev->destructor -----> dev->priv_destructor
 *  v2.4: Let debug message define macro to Makefile (without compile)
 *        Add mutex lock & udelay in emuc_xmit() function
 *        Add ioctl (from user space) command for xmit_delay setting
 *  v2.5: EMUC device will auto active after setting up "two" CAN port by driver
 *  v2.6: If kernel version >= 5.4.0: emuc_alloc() use alloc_candev() instead of alloc_netdev()
 *        If kernel version < 3.6.0 use register_netdev(), else use register_candev()
 *        Fix RX RTR with data length
 *        emuc_receive_buf() add usleep_range(10, 100);
 *  V3.0: Start to support high level CANbus protocal
 *        Add created kernel object for checked device model
 *        Add ioctl (from user space) command for recorded device model name
 *        Fix auto active issue when multi canbus
 *  V3.1: Start to support ERROR auto return command
 *        usage: candump any,0~0,#20000004 -t z
 *        must use SocketCAN utility >= ver 3.1
 *  V3.2: kernel version >= 5.4.0: fix struct can_priv() iuuse
 *  V3.3: kernel version < 3.15.0: fix struct net_device_ops() iuuse
 *  V3.4: Fix stop/start CAN frequently to cause parse fail
 *        EMUC device will auto inactive after setting down "two" CAN port by driver
 *  V3.5: Modify emuc_ldisc structure for kernel version >= 5.13.0
 *  V3.6: Modify about related to tty_ldisc's function for kernel version >= 5.14.0
 *  V3.7: Modify about related to tty_ldisc's function for kernel version >= 5.19.0
 *
 */
/* module info. */
/*=====================================================================*/
MODULE_VERSION("v3.7");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Innodisk EMUC-B202 CAN interface driver");
MODULE_ALIAS("Innodisk EMUC-B202");
MODULE_AUTHOR("Innodisk");

/* entry (1) */
/*=====================================================================*/
static int  __init emuc_init(void);
static void __exit emuc_exit(void);

module_init(emuc_init);
module_exit(emuc_exit);

/* entry (2) */
/*=====================================================================*/
static int  emuc_open  (struct tty_struct *tty);
static void emuc_close (struct tty_struct *tty);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,16,0)
static void emuc_hangup(struct tty_struct *tty);
#else
static int  emuc_hangup(struct tty_struct *tty);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)
static int  emuc_ioctl (struct tty_struct *tty, unsigned int cmd, unsigned long arg);
#else
static int  emuc_ioctl (struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg);
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,6,0)
static void emuc_receive_buf (struct tty_struct *tty, const u8 *cp, const u8 *fp, size_t count);
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,14,0)
static void emuc_receive_buf (struct tty_struct *tty, const unsigned char *cp, const char *fp, int count);
#else
static void emuc_receive_buf (struct tty_struct *tty, const unsigned char *cp, char *fp, int count);
#endif
#endif

static void emuc_write_wakeup(struct tty_struct *tty);

static struct tty_ldisc_ops emuc_ldisc =
{
  .owner  = THIS_MODULE,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,13,0)
  .magic  = TTY_LDISC_MAGIC,
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,14,0)
  .num		= N_EMUC,
#endif
  .name   = "emuccan",

  .open   = emuc_open,
  .close  = emuc_close,
  .hangup = emuc_hangup,
  .ioctl  = emuc_ioctl,
  .receive_buf  = emuc_receive_buf,
  .write_wakeup = emuc_write_wakeup,
};

/* entry (3) */
/*=====================================================================*/
static int emuc_netdev_open (struct net_device *dev);
static int emuc_netdev_close(struct net_device *dev);
static netdev_tx_t emuc_xmit(struct sk_buff *skb, struct net_device *dev);
static int emuc_change_mtu  (struct net_device *dev, int new_mtu);

static struct net_device_ops emuc_netdev_ops =
{
  .ndo_open       = emuc_netdev_open,
  .ndo_stop       = emuc_netdev_close,
  .ndo_start_xmit = emuc_xmit,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,15,0)
  .ndo_change_mtu = emuc_change_mtu,
#endif
};

static void emuc_sync (void);
static int  emuc_alloc(dev_t line, EMUC_RAW_INFO *info);
static void emuc_setup(struct net_device *dev);
static void emuc_free_netdev(struct net_device *dev);

/* entry (4) */
/*=====================================================================*/
static ssize_t model_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf);
static void free_can_k_obj(void);

static struct kobj_attribute model_name_attribute = __ATTR(model_name, 0644, model_name_show, NULL);
static struct kobject *emuc_b202_kobj;

typedef struct
{
  char   model[VER_LEN];
  char   can_name[2][IFNAMSIZ+1];

} MODEL_INFO;

struct can_k_obj
{
  struct kobject *kobj;
  char can_name[IFNAMSIZ+1];
  char model[VER_LEN];
  struct can_k_obj *next;
};

typedef struct can_k_obj CAN_K_OBJ;


#if _DBG_FUNC
void print_func_trace (int line, const char *func); /* extern function */
#endif

unsigned long xmit_delay = 0;
int maxdev = 10;
__initconst const char banner[] = "emuc: EMUC-B202 SocketCAN interface driver\n";
struct net_device **emuc_devs;
CAN_K_OBJ *can_k_obj_head = NULL , *can_k_obj_current = NULL;

/*---------------------------------------------------------------------------------------------------*/
static int __init emuc_init (void)
{
  int  status;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif


  if(maxdev < 4)
    maxdev = 4; /* Sanity */

  printk(banner);
  printk(KERN_INFO "emuc: %d dynamic interface channels.\n", maxdev);

  emuc_devs = kzalloc(sizeof(struct net_device *)*maxdev, GFP_KERNEL);
  
  if(!emuc_devs)
    return -ENOMEM;

  /* Fill in our line protocol discipline, and register it */
  #if LINUX_VERSION_CODE >= KERNEL_VERSION(5,14,0)
    status = tty_register_ldisc(&emuc_ldisc);
  #else
    status = tty_register_ldisc(N_EMUC, &emuc_ldisc);
  #endif

  if(status)
  {
    printk(KERN_ERR "emuc: can't register line discipline\n");
    kfree(emuc_devs);
  }

  emuc_b202_kobj = kobject_create_and_add("emuc_b202", kernel_kobj);
  if(!emuc_b202_kobj)
    return -ENOMEM;

  return status;
}

/*---------------------------------------------------------------------------------------------------*/
static void __exit emuc_exit (void)
{
  int                 i = 0;
  int                 busy = 0;
  struct net_device  *dev;
  EMUC_RAW_INFO      *info;
  unsigned long       timeout = jiffies + HZ;

  xmit_delay = 0;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  if(emuc_devs == NULL)
    return;

  /* First of all: check for active disciplines and hangup them. */   
  do
  {
    if(busy)
      msleep_interruptible(100);

    busy = 0;

    for(i=0; i<maxdev; i++)
    {
      dev = emuc_devs[i];

      if(!dev)
        continue;

      info = ((EMUC_PRIV *) netdev_priv(dev))->info;

      spin_lock_bh(&info->lock);
      if (info->tty)
      {
        busy++;
        tty_hangup(info->tty);
      }
      spin_unlock_bh(&info->lock);

    }

  } while (busy && time_before(jiffies, timeout));

  /* FIXME: hangup is async so we should wait when doing this second phase */
  for(i=0; i<maxdev; i++)
  {
    dev = emuc_devs[i];

    if(!dev)
      continue;

    emuc_devs[i] = NULL;

    info = ((EMUC_PRIV *)netdev_priv(dev))->info;

    if(info->tty)
    {
      printk(KERN_ERR "%s: tty discipline still running\n", dev->name);

      /* Intentionally leak the control block. */

      #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,9)
      dev->priv_destructor = NULL;
      #else
      dev->destructor = NULL;
      #endif
    }

  #if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
    unregister_candev(dev);
  #else
    unregister_netdev(dev);
  #endif
  }

  kfree(emuc_devs);
  emuc_devs = NULL;

  free_can_k_obj();

  #if LINUX_VERSION_CODE >= KERNEL_VERSION(5,14,0)
    tty_unregister_ldisc(&emuc_ldisc);
  #else
    i = tty_unregister_ldisc(N_EMUC);
    if(i)
      printk(KERN_ERR "emuc: can't unregister ldisc (err %d)\n", i);
  #endif

} /* END: emuc_exit() */

/*---------------------------------------------------------------------------------------------------*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6,6,0)
static void emuc_receive_buf (struct tty_struct *tty, const u8 *cp, const u8 *fp, size_t count)
#else
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,14,0)
static void emuc_receive_buf (struct tty_struct *tty, const unsigned char *cp, const char *fp, int count)
#else
static void emuc_receive_buf (struct tty_struct *tty, const unsigned char *cp, char *fp, int count)
#endif
#endif
{
  EMUC_RAW_INFO *info = (EMUC_RAW_INFO *) tty->disc_data;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  if(!info || info->magic != EMUC_MAGIC || (!netif_running(info->devs[0]) && !netif_running(info->devs[1])))
    return;

  usleep_range(10, 100);

  while(count--)
  {
    if (fp && *fp++)
    {
      if (!test_and_set_bit(SLF_ERROR, &info->flags))
      {
        if (netif_running(info->devs[0]))
          info->devs[0]->stats.rx_errors++;

        if (netif_running(info->devs[1]))
          info->devs[1]->stats.rx_errors++;
      }

      cp++;
      continue;
    }

    emuc_unesc(info, *cp++);
  }
}

/*---------------------------------------------------------------------------------------------------*/
static int emuc_open (struct tty_struct *tty)
{
  int             err;
  EMUC_RAW_INFO  *info;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  if (!capable(CAP_NET_ADMIN))
    return -EPERM;

  if (tty->ops->write == NULL)
    return -EOPNOTSUPP;

  /* RTnetlink lock is misused here to serialize concurrent
     opens of emuc channels. There are better ways, but it is
     the simplest one.
   */
  rtnl_lock();

  /* Collect hanged up channels. */
  emuc_sync();

  info = tty->disc_data;
  err = -EEXIST;

  /* First make sure we're not already connected. */
  if(info && info->magic == EMUC_MAGIC)
    goto ERR_EXIT;

  /* OK. Allocate emuc info. */
  err = -ENOMEM;
  info = kzalloc(sizeof(*info), GFP_KERNEL);
  if(!info)
    goto ERR_EXIT;

  /* OK.  Find a free EMUC channel to use. */
  err = -ENFILE;
  if (emuc_alloc(tty_devnum(tty), info) != 0)
  {
    kfree(info);
    goto ERR_EXIT;
  }

  info->tty = tty;
  tty->disc_data = info;

  if (!test_bit(SLF_INUSE, &info->flags))
  {
    /* Perform the low-level EMUC initialization. */
    info->rcount = 0;
    info->xleft  = 0;

    set_bit(SLF_INUSE, &info->flags);

  #ifdef USES_ALLOC_CANDEV
    if(tty->dev)
    {
      SET_NETDEV_DEV(info->devs[0], tty->dev);
      SET_NETDEV_DEV(info->devs[1], tty->dev);
    }
    else
      goto ERR_EXIT;
  #endif /* USES_ALLOC_CANDEV */

    err = register_netdevice(info->devs[0]);
    if(err)
      goto ERR_FREE_CHAN;

    err = register_netdevice(info->devs[1]);
    if(err)
    {
      #if LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)
        unregister_netdev(info->devs[0]);
      #else
        unregister_candev(info->devs[0]);
      #endif
      goto ERR_FREE_CHAN;
    }
  }

  /* Done.  We have linked the TTY line to a channel. */
  rtnl_unlock();
  tty->receive_room = 65536;  /* We don't flow control */

  /* TTY layer expects 0 on success */
  return 0;

ERR_FREE_CHAN:
  info->tty = NULL;
  tty->disc_data = NULL;
  clear_bit(SLF_INUSE, &info->flags);

ERR_EXIT:
  rtnl_unlock();

  /* Count references from TTY module */
  return err;

} /* END: emuc_open() */

/*---------------------------------------------------------------------------------------------------*/
static void emuc_close (struct tty_struct *tty)
{
  EMUC_RAW_INFO *info = (EMUC_RAW_INFO *) tty->disc_data;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  /* First make sure we're connected. */
  if (!info || info->magic != EMUC_MAGIC || info->tty != tty)
    return;

  spin_lock_bh(&info->lock);
  tty->disc_data = NULL;
  info->tty = NULL;
  spin_unlock_bh(&info->lock);

  flush_work(&info->tx_work);

  /* Flush network side */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,6,0)
  unregister_candev(info->devs[0]);
  unregister_candev(info->devs[1]);
#else
  unregister_netdev(info->devs[0]);
  unregister_netdev(info->devs[1]);
#endif
  /* This will complete via emuc_free_netdev */
}



/*---------------------------------------------------------------------------------------------------*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,16,0)
static void emuc_hangup(struct tty_struct *tty)
#else
static int  emuc_hangup(struct tty_struct *tty)
#endif
{
#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  emuc_close(tty);

  #if LINUX_VERSION_CODE >= KERNEL_VERSION(5,16,0)
    return;
  #else
    return 0;
  #endif
}

/*---------------------------------------------------------------------------------------------------*/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)
static int emuc_ioctl (struct tty_struct *tty, unsigned int cmd, unsigned long arg)
#else
static int emuc_ioctl (struct tty_struct *tty, struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
  int            channel;
  unsigned int   tmp;
  unsigned long  ret_ulong;
  int            ret_int;
  EMUC_RAW_INFO *info = (EMUC_RAW_INFO *) tty->disc_data;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  /* First make sure we're connected. */
  if (!info || info->magic != EMUC_MAGIC)
    return -EINVAL;

  switch(cmd)
  {
    case INNO_XMIT_DELAY_CMD:
                        {
                          char delay_str[5]; /* 0 ~ 1000 */

                          ret_ulong = copy_from_user(delay_str, (void __user *)arg, 5);
                          if(ret_ulong)
                          {
                            printk(KERN_INFO "emuc: copy_from_user() failed (ret: %ld).\n", ret_ulong);
                            return -2;
                          }

                          delay_str[4] = '\0';

                          ret_int = kstrtol(delay_str, 10, &xmit_delay);
                          if(ret_int)
                          {
                            printk(KERN_INFO "emuc: kstrtol() failed (ret: %d).\n", ret_int);
                            return -3;
                          }

                          printk(KERN_INFO "----------> INNO_XMIT_DELAY_CMD ioctl(), xmit_delay = %lu\n", xmit_delay);
                          if(xmit_delay > 1000)
                          {
                            return -1;
                          }
                          return 0;
                        }

    case INNO_XMIT_MODEL_CMD:
                        {
                          /* create kernel object for checked device model */
                          int channel;
                          static MODEL_INFO model_info;

                          ret_ulong = copy_from_user(&model_info, (void __user *)arg, sizeof(MODEL_INFO));
                          if(ret_ulong)
                          {
                            printk(KERN_INFO "emuc: copy_from_user() failed (ret: %ld).\n", ret_ulong);
                            return -2;
                          }

                          printk(KERN_INFO "----------> INNO_XMIT_MODEL_CMD ioctl(), model:%s, can_name[0]:%s, can_name[1]:%s\n",
                                 model_info.model, model_info.can_name[0], model_info.can_name[1]);

                          for(channel = 0; channel < 2;)
                          {
                            if(!can_k_obj_head)
                            {
                              can_k_obj_head = (CAN_K_OBJ *) kmalloc(sizeof(CAN_K_OBJ), GFP_USER);
                              can_k_obj_head->kobj = kobject_create_and_add(model_info.can_name[channel], emuc_b202_kobj);
                              if(!can_k_obj_head->kobj)
                              {
                                free_can_k_obj();
                                return -ENOMEM;
                              }
                              if(sysfs_create_file(can_k_obj_head->kobj, &model_name_attribute.attr))
                                kobject_put(can_k_obj_head->kobj);
                              strcpy(can_k_obj_head->can_name, model_info.can_name[channel]);
                              strcpy(can_k_obj_head->model, model_info.model);
                              can_k_obj_head->next = NULL;
                              can_k_obj_current = can_k_obj_head;
                            }
                            else
                            {
                              can_k_obj_current->next = (CAN_K_OBJ *) kmalloc(sizeof(CAN_K_OBJ), GFP_USER);
                              can_k_obj_current = can_k_obj_current->next;
                              can_k_obj_current->kobj = kobject_create_and_add(model_info.can_name[channel], emuc_b202_kobj);
                              if(!can_k_obj_current->kobj)
                              {
                                free_can_k_obj();
                                return -ENOMEM;
                              }
                              if(sysfs_create_file(can_k_obj_current->kobj, &model_name_attribute.attr))
                                kobject_put(can_k_obj_current->kobj);
                              strcpy(can_k_obj_current->can_name, model_info.can_name[channel]);
                              strcpy(can_k_obj_current->model, model_info.model);
                              can_k_obj_current->next = NULL;
                            }
                            channel++;
                          }
                          
                          return 0;
                        }

    case SIOCGIFNAME:
                        {
                          channel = info->gif_channel;
                          tmp = strlen(info->devs[channel]->name) + 1;

                          if(copy_to_user((void __user *)arg, info->devs[channel]->name, tmp))
                            return -EFAULT;

                          info->gif_channel = !info->gif_channel;
                          return 0;
                        }

    case SIOCSIFHWADDR:
                        return -EINVAL;

    default:
                        #if LINUX_VERSION_CODE >= KERNEL_VERSION(5,17,0)
                          return tty_mode_ioctl(tty, cmd, arg);
                        #else
                          return tty_mode_ioctl(tty, file, cmd, arg);
                        #endif
  }
}

/*---------------------------------------------------------------------------------------------------*/
static void emuc_write_wakeup (struct tty_struct *tty)
{
  EMUC_RAW_INFO *info = tty->disc_data;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  schedule_work(&info->tx_work);
}

/*---------------------------------------------------------------------------------------------------*/
static int emuc_netdev_open (struct net_device *dev)
{
  int            channel;
  EMUC_RAW_INFO *info = ((EMUC_PRIV *) netdev_priv(dev))->info;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  if(info->tty == NULL)
    return -ENODEV;

  info->flags &= (1 << SLF_INUSE);
  netif_start_queue(dev);

  channel = (dev->base_addr & 0xF00) >> 8;

  if (netif_running(info->devs[!channel]))
  {
    emuc_initCAN(info, EMUC_ACTIVE);
    printk(KERN_INFO "emuc: Device will become active status.\n");
  }

  return 0;
}

/*---------------------------------------------------------------------------------------------------*/
static int emuc_netdev_close (struct net_device *dev)
{
  int             channel;
  EMUC_RAW_INFO  *info = ((EMUC_PRIV *) netdev_priv(dev))->info;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  channel = (dev->base_addr & 0xF00) >> 8;
  if(channel > 1)
  {
    printk(KERN_WARNING "%s: close: invalid channel\n", dev->name);
    return -1;
  }

  spin_lock_bh(&info->lock);

  if(info->tty)
  {
    /* TTY discipline is running. */
    if(!netif_running(info->devs[!channel]))
      clear_bit(TTY_DO_WRITE_WAKEUP, &info->tty->flags);
  }

  netif_stop_queue(dev);

  if (!netif_running(info->devs[!channel]))
  {
    /* another netdev is closed (down) too, reset TTY buffers. */
    info->rcount   = 0;
    info->xleft    = 0;

    if(info->tty)
    {
      emuc_initCAN(info, EMUC_INACTIVE);
      printk(KERN_INFO "emuc: Device will become inactive status.\n");
    }
  }

  spin_unlock_bh(&info->lock);
  return 0;
}

/*---------------------------------------------------------------------------------------------------*/
static netdev_tx_t emuc_xmit (struct sk_buff *skb, struct net_device *dev)
{
  int             channel;
  EMUC_RAW_INFO  *info = ((EMUC_PRIV *) netdev_priv(dev))->info;


#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  if(skb->len != sizeof(struct can_frame))
    goto OUT;

  spin_lock(&info->lock);

  if(xmit_delay)
  {
    udelay(xmit_delay);
  }

  if(!netif_running(dev))
  {
    spin_unlock(&info->lock);
    printk(KERN_WARNING "%s: xmit: iface is down\n", dev->name);
    goto OUT;
  }

  if(info->tty == NULL)
  {
    spin_unlock(&info->lock);
    goto OUT;
  }

  channel = (dev->base_addr & 0xF00) >> 8;

  if(channel > 1)
  {
    spin_unlock(&info->lock);
    printk(KERN_WARNING "%s: xmit: invalid channel\n", dev->name);
    goto OUT;
  }

  netif_stop_queue(info->devs[0]);
  netif_stop_queue(info->devs[1]);
  emuc_encaps(info, channel, (struct can_frame *) skb->data); /* encaps & send */
  spin_unlock(&info->lock);

OUT:
  kfree_skb(skb);
  return NETDEV_TX_OK;

} /* END: emuc_xmit() */

/*---------------------------------------------------------------------------------------------------*/
static int emuc_change_mtu (struct net_device *dev, int new_mtu)
{
#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  return -EINVAL;
}

/*---------------------------------------------------------------------------------------------------*/
static void emuc_sync (void)
{
  int                 i;
  struct net_device  *dev;
  EMUC_RAW_INFO      *info;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  for(i=0; i<maxdev; i++)
  {
    dev = emuc_devs[i];

    if(dev == NULL)
      break;

    info = ((EMUC_PRIV *) netdev_priv(dev))->info;
    
    if(info->tty)
      continue;

    if(dev->flags & IFF_UP)
      dev_close(dev);
  }
}

/*---------------------------------------------------------------------------------------------------*/
static int emuc_alloc (dev_t line, EMUC_RAW_INFO *info)
{
  int                 i;
  int                 channel = 0;
  int                 id[2];
  char                name[IFNAMSIZ];
  struct net_device  *dev;
  struct net_device  *devs[2];
  EMUC_PRIV          *priv;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  for(i=0; i<maxdev; i++)
  {
    dev = emuc_devs[i];

    if (dev == NULL)
    {
      id[channel++] = i;

      if(channel > 1)
        break;
    }
  }

  /* Sorry, too many, all slots in use */
  if(i >= maxdev)
    return -1;

  sprintf(name, "emuccan%d", id[0]);

  #ifdef USES_ALLOC_CANDEV
    devs[0] = alloc_candev(sizeof(*priv), 0);
  #else
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0)
      devs[0] = alloc_netdev(sizeof(*priv), name, emuc_setup);
    #else
      devs[0] = alloc_netdev(sizeof(*priv), name, NET_NAME_UNKNOWN, emuc_setup);
    #endif
  #endif /* USES_ALLOC_CANDEV */
  
  if (!devs[0])
    return -1;

  #ifdef USES_ALLOC_CANDEV
    strncpy(devs[0]->name, name, sizeof(devs[0]->name));
  #endif /* USES_ALLOC_CANDEV */

  sprintf(name, "emuccan%d", id[1]);
  
  #ifdef USES_ALLOC_CANDEV
    devs[1] = alloc_candev(sizeof(*priv), 0);
  #else
    #if LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0)
      devs[1] = alloc_netdev(sizeof(*priv), name, emuc_setup);
    #else
      devs[1] = alloc_netdev(sizeof(*priv), name, NET_NAME_UNKNOWN, emuc_setup);
    #endif
  #endif /* USES_ALLOC_CANDEV */
  
  if (!devs[1])
  {
    free_netdev(devs[0]);
    return -1; 
  }

  #ifdef USES_ALLOC_CANDEV
    strncpy(devs[1]->name, name, sizeof(devs[1]->name));
  #endif /* USES_ALLOC_CANDEV */

  devs[0]->base_addr = id[0];
  devs[1]->base_addr = 0x100 | id[1];

  priv = netdev_priv(devs[0]);
  priv->magic = EMUC_MAGIC;
  priv->info = info;
  priv = netdev_priv(devs[1]);
  priv->magic = EMUC_MAGIC;
  priv->info = info;

  #ifdef USES_ALLOC_CANDEV
    emuc_setup(devs[0]);
    emuc_setup(devs[1]);
  #endif /* USES_ALLOC_CANDEV */

  /* Initialize channel control data */
  info->magic = EMUC_MAGIC;
  info->devs[0] = devs[0];
  info->devs[1] = devs[1];
  emuc_devs[id[0]] = devs[0];
  emuc_devs[id[1]] = devs[1];
  spin_lock_init(&info->lock);
  atomic_set(&info->ref_count, 2);
  INIT_WORK(&info->tx_work, emuc_transmit);

  return 0;

} /* END: emuc_alloc() */


/*---------------------------------------------------------------------------------------------------*/
static void emuc_setup (struct net_device *dev)
{
#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  dev->netdev_ops = &emuc_netdev_ops;

  #if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,9)
  dev->priv_destructor = emuc_free_netdev;
  #else
  dev->destructor = emuc_free_netdev;
  #endif

  dev->hard_header_len = 0;
  dev->addr_len        = 0;
  dev->tx_queue_len    = 10;

  dev->mtu  = sizeof(struct can_frame);
  dev->type = ARPHRD_CAN;

  /* New-style flags. */
  dev->flags    = IFF_NOARP;
  dev->features = NETIF_F_HW_CSUM;
}

/*---------------------------------------------------------------------------------------------------*/
static void emuc_free_netdev (struct net_device *dev)
{
  int             i = (dev->base_addr & 0xFF);
  EMUC_RAW_INFO  *info = ((EMUC_PRIV *) netdev_priv(dev))->info;

#if _DBG_FUNC
  print_func_trace(__LINE__, __FUNCTION__);
#endif

  free_netdev(dev);

  emuc_devs[i] = NULL;

  if(atomic_dec_and_test(&info->ref_count))
  {
    printk("free_netdev: free info\n");
    kfree(info);
  }
}

/*---------------------------------------------------------------------------------------------------*/
static void free_can_k_obj(void)
{
  CAN_K_OBJ *tmp;
  can_k_obj_current = can_k_obj_head;

	while(can_k_obj_current != NULL)
  {
		tmp = can_k_obj_current;
		can_k_obj_current = can_k_obj_current->next;
    kobject_put(tmp->kobj);
		kfree(tmp);
	}
 
  kobject_put(emuc_b202_kobj);
} /* END: free_can_k_obj() */

/*---------------------------------------------------------------------------------------------------*/
static ssize_t model_name_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
  CAN_K_OBJ *scan;
  scan = can_k_obj_head;

	while(scan != NULL)
  {
    if(!strcmp(scan->can_name, kobj->name))
      break;
		scan = scan->next;
	}

  return sprintf(buf, "%s", scan->model);
}

#if _DBG_FUNC
/*---------------------------------------------------------------------------------------------------*/
void print_func_trace (int line, const char *func)
{
  printk("----------------> %d, %s()\n", line, func);
}
#endif
