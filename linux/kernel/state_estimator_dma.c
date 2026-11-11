// SPDX-License-Identifier: GPL-2.0-only
/* State-estimator DMA ownership shim. Userspace protocol logic stays in Rust. */

#include <linux/capability.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/uaccess.h>

#include "include/uapi/linux/state_estimator.h"

#define SE_RING_BYTES             (1024U * 1024U)
#define SE_BLOCK_BYTES            4096U
#define SE_BLOCK_COUNT            256U
#define SE_REG_CONTROL            0x0038
#define SE_REG_STATUS             0x003c
#define SE_REG_RESET_GENERATION   0x0034
#define SE_REG_RESET_KEY          0x00a8
#define SE_REG_RESET_CONTROL      0x00ac
#define SE_RESET_KEY              0x21545352U
#define SE_CONTROL_CAPTURE        BIT(0)
#define SE_CONTROL_DMA            BIT(2)

struct se_device {
	struct device *dev;
	void __iomem *registers;
	struct dma_chan *s2mm;
	struct dma_chan *mm2s;
	void *ring_cpu;
	dma_addr_t ring_dma;
	spinlock_t lock;
	wait_queue_head_t waitq;
	struct miscdevice miscdev;
	u32 producer_block;
	u32 consumer_block;
	u64 host_overruns;
};

static irqreturn_t se_irq(int irq, void *argument)
{
	struct se_device *se = argument;

	/* DMA completion accounting is connected when the cyclic descriptor is armed. */
	wake_up_interruptible(&se->waitq);
	return IRQ_HANDLED;
}

static int se_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct se_device *se = container_of(misc, struct se_device, miscdev);

	file->private_data = se;
	return 0;
}

static __poll_t se_poll(struct file *file, poll_table *wait)
{
	struct se_device *se = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &se->waitq, wait);
	if (READ_ONCE(se->producer_block) != READ_ONCE(se->consumer_block))
		mask |= EPOLLIN | EPOLLRDNORM;
	return mask;
}

static int se_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct se_device *se = file->private_data;
	unsigned long length = vma->vm_end - vma->vm_start;

	if ((vma->vm_flags & VM_WRITE) || length > SE_RING_BYTES)
		return -EPERM;
	return dma_mmap_coherent(se->dev, vma, se->ring_cpu, se->ring_dma,
				 SE_RING_BYTES);
}

static long se_ioctl(struct file *file, unsigned int command,
		     unsigned long argument)
{
	struct se_device *se = file->private_data;
	struct se_ring_info info;
	u32 consumer;
	u32 control;

	switch (command) {
	case SE_IOCTL_GET_RING_INFO:
		info.block_bytes = SE_BLOCK_BYTES;
		info.block_count = SE_BLOCK_COUNT;
		info.producer_block = READ_ONCE(se->producer_block);
		info.consumer_block = READ_ONCE(se->consumer_block);
		info.host_overruns = READ_ONCE(se->host_overruns);
		info.reset_generation = readl(se->registers + SE_REG_RESET_GENERATION);
		info.reserved = 0;
		return copy_to_user((void __user *)argument, &info, sizeof(info)) ? -EFAULT : 0;
	case SE_IOCTL_SET_CONSUMER:
		if (copy_from_user(&consumer, (void __user *)argument, sizeof(consumer)))
			return -EFAULT;
		if (consumer >= SE_BLOCK_COUNT)
			return -EINVAL;
		WRITE_ONCE(se->consumer_block, consumer);
		return 0;
	case SE_IOCTL_START:
	case SE_IOCTL_STOP:
	case SE_IOCTL_FLUSH:
	case SE_IOCTL_SOFT_RESET:
		if (!capable(CAP_SYS_RAWIO))
			return -EPERM;
		break;
	default:
		return -ENOTTY;
	}

	control = readl(se->registers + SE_REG_CONTROL);
	if (command == SE_IOCTL_START)
		writel(control | SE_CONTROL_CAPTURE | SE_CONTROL_DMA,
		       se->registers + SE_REG_CONTROL);
	else if (command == SE_IOCTL_STOP)
		writel(control & ~(SE_CONTROL_CAPTURE | SE_CONTROL_DMA),
		       se->registers + SE_REG_CONTROL);
	else if (command == SE_IOCTL_SOFT_RESET) {
		writel(SE_RESET_KEY, se->registers + SE_REG_RESET_KEY);
		writel(1, se->registers + SE_REG_RESET_CONTROL);
	} else {
		/* FLUSH is implemented by the DMA block packer register in the next integration. */
		return -EOPNOTSUPP;
	}
	return 0;
}

static const struct file_operations se_fops = {
	.owner = THIS_MODULE,
	.open = se_open,
	.poll = se_poll,
	.mmap = se_mmap,
	.unlocked_ioctl = se_ioctl,
	.llseek = no_llseek,
};

static int se_probe(struct platform_device *pdev)
{
	struct se_device *se;
	int irq;
	int error;

	se = devm_kzalloc(&pdev->dev, sizeof(*se), GFP_KERNEL);
	if (!se)
		return -ENOMEM;
	se->dev = &pdev->dev;
	se->registers = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(se->registers))
		return PTR_ERR(se->registers);
	se->s2mm = dma_request_chan(&pdev->dev, "s2mm");
	if (IS_ERR(se->s2mm))
		return dev_err_probe(&pdev->dev, PTR_ERR(se->s2mm), "missing S2MM DMA channel\n");
	se->mm2s = dma_request_chan(&pdev->dev, "mm2s");
	if (IS_ERR(se->mm2s)) {
		error = dev_err_probe(&pdev->dev, PTR_ERR(se->mm2s), "missing MM2S DMA channel\n");
		goto release_s2mm;
	}
	se->ring_cpu = dma_alloc_coherent(&pdev->dev, SE_RING_BYTES,
					  &se->ring_dma, GFP_KERNEL);
	if (!se->ring_cpu) {
		error = -ENOMEM;
		goto release_mm2s;
	}
	spin_lock_init(&se->lock);
	init_waitqueue_head(&se->waitq);
	se->miscdev.minor = MISC_DYNAMIC_MINOR;
	se->miscdev.name = "state-estimator";
	se->miscdev.fops = &se_fops;
	se->miscdev.parent = &pdev->dev;
	irq = platform_get_irq_optional(pdev, 0);
	if (irq > 0) {
		error = devm_request_irq(&pdev->dev, irq, se_irq, 0,
					 dev_name(&pdev->dev), se);
		if (error)
			goto free_ring;
	}
	error = misc_register(&se->miscdev);
	if (error)
		goto free_ring;
	platform_set_drvdata(pdev, se);
	dev_info(&pdev->dev, "allocated %u coherent DMA blocks\n", SE_BLOCK_COUNT);
	return 0;

free_ring:
	dma_free_coherent(&pdev->dev, SE_RING_BYTES, se->ring_cpu, se->ring_dma);
release_mm2s:
	dma_release_channel(se->mm2s);
release_s2mm:
	dma_release_channel(se->s2mm);
	return error;
}

static void se_remove(struct platform_device *pdev)
{
	struct se_device *se = platform_get_drvdata(pdev);

	misc_deregister(&se->miscdev);
	dma_free_coherent(&pdev->dev, SE_RING_BYTES, se->ring_cpu, se->ring_dma);
	dma_release_channel(se->mm2s);
	dma_release_channel(se->s2mm);
}

static const struct of_device_id se_of_match[] = {
	{ .compatible = "cornell-rocketry,state-estimator-1.0" },
	{ }
};
MODULE_DEVICE_TABLE(of, se_of_match);

static struct platform_driver se_driver = {
	.probe = se_probe,
	.remove = se_remove,
	.driver = {
		.name = "state-estimator",
		.of_match_table = se_of_match,
	},
};
module_platform_driver(se_driver);

MODULE_DESCRIPTION("Cornell Rocketry state-estimator DMA shim");
MODULE_AUTHOR("Cornell Rocketry");
MODULE_LICENSE("GPL");
