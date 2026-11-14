#ifndef _UAPI_LINUX_STATE_ESTIMATOR_H
#define _UAPI_LINUX_STATE_ESTIMATOR_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define SE_IOCTL_MAGIC 0x53

struct se_ring_info {
	__u32 block_bytes;
	__u32 block_count;
	__u32 producer_block;
	__u32 consumer_block;
	__u64 host_overruns;
	__u32 reset_generation;
	__u32 reserved;
};

#define SE_IOCTL_GET_RING_INFO _IOR(SE_IOCTL_MAGIC, 0x00, struct se_ring_info)
#define SE_IOCTL_SET_CONSUMER  _IOW(SE_IOCTL_MAGIC, 0x01, __u32)
#define SE_IOCTL_START         _IO(SE_IOCTL_MAGIC, 0x02)
#define SE_IOCTL_STOP          _IO(SE_IOCTL_MAGIC, 0x03)
#define SE_IOCTL_FLUSH         _IO(SE_IOCTL_MAGIC, 0x04)
#define SE_IOCTL_SOFT_RESET    _IO(SE_IOCTL_MAGIC, 0x05)

#endif
