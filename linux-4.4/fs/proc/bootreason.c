#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/idr.h>
#include <linux/device.h>
#include <linux/irq.h>

static unsigned int resetInd = 0;

static int bootreason_proc_show(struct seq_file *m, void *v)
{
	switch (resetInd) {
		case 0x5A5A5A5F:
			seq_printf(m, "%s\n", "panic");
			break;
		case 0x5A5A5A50:
			seq_printf(m, "%s\n", "soft");
			break;
		case 0xffffffff:
			seq_printf(m, "%s\n", "hard");
			break;
		default:
			seq_printf(m, "%s (0x%x)\n", "unknown", resetInd);
			break;
	}

	return 0;
}

static int bootreason_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, bootreason_proc_show, NULL);
}

static const struct file_operations bootreason_proc_fops = {
	.open		= bootreason_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init proc_bootreason_init(void)
{
	proc_create("bootreason", 0, NULL, &bootreason_proc_fops);

	if (request_mem_region(0x87b7fffc, 8, "BootReason")) {
		void *ptr;
		if (ptr=ioremap(0x87b7fffc, 8)) {
			unsigned int setflag = 0x5a5a5a50;
			memcpy(&resetInd,ptr,4);//Reset indicator
			memcpy(ptr,&setflag,4);//set default flag as soft reset
			iounmap(ptr);
		}
		release_mem_region(0x87b7fffc, 8);
	}

	return 0;
}
fs_initcall(proc_bootreason_init);
