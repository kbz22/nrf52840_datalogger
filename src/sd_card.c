#include <zephyr/kernel.h>
#include <zephyr/fs/fs.h>
#include <zephyr/storage/disk_access.h>
#include <errno.h>
#include <string.h>

#include "sd_card.h"

#define DISK_NAME "SD"
#define MOUNT_POINT "/mnt/sd"

static struct fs_mount_t mp = {
	.type = FS_FATFS,
	.fs_data = NULL,
	.storage_dev = (void *)DISK_NAME,
	.mnt_point = MOUNT_POINT,
};

int sd_card_init(void)
{
	int ret = disk_access_init(DISK_NAME);
	if (ret != 0) {
		printk("disk_access_init failed: %d\n", ret);
		return ret;
	}

	ret = fs_mount(&mp);
	if (ret != 0) {
		printk("fs_mount failed: %d\n", ret);
		return ret;
	}

	return 0;
}

int sd_card_write_test_file(const char *filename, const char *contents)
{
	if (!filename || !contents) {
		return -EINVAL;
	}

	char path[64];
	int len = snprintk(path, sizeof(path), "%s/%s", MOUNT_POINT, filename);
	if (len < 0 || len >= (int)sizeof(path)) {
		return -ENAMETOOLONG;
	}

	struct fs_file_t file;
	fs_file_t_init(&file);

	int ret = fs_open(&file, path, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (ret != 0) {
		printk("fs_open failed: %d\n", ret);
		return ret;
	}

	size_t content_len = strlen(contents);
	size_t written = 0U;

	while (written < content_len) {
		ssize_t rc = fs_write(&file, contents + written, content_len - written);
		if (rc < 0) {
			fs_close(&file);
			printk("fs_write failed: %zd\n", rc);
			return (int)rc;
		}
		written += (size_t)rc;
	}

	ret = fs_sync(&file);
	fs_close(&file);

	return ret;
}
