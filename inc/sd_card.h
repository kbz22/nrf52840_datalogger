#include <stddef.h>

#ifndef SD_CARD_H
#define SD_CARD_H

#ifdef __cplusplus
extern "C" {
#endif

int sd_card_init(void);
int sd_card_write_test_file(const char *filename, const char *contents);

#ifdef __cplusplus
}
#endif

#endif /* SD_CARD_H */
