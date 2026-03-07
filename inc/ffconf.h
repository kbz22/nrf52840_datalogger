/*
 * Custom FatFS configuration for NCS 3.1.1
 * Defines FF_VOLUMES to prevent compilation error
 */

#ifndef FFCONF_H
#define FFCONF_H

#define FF_VOLUMES 1

#define FF_USE_LFN        1
#define FF_MAX_LFN        255
#define FF_LFN_UNICODE    0

#endif /* FFCONF_H */
