/* Force-included (-include) into every engine file, before its own includes.
 * Redirects the few string/posix functions whose ML-core versions would mix allocators
 * (strdup) or that the core doesn't export (strncasecmp, mkdir, ...). */
#ifndef DM_FORCE_H
#define DM_FORCE_H

#include <stddef.h>
#include <string.h>
#include <strings.h>

#define strdup       dm_strdup
#define strncasecmp  dm_strncasecmp
#define strcasecmp   dm_strcasecmp
#define strstr       dm_strstr
#define strrchr      dm_strrchr
#define strchr       dm_strchr
#define mkdir        dm_mkdir
#define access       dm_access

char *dm_strdup(const char *s);
int   dm_strncasecmp(const char *a, const char *b, size_t n);
int   dm_strcasecmp(const char *a, const char *b);
char *dm_strstr(const char *h, const char *n);
char *dm_strrchr(const char *s, int c);
char *dm_strchr(const char *s, int c);
/* dm_mkdir: prototype comes from <sys/stat.h> through the macro */
int   dm_access(const char *path, int mode);

#endif
