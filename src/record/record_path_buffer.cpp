#include "record_path_buffer.h"

#include <stddef.h>
#include <string.h>

char record_path_buffer[RECORD_PATH_BUFFER_MAX];

const char *record_path_filename(void)
{
    const char *last_slash = strrchr(record_path_buffer, '/');
    return (last_slash == NULL) ? record_path_buffer : last_slash + 1U;
}
