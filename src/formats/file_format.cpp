#include <ctype.h>
#include <stddef.h>
#include <string.h>
#include "file_format.h"

static bool chars_equal_ignore_case(char a, char b)
{
    return (char)toupper((unsigned char)a) == (char)toupper((unsigned char)b);
}

static bool extension_equals(const char *filename, const char *extension)
{
    if ((filename == NULL) || (extension == NULL))
    {
        return false;
    }
    size_t filename_len = strlen(filename);
    size_t extension_len = strlen(extension);
    if (filename_len < extension_len)
    {
        return false;
    }
    const char *filename_extension = filename + filename_len - extension_len;
    for (size_t i = 0; i < extension_len; i++)
    {
        if (!chars_equal_ignore_case(filename_extension[i], extension[i]))
        {
            return false;
        }
    }
    return true;
}

bool file_format_is_sharp_tape(file_format_t format)
{
    return (format == FILE_FORMAT_MZF) ||
           (format == FILE_FORMAT_MZT) ||
           (format == FILE_FORMAT_M12);
}

file_format_t file_format_detect_from_name(const char *filename)
{
    if (extension_equals(filename, ".MZF")) return FILE_FORMAT_MZF;
    if (extension_equals(filename, ".MZT")) return FILE_FORMAT_MZT;
    if (extension_equals(filename, ".M12")) return FILE_FORMAT_M12;
    if (extension_equals(filename, ".LEP")) return FILE_FORMAT_LEP;
    if (extension_equals(filename, ".L16")) return FILE_FORMAT_L16;
    if (extension_equals(filename, ".WAV")) return FILE_FORMAT_WAV;
    return FILE_FORMAT_UNKNOWN;
}

const char* file_format_to_label(file_format_t format)
{
    switch (format)
    {
        case FILE_FORMAT_MZF: return "MZF";
        case FILE_FORMAT_MZT: return "MZT";
        case FILE_FORMAT_M12: return "M12";
        case FILE_FORMAT_LEP: return "LEP";
        case FILE_FORMAT_L16: return "L16";
        case FILE_FORMAT_WAV: return "WAV";
        case FILE_FORMAT_UNKNOWN:
        default: return "UNKNOWN";
    }
}
