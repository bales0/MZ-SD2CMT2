#pragma once

typedef enum
{
    FILE_FORMAT_UNKNOWN = 0,
    FILE_FORMAT_MZF,
    FILE_FORMAT_MZT,
    FILE_FORMAT_LEP,
    FILE_FORMAT_L16,
    FILE_FORMAT_WAV
} file_format_t;

file_format_t file_format_detect_from_name(const char *filename);
const char* file_format_to_label(file_format_t format);
