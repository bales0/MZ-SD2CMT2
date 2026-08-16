#ifndef SD2CMT2_RECORD_PATH_BUFFER_H
#define SD2CMT2_RECORD_PATH_BUFFER_H

#define RECORD_PATH_BUFFER_MAX 160U

/* WAV, LEP/L16 and MZF recording are mutually exclusive and share this path. */
extern char record_path_buffer[RECORD_PATH_BUFFER_MAX];

const char *record_path_filename(void);

#endif
