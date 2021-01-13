#ifndef __PCM_FIFO_H
#define __PCM_FIFO_H
//fifo
typedef struct {
	char *fifo_buf;// = NULL;
	volatile unsigned int read_ptr; // = 0;
	volatile unsigned int write_ptr; // = 0;
	unsigned int fifo_max_len; // = 0;
	unsigned int need_len;
} FIFI_WITH_SEM;

void Fifo_Init(FIFI_WITH_SEM *fifo, int max_len);
void Fifo_Free(FIFI_WITH_SEM *fifo);
int  Fifo_Write(FIFI_WITH_SEM *fifo, char* data, int len);
int  Fifo_Read(FIFI_WITH_SEM *fifo, char* data, int len, int shift_len);
void Fifo_Reset(FIFI_WITH_SEM *fifo);
int Fifo_Data_Len(FIFI_WITH_SEM *fifo);

#endif
