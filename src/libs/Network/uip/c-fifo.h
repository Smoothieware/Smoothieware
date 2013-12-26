#ifndef _CFIFO_H_
#define _CFIFO_H_

#ifdef __cplusplus
extern "C" {
#endif

void *new_fifo();
void delete_fifo(void *);
char *fifo_pop(void *);
void fifo_push(void *, char *);
int fifo_size(void *);

#ifdef __cplusplus
}
#endif

#endif
