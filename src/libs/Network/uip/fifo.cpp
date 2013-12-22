// c accessibllity to the c++ fifo class
#include "fifo.h"
#include "c-fifo.h"

void *new_fifo()
{
    return new Fifo<char*>;
}

void delete_fifo(void *fifo)
{
    if(fifo == NULL) return;
    Fifo<char *> *f= static_cast<Fifo<char *> *>(fifo);
    while(f->size() > 0) {
        char *s= f->pop();
        if (s != NULL) {
            free(s);
        }
    }
    delete f;
}

char *fifo_pop(void *fifo)
{
    Fifo<char *> *f= static_cast<Fifo<char *> *>(fifo);
    return f->pop();
}

void fifo_push(void *fifo, char *str)
{
    Fifo<char *> *f= static_cast<Fifo<char *> *>(fifo);
    f->push(str);
}

int fifo_size(void *fifo)
{
    Fifo<char *> *f= static_cast<Fifo<char *> *>(fifo);
    return f->size();
}
