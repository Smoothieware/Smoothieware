/*************************************************************************
 *
 * $Author: Jim Morris $
 * $Date: 1999/02/05 21:05:00 $
 *
 * this code is Licensed LGPL
 *
 *************************************************************************/
#ifndef _FIFO_H_
#define _FIFO_H_

#include <stdlib.h>

// Doubly Linked list class

template<class T> class LList;

template<class T>
class Tlink
{
public:
    Tlink<T> *pnext;
    Tlink<T> *pprev;

public:
    Tlink()
    {
        pnext = pprev = 0;
    }
    Tlink(Tlink *p, Tlink *n)
    {
        pprev = p;
        pnext = n;
    }
    Tlink(const T &d) : data(d)
    {
        pnext = pprev = 0;
    }
    T data;
};

template<class T>
class list_base
{
private:
    Tlink<T> *head;
    Tlink<T> *tail;
    int cnt;

protected:
    list_base()
    {
        head = tail = NULL;
        cnt = 0;
    }

    list_base(Tlink<T> *n) // link into head of list
    {
        cnt = 1;
        n->pnext = NULL;
        n->pprev = NULL;
        head = n;
        tail = n;
    }

    Tlink<T> *gethead(void) const
    {
        return head;
    }
    Tlink<T> *gettail(void) const
    {
        return tail;
    }
    Tlink<T> *getnext(Tlink<T> *n) const
    {
        return n->pnext;
    }
    Tlink<T> *getprev(Tlink<T> *n) const
    {
        return n->pprev;
    }

    void addtohead(Tlink<T> *n) // add at head of list
    {
        n->pnext = head;
        n->pprev = NULL;
        if (head) head->pprev = n;
        head = n;
        if (tail == NULL) // first one
            tail = n;
        cnt++;
    }

    void addtohead(int c, Tlink<T> *a, Tlink<T> *b) // add list at head of list
    {
        b->pnext = head;
        a->pprev = NULL;
        if (head) head->pprev = b;
        head = a;
        if (tail == NULL) // first one
            tail = b;
        cnt += c;
    }

    void addtotail(Tlink<T> *n)  // add to tail of list
    {
        n->pnext = NULL;
        n->pprev = tail;
        if (tail) tail->pnext = n;
        tail = n;
        if (head == NULL) // first one
            head = n;
        cnt++;
    }

    void remove(Tlink<T> *n) // remove it by relinking
    {
        cnt--;
        if (n->pprev) n->pprev->pnext = n->pnext;
        else head = n->pnext; // it must be the head
        if (n->pnext) n->pnext->pprev = n->pprev;
        else tail = n->pprev;
    }

    void reset()
    {
        head = tail = NULL;
        cnt = 0;
    }
    int count() const
    {
        return cnt;
    }
};

// fifo
template<class T>
class Fifo : private list_base<T>
{
public:
    Fifo(){}

    void push(const T &a);
    T pop();
    T peek();
    int size() const;
};

template <class T>
int Fifo<T>::size() const
{
    return list_base<T>::count();
}

// add to end of list (FIFO)
template <class T>
void Fifo<T>::push(const T &a)
{
    list_base<T>::addtotail(new Tlink<T>(a));
}

// return the first item in the list
template <class T>
T Fifo<T>::peek()
{
    Tlink<T> *p = list_base<T>::gethead();
    return p->data;
}

// pop the first item off the fifo
template <class T>
T Fifo<T>::pop()
{
    Tlink<T> *p = list_base<T>::gethead();
    T data = p->data;
    list_base<T>::remove(p);
    delete p;
    return data;
};
#endif
