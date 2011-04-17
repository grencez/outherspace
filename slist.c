
#include "slist.h"

#include <assert.h>

void init_SList (SList* l)
{
    l->nmembs = 0;
    l->head = 0;
    l->tail = 0;
}

void cleanup_SList (SList* l)
{
    SListNode* node;
    node = l->head;
    l->head = 0;
    l->tail = 0;
    while (node)
    {
        SListNode* tmp;
        assert (node->car);
        free (node->car);
        tmp = node;
        node = node->cdr;
        free (tmp);
    }
}

void unroll_SList (void* dst, SList* src, size_t size)
{
    uint i = 0;
    SListNode* node;
    node = src->head;
    src->head = 0;
    src->tail = 0;
    while (node)
    {
        SListNode* tmp;
        assert (node->car);
        array_set (dst, i++, node->car, size);
        free (node->car);
        tmp = node;
        node = node->cdr;
        free (tmp);
    }
}

void app_SList (SList* l, void* data)
{
    SListNode* node;
    node = AllocT( SListNode, 1 );

    node->car = data;
    node->cdr = 0;

    if (!l->tail)
    {
        assert (!l->head);
        l->head = node;
    }
    else
    {
        l->tail->cdr = node;
    }

    l->tail = node;
    ++ l->nmembs;

    assert (l->head);
    assert (l->tail);
    assert (!l->tail->cdr);
}

    void*
aref_SList (SList* l, uint i)
{
    uint ctr;
    SListNode* node;
    assert (i < l->nmembs);
    node = l->head;

    UFor( ctr, i )
    {
        assert (node);
        node = node->cdr;
    }
    assert (node);
    return node->car;
}

    uint
search_SList (SList* l, const void* item,
              bool (*f) (const void*, const void*))
{
    uint i;
    SListNode* node;
    node = l->head;
    UFor( i, l->nmembs )
    {
        if (f (item, node->car))  return i;
        node = node->cdr;
    }
    return Max_uint;
}

