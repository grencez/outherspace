
#include "slist.h"

#include <assert.h>
#include <string.h>

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
        memcpy (EltZ(dst,i,size), node->car, size);
        i += 1;
        free (node->car);
        tmp = node;
        node = node->cdr;
        free (tmp);
    }
}

    void
app_uint_SList (SList* l, uint x)
{
    uint* p;
    AllocTo( p, 1 );
    *p = x;
    app_SList (l, p);
}

void app_SList (SList* l, void* data)
{
    SListNode* node;
    AllocTo( node, 1 );

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

    void
cat_SList (SList* dst, SList* src)
{
    if (!dst->tail)
    {
        assert (!dst->head);
        dst->nmembs = src->nmembs;
        dst->head = src->head;
        dst->tail = src->tail;
    }
    else if (src->tail)
    {
        assert (src->head);
        dst->nmembs += src->nmembs;
        dst->tail->cdr = src->head;
        dst->tail = src->tail;
    }
    else
    {
        assert (src->nmembs == 0);
        assert (!src->head);
    }

    init_SList (src);
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
    return UINT_MAX;
}

