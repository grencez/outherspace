
#ifndef SLIST_H_
#define SLIST_H_

#include "util.h"

typedef struct SListNode SListNode;
typedef struct SList SList;

struct SListNode
{
    void* car;
    SListNode* cdr;
};

struct SList
{
    uint nmembs;
    SListNode* head;
    SListNode* tail;
};

void init_SList (SList* l);
void cleanup_SList (SList* l);
void unroll_SList (void* dst, SList* src, size_t size);

void app_uint_SList (SList* l, uint x);
void app_SList (SList* l, void* data);
void cat_SList (SList* dst, SList* src);

void*
aref_SList (SList* l, uint i);
uint
search_SList (SList* l, const void* item,
              bool (*f) (const void*, const void*));

#ifdef INCLUDE_SOURCE
#include "slist.c"
#endif
#endif

