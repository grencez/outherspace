
#ifndef SLIST_H_
#define SLIST_H_

#include "util.h"

struct slist_node_struct;
struct slist_struct;

typedef struct slist_node_struct SListNode;
typedef struct slist_struct SList;

struct slist_node_struct
{
    void* car;
    SListNode* cdr;
};

struct slist_struct
{
    uint nmembs;
    SListNode* head;
    SListNode* tail;
};

void init_SList (SList* l);
void cleanup_SList (SList* l);
void unroll_SList (void* dst, SList* src, size_t size);

void app_SList (SList* l, void* data);

void*
aref_SList (SList* l, uint i);
uint
search_SList (SList* l, const void* item,
              bool (*f) (const void*, const void*));

#ifdef INCLUDE_SOURCE
#include "slist.c"
#endif
#endif

