
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
void acpy_SList (void* dst, const SList* src, size_t size);

void app_SList (SList* l, void* data);

#include "slist.c"
#endif

