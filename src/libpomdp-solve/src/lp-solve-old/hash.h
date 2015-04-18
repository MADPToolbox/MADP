#ifndef LP_SOLVE_HASH
#define LP_SOLVE_HASH 1

typedef struct _hashelem
{
  char             *name;
  struct _hashelem *next;
  struct _column   *col;
  struct _bound    *bnd;
  int               must_be_int;
  int               index; /* for row and column name hash tables */
} hashelem;

typedef struct _hashtable
{
  hashelem **table;
  int        size;  
} hashtable;

hashtable *create_hash_table(int size);
void free_hash_table(hashtable *ht);
hashelem *findhash(const char *name, hashtable *ht);
hashelem *puthash(const char *name, hashtable *ht);

#endif
