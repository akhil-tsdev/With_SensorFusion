#include <stdint.h>

typedef struct {uint32_t timestamp; uint32_t theta;} q_entry_t;

void push(q_entry_t);
bool pop(q_entry_t*);
int sz(void);
