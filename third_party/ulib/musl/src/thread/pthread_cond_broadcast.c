#include "pthread_impl.h"

int __private_cond_signal(pthread_cond_t*, int);

int pthread_cond_broadcast(pthread_cond_t* c) {
    if (!c->_c_shared)
        return __private_cond_signal(c, -1);
    if (!c->_c_waiters)
        return 0;
    a_inc(&c->_c_seq);
    __wake(&c->_c_seq, -1);
    return 0;
}
