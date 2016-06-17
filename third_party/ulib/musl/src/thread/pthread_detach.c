#include "pthread_impl.h"
#include <threads.h>

int __pthread_join(pthread_t, void**);

static int __pthread_detach(pthread_t t) {
    /* Cannot detach a thread that's already exiting */
    if (!mxr_mutex_trylock(&t->exitlock))
        return __pthread_join(t, 0);
    t->detached = 2;
    mxr_mutex_unlock(&t->exitlock);
    return 0;
}

weak_alias(__pthread_detach, pthread_detach);
weak_alias(__pthread_detach, thrd_detach);
