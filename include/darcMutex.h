/*
darc, the Durham Adaptive optics Real-time Controller.
Copyright (C) 2010 Alastair Basden.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Affero General Public License as
published by the Free Software Foundation, either version 3 of the
License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Affero General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef DARCMUTEX_H //header guard
#define DARCMUTEX_H

/* atomic option */
#ifdef USEATOMICS

#include <stdatomic.h>
#define darc_set(x,y) atomic_store((x),(y))
#define darc_check(x,y) (atomic_load(x)==(y))
#define darc_wait_val atomic_int

    #ifdef USEMYBARRIERS
    struct MyBarrier{
    int nthreads;
    volatile int sense;
    atomic_int threadCount;
    };
    #define darc_barrier_t struct MyBarrier
    #define darc_barrier_init(x,y,z) do {\
                    (x)->nthreads = (z);\
                    (x)->sense = 0;\
                    atomic_init(&((x)->threadCount),0);\
                    } while (0)
    #define darc_barrier_wait(x) do {\
                    int sense = (x)->sense;\
                    if(atomic_fetch_add_explicit(&((x)->threadCount),1,memory_order_relaxed)==((x)->nthreads-1)){\
                        sched_yield();\
                        atomic_store(&((x)->threadCount),0);\
                        (x)->sense = 1-sense;\
                    }else{\
                        while((x)->sense==sense)\
                            sched_yield();\
                    }\
                    } while (0)
    #else
    #define darc_barrier_t pthread_barrier_t
    #define darc_barrier_init(x,y,z) pthread_barrier_init(x,y,z)
    #define darc_barrier_wait(x) pthread_barrier_wait(x)
    #endif

#else

#define darc_set(x,y) (*(x)=(y))
#define darc_check(x,y) (*(x)==(y))
#define darc_wait_val volatile int

#define darc_barrier_t pthread_barrier_t
#define darc_barrier_init(x,y,z) pthread_barrier_init(x,y,z)
#define darc_barrier_wait(x) pthread_barrier_wait(x)

#endif

#define darc_wait(x,y) do {} while (darc_check(x,y))

/* spinlock option */
#ifdef USESPINLOCKS
#define darc_mutex_t pthread_spinlock_t
#define darc_mutex_init pthread_spin_init
#define darc_mutex_init_var PTHREAD_PROCESS_SHARED
#define darc_mutex_lock pthread_spin_lock
#define darc_mutex_unlock pthread_spin_unlock
#define darc_mutex_destroy pthread_spin_destroy
#else
#define darc_mutex_t pthread_mutex_t
#define darc_mutex_init pthread_mutex_init
#define darc_mutex_init_var NULL
#define darc_mutex_lock pthread_mutex_lock
#define darc_mutex_unlock pthread_mutex_unlock
#define darc_mutex_destroy pthread_mutex_destroy
#endif

#define darc_fast_cond_t volatile int
#define darc_fast_cond_init(x,y) (*(x)=0)
#define darc_fast_cond_wait(x,y) do {\
    const int local_cond = *(x);\
    darc_mutex_unlock(y);\
    darc_wait(x,local_cond);\
    } while (0)
#define darc_fast_cond_broadcast(x) (*(x)=1-*(x))
#define darc_fast_cond_destroy(x) (*x=10)

/* condition option */
#if defined(USEPTHREADCOND) && !defined(USESPINLOCKS)
#define darc_cond_t pthread_cond_t
#define darc_cond_init(x,y) pthread_cond_init(x,y)
#define darc_cond_wait(x,y) pthread_cond_wait(x,y)
#define darc_cond_timedwait(x,y,t) pthread_cond_timedwait(x,y,t)
#define darc_cond_broadcast(x) pthread_cond_broadcast(x)
#define darc_cond_destroy(x) pthread_cond_destroy(x)
#define darc_cond_signal(x) phread_cond_signal(x)
#else
#define darc_cond_t volatile int
#define darc_cond_init(x,y) (*(x)=0)
#define darc_cond_wait(x,y) do {\
                const int local_cond = *(x);\
                darc_mutex_unlock(y);\
                darc_wait(x,local_cond);\
                darc_mutex_lock(y);\
                } while (0)
inline int darc_cond_timedwait(darc_cond_t *x,darc_mutex_t *y,struct timespec *t){
  const int local_cond = *(x);
  struct timespec t2;
  int rt=0;
  darc_mutex_unlock(y);
  do {clock_gettime(CLOCK_REALTIME,&t2);} while (darc_check(x,local_cond) && (t2.tv_sec<t->tv_sec || (t2.tv_sec==t->tv_sec && t2.tv_nsec<t->tv_nsec)));
  if(t2.tv_sec>t->tv_sec || (t2.tv_sec==t->tv_sec && t2.tv_nsec>=t->tv_nsec)){
    rt=1;
  }
  darc_mutex_lock(y);
  return rt;
}

#define darc_cond_broadcast(x) do {		\
                int local_cond = *(x);\
                local_cond++;\
                darc_set(x,local_cond%100);\
                } while (0)
#define darc_cond_destroy(x) (*x=100)

#endif

//not yet implemented these as spinlocks:
#define darc_rwlock_t pthread_rwlock_t
#define darc_rwlock_init pthread_rwlock_init
#define darc_rwlock_rdlock pthread_rwlock_rdlock
#define darc_rwlock_wrlock pthread_rwlock_wrlock
#define darc_rwlock_destroy pthread_rwlock_destroy
#define darc_rwlock_unlock pthread_rwlock_unlock





#endif //header guard
