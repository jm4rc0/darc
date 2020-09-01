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

/* The darc mutex is used where spinning is likely to help on multicore systems
   However by default the darc_mutex is a normal pthread_mutex */
#ifdef USESPINLOCKS
  #define darc_mutex_t pthread_spinlock_t
  #define darc_mutex_init pthread_spin_init
  #define darc_mutex_init_NULL PTHREAD_PROCESS_PRIVATE
  #define darc_mutex_init_attr PTHREAD_PROCESS_PRIVATE
  #define darc_mutex_lock pthread_spin_lock
  #define darc_mutex_timedlock(x,y) pthread_spin_lock(x)
  #define darc_mutex_unlock pthread_spin_unlock
  #define darc_mutex_destroy pthread_spin_destroy
  #define darc_mutex_consistent_np(x)

  #define darc_mutexattr_t int
  #define darc_mutexattr_init(x)
  #define darc_mutexattr_setpshared(x,y)
  #define darc_mutexattr_setrobust_np(x,y)
  #define darc_mutexattr_destroy(x)
#else
  #define darc_mutex_t pthread_mutex_t
  #define darc_mutex_init pthread_mutex_init
  #define darc_mutex_init_NULL NULL
  #define darc_mutex_init_attr &mutexattr
  #define darc_mutex_lock pthread_mutex_lock
  #define darc_mutex_timedlock pthread_mutex_timedlock
  #define darc_mutex_unlock pthread_mutex_unlock
  #define darc_mutex_destroy pthread_mutex_destroy
  #define darc_mutex_consistent_np pthread_mutex_consistent_np

  #define darc_mutexattr_t pthread_mutexattr_t
  #define darc_mutexattr_init pthread_mutexattr_init
  #define darc_mutexattr_setpshared pthread_mutexattr_setpshared
  #define darc_mutexattr_setrobust_np pthread_mutexattr_setrobust_np
  #define darc_mutexattr_destroy pthread_mutexattr_destroy
#endif



/* atomic option - use C11 atomics for the spinlock/mutex condition waits 
   else a spinlock will use simple flags and the mutex will use pthread_cond */
#ifdef USEATOMICOND
  #include <stdatomic.h>

  #define darc_cond_t atomic_int
  #define darc_condattr_t int

  inline int darc_cond_init(darc_cond_t *x, darc_condattr_t *y){
    atomic_init(x,0);
    return 0;
  }

  inline int darc_cond_wait(darc_cond_t *x, darc_mutex_t *y){
    const int old_val = atomic_load(x);
    volatile int new_val;
    darc_mutex_unlock(y);
    do{
      sched_yield();
      new_val = atomic_exchange(x,old_val);
    }while(new_val==old_val);
    if(new_val==((old_val+1)%100)){
      atomic_store(x,new_val);
    }
    darc_mutex_lock(y);
    return 0;
  }

  inline int darc_cond_timedwait(darc_cond_t *x, darc_mutex_t *y,struct timespec *t){
    const int old_val = atomic_load(x);
    struct timespec t2;
    volatile int rt=0;
    volatile int new_val;
    darc_mutex_unlock(y);
    do{
      sched_yield();
      new_val = atomic_exchange(x,old_val);
      clock_gettime(CLOCK_MONOTONIC,&t2);
      rt = (t2.tv_sec>t->tv_sec || (t2.tv_sec>=t->tv_sec && t2.tv_nsec>t->tv_nsec));
    }while(new_val==old_val && rt==0);
    if(new_val==((old_val+1)%100)){
      atomic_store(x,new_val);
    }
    darc_mutex_lock(y);
    return rt;
  }

  inline int darcm_fcond_wait(darc_cond_t *x, pthread_mutex_t *y){
    const int old_val = atomic_load(x);
    volatile int new_val;
    pthread_mutex_unlock(y);
    do{
      new_val = atomic_exchange(x,old_val);
      sched_yield();
    }while(new_val==old_val);
    if(new_val==((old_val+1)%100)){
      atomic_store(x,new_val);
    }
    pthread_mutex_lock(y);
    return 0;
  }

  inline int darc_cond_broadcast(darc_cond_t *x){
    atomic_store(x,(atomic_load(x)+1)%100);
    return 0;
  }

  inline int darc_cond_signal(darc_cond_t *x){
    atomic_store(x,-1*atomic_load(x));
    return 0;
  }

  inline int darc_cond_destroy(darc_cond_t *x){
    atomic_store(x,100);
    return 0;
  }

  #define darc_condattr_init(x)
  #define darc_condattr_setpshared(x,y)
  #define darc_condattr_destroy(x)

#else
  #ifdef USESPINLOCKS
    #define darc_cond_t volatile int
    #define darc_condattr_t int

    inline int darc_cond_init(darc_cond_t *x, darc_condattr_t *y){
      *x=0;
      return 0;
    }

    inline int darc_cond_wait(darc_cond_t *x, darc_mutex_t *y){
      const int local_cond = *(x);
      darc_mutex_unlock(y);
      while (*(x)==(local_cond)){
        sched_yield();
      }
      darc_mutex_lock(y);
      return 0;
    }

    inline int darc_cond_timedwait(darc_cond_t *x,darc_mutex_t *y,struct timespec *t){
      const int local_cond = *(x);
      struct timespec t2;
      volatile int rt=0;
      darc_mutex_unlock(y);
      do {
        sched_yield();
        clock_gettime(CLOCK_MONOTONIC,&t2);
        rt = (t2.tv_sec>t->tv_sec || (t2.tv_sec>=t->tv_sec && t2.tv_nsec>t->tv_nsec));
      }while (*(x)==local_cond && rt==0);
      darc_mutex_lock(y);
      return rt;
    }

    inline int darcm_fcond_wait(darc_cond_t *x, pthread_mutex_t *y){
      const int local_cond = *(x);
      pthread_mutex_unlock(y);
      while (*(x)==(local_cond)){
        sched_yield();
      }
      pthread_mutex_lock(y);
      return 0;
    }

    inline int darc_cond_broadcast(darc_cond_t *x){
      *x=(*x+1)%100;
      return 0;
    }

    inline int darc_cond_signal(darc_cond_t *x){
      *x=(*x+1)%100;
      return 0;
    }

    inline int darc_cond_destroy(darc_cond_t *x){
      *x=100;
      return 0;
    }

    #define darc_condattr_init(x)
    #define darc_condattr_setpshared(x,y)
    #define darc_condattr_destroy(x)

  #else

    #define darc_cond_t pthread_cond_t
    #define darc_cond_init pthread_cond_init
    #define darc_cond_wait pthread_cond_wait
    #define darc_cond_timedwait pthread_cond_timedwait
    #define darc_cond_broadcast pthread_cond_broadcast
    #define darc_cond_destroy pthread_cond_destroy
    #define darc_cond_signal pthread_cond_signal
    #define darc_condattr_t pthread_condattr_t

    #define darc_condattr_init pthread_condattr_init
    #define darc_condattr_setpshared pthread_condattr_setpshared
    #define darc_condattr_destroy pthread_condattr_destroy

  #endif
#endif

#ifdef USESPINBARRIER
  #ifndef USEATOMICOND
    #include <stdatomic.h>
  #endif
  typedef struct SpinBarrier{
    int nthreads;
    atomic_int sense;
    atomic_int threadCount;
  } darc_barrier_t;

  inline int darc_barrier_init(darc_barrier_t *x, const pthread_barrierattr_t *y, unsigned int z){
    (x)->nthreads = (z);
    atomic_init(&((x)->sense),0);
    atomic_init(&((x)->threadCount),0);
    return 0;
  }

  inline int darc_barrier_wait(darc_barrier_t *x){
    int sense = atomic_load(&((x)->sense));
    if(atomic_fetch_add(&((x)->threadCount),1)==((x)->nthreads-1)){
      atomic_store(&((x)->threadCount),0);
      atomic_store(&((x)->sense),1-sense);
    }else{
      while(atomic_load(&((x)->sense))==sense)
        sched_yield();
    }
    return 0;
  }
  inline int darc_barrier_destroy(darc_barrier_t *x){
    atomic_store(&((x)->sense),1-sense);
    return 0;
  }

#else
#define darc_barrier_t pthread_barrier_t
#define darc_barrier_init pthread_barrier_init
#define darc_barrier_wait pthread_barrier_wait
#define darc_barrier_destroy pthread_barrier_destroy
#endif

/* The darc futex is used as a semaphore-like construct
   However is uses the linux futex and can broadcast */
#include <linux/futex.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <limits.h>

typedef volatile int darc_futex_t;

static inline int darc_futex_init(darc_futex_t *futex){
  *(futex) = 0;
  return 0;
}

static inline int darc_futex_init_to_value(darc_futex_t *futex, int val){
  *(futex) = val;
  return 0;
}

static inline int darc_futex_wait(darc_futex_t *futex){
  return syscall(SYS_futex, futex, FUTEX_WAIT, 0, NULL, NULL, 0);
}

static inline int darc_futex_wait_if_value(darc_futex_t *futex, int val){
  return syscall(SYS_futex, futex, FUTEX_WAIT, val, NULL, NULL, 0);
}

static inline int darc_futex_timedwait(darc_futex_t *futex, const struct timespec *to){
  return syscall(SYS_futex, futex, FUTEX_WAIT, 0, to, NULL, 0);
}

static inline int darc_futex_timedwait_if_value(darc_futex_t *futex, int val, const struct timespec *to){
  return syscall(SYS_futex, futex, FUTEX_WAIT, val, to, NULL, 0);
}

static inline int darc_futex_signal(darc_futex_t *futex){
  return syscall(SYS_futex, futex, FUTEX_WAKE, 1, NULL, NULL, 0);
}

static inline int darc_futex_broadcast(darc_futex_t *futex){
  return syscall(SYS_futex, futex, FUTEX_WAKE, INT_MAX, NULL, NULL, 0);
}

static inline int darc_futex_destroy(darc_futex_t *futex){
  *(futex) = 1000;
  return syscall(SYS_futex, futex, FUTEX_WAKE, INT_MAX, NULL, NULL, 0);
}

//not yet implemented these as spinlocks:
#define darc_rwlock_t pthread_rwlock_t
#define darc_rwlock_init pthread_rwlock_init
#define darc_rwlock_rdlock pthread_rwlock_rdlock
#define darc_rwlock_wrlock pthread_rwlock_wrlock
#define darc_rwlock_destroy pthread_rwlock_destroy
#define darc_rwlock_unlock pthread_rwlock_unlock

#endif //header guard
