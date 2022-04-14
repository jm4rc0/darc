
#ifndef _MUTEX_GUARD_
#define _MUTEX_GUARD_


/* The custom mutexes are mainly used in the darc_condwait* calls
   they can also be used to replace darc_mutex* */
#define OPTMUTEX
/* OPTMUTEX is for optimised mutexes/conds which are more complex than the standard
   ones but could provide better performance
   the default is a less complicated futex based mutex/cond */

/* the  darc_condwait* calls are a replacement for the darc_futex* calls */
// #define POSIXCONDWAIT
/* POSIXCONDWAIT uses pthread mutex/cond for the condwait functions */
// #define MUTEXCONDWAIT
/* MUTEXCONDWAIT uses the futex based mutex/conds defined above and in mutex.c
   the default for condwait uses a simple futex setup */


#include <sys/syscall.h>
#include <unistd.h>
#include <linux/futex.h>
#include <pthread.h>
#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <time.h>
#include <semaphore.h>
#include <string.h>

#include <stdio.h>
#include <stdlib.h>

#define atomic_xadd(P, V) __sync_fetch_and_add((P), (V))
#define cmpxchg(P, O, N) __sync_val_compare_and_swap((P), (O), (N))
#define atomic_inc(P) __sync_add_and_fetch((P), 1)
#define atomic_dec(P) __sync_add_and_fetch((P), -1)
#define atomic_add(P, V) __sync_add_and_fetch((P), (V))
#define atomic_set_bit(P, V) __sync_or_and_fetch((P), 1<<(V))
#define atomic_clear_bit(P, V) __sync_and_and_fetch((P), ~(1<<(V)))

#define cpu_relax() asm volatile("pause\n": : :"memory")
#define barrier() asm volatile("": : :"memory")

/* Force a read of the variable */
#define atomic_read(V) (*(volatile typeof(V) *)&(V))

/* Atomic 64 bit exchange */
static inline unsigned long long xchg_64(void *ptr, unsigned long long x)
{
	__asm__ __volatile__("xchgq %0,%1"
				:"=r" ((unsigned long long) x)
				:"m" (*(volatile unsigned long long *)ptr), "0" (x)
				:"memory");

	return x;
}

static inline unsigned xchg_32(void *ptr, unsigned x)
{
	__asm__ __volatile__("xchgl %0,%1"
				:"=r" ((unsigned) x)
				:"m" (*(volatile unsigned *)ptr), "0" (x)
				:"memory");

	return x;
}

static inline unsigned char xchg_8(void *ptr, unsigned char x)
{
    __asm__ __volatile__("xchgb %0,%1"
                :"=r" ((unsigned char) x)
                :"m" (*(volatile unsigned char *)ptr), "0" (x)
                :"memory");

    return x;
}

static inline int sys_futex(void *addr1, int op, int val1, const struct timespec *timeout, void *addr2, int val3)
{
	return syscall(SYS_futex, addr1, op, val1, timeout, addr2, val3);
}

typedef struct mutex_s mutex_t;
typedef union mutex_u mutex_u;

union mutex_u
{
	unsigned u;
	struct
	{
		unsigned char locked;
		unsigned char contended;
	} b;
};

struct mutex_s
{
	union mutex_u m;
	unsigned flags;
	int id;
	unsigned pad;
};

typedef struct cv cv_t;
struct cv
{
	int mid;
	unsigned seq;
	unsigned bcast;
	unsigned flags;
};

typedef struct spincond_t spincond_t;
struct spincond_t
{
	unsigned sig;
	unsigned bcast;
};

/*  The initialisation functions are defined and compiled in mutex.c
	The time critical functions are defined static inline here */

int mutex_init(mutex_t *m, const pthread_mutexattr_t *a);

int mutex_destroy(mutex_t *m);

int cond_init(cv_t *c, pthread_condattr_t *a);

int cond_destroy(cv_t *c);

#ifdef OPTMUTEX

static inline int mutex_lock(mutex_t *m)
{
	int i;

	/* Try to grab lock */
	for (i = 0; i < 100; i++)
	{
		if (!xchg_8(&m->m.b.locked, 1)) return 0;

		cpu_relax();
	}

	/* Have to sleep */
	while (xchg_32(&m->m.u, 257) & 1)
	{
		sys_futex(&m->m, FUTEX_WAIT | m->flags, 257, NULL, NULL, 0);
	}

	return 0;
}

static inline int mutex_timedlock(mutex_t *m, const struct timespec *to)
{
	int i;

	/* Try to grab lock */
	for (i = 0; i < 100; i++)
	{
		if (!xchg_8(&m->m.b.locked, 1)) return 0;

		cpu_relax();
	}
	int retval = 0;
	/* Have to sleep */
	while ((xchg_32(&m->m.u, 257) & 1) && retval==0)
	{
		retval = sys_futex(&m->m, FUTEX_WAIT | m->flags, 257, to, NULL, 0);
	}
	if (retval == -1 && errno == ETIMEDOUT)
		retval = ETIMEDOUT;

	return retval;
}

static inline int mutex_unlock(mutex_t *m)
{
	int i;

	/* Locked and not contended */
	if ((m->m.u == 1) && (cmpxchg(&m->m.u, 1, 0) == 1)) return 0;

	/* Unlock */
	m->m.b.locked = 0;

	barrier();

	/* Spin and hope someone takes the lock */
	for (i = 0; i < 200; i++)
	{
		if (m->m.b.locked) return 0;

		cpu_relax();
	}

	/* We need to wake someone up */
	m->m.b.contended = 0;

	sys_futex(&m->m, FUTEX_WAKE | m->flags, 1, NULL, NULL, 0);

	return 0;
}

static inline int mutex_trylock(mutex_t *m)
{
	/* Try to take the lock, if is currently unlocked */
	unsigned c = xchg_8(&m->m.b.locked, 1);
	if (!c) return 0;
	return EBUSY;
}

static inline int cond_signal(cv_t *c)
{
	/* We are waking someone up */
	atomic_add(&c->seq, 1);

	/* Wake up a thread */
	sys_futex(&c->seq, FUTEX_WAKE | c->flags, 1, NULL, NULL, 0);

	return 0;
}

static inline int cond_broadcast(cv_t *c)
{

	/* We are waking everyone up */
	atomic_add(&c->seq, 1);

	/* Signal a broadcast and wake one thread */
	xchg_32(&c->bcast,1);
	sys_futex(&c->seq, FUTEX_WAKE | c->flags, 1, NULL, NULL, 0);

	return 0;
}

static inline int cond_wait(cv_t *c, mutex_t *m)
{
	int seq = c->seq;

	if (c->mid != m->id)
	{
		if (c->flags!=m->flags) return EINVAL;
		if (c->mid!=-1) return EINVAL;
		/* Atomically set mutex inside cv */
		cmpxchg(&c->mid, -1, m->id);
		if (c->mid != m->id) return EINVAL;
	}

	mutex_unlock(m);

	sys_futex(&c->seq, FUTEX_WAIT | c->flags, seq, NULL, NULL, 0);

	while (xchg_32(&m->m.b.locked, 257) & 1)
	{
		sys_futex(&m->m, FUTEX_WAIT | m->flags, 257, NULL, NULL, 0);
	}

	if (xchg_32(&c->bcast,0) == 1) {
		int seq = c->seq;
		sys_futex(&c->seq, FUTEX_CMP_REQUEUE | c->flags, 0, (void *) INT_MAX, &m->m, seq);
		c->mid = -1;
	}

	return 0;
}

static inline int cond_timedwait(cv_t *c, mutex_t *m, const struct timespec *to)
{
	int seq = c->seq;

	if (c->mid != m->id)
	{
		if (c->flags!=m->flags) return EINVAL;
		if (c->mid!=-1) return EINVAL;
		/* Atomically set mutex inside cv */
		cmpxchg(&c->mid, -1, m->id);
		if (c->mid != m->id) return EINVAL;
	}

	mutex_unlock(m);

	int retval = sys_futex(&c->seq, FUTEX_WAIT | c->flags, seq, to, NULL, 0);
	if (retval == -1 && errno == ETIMEDOUT)
		retval = ETIMEDOUT;

	while (xchg_32(&m->m.b.locked, 257) & 1)
	{
		sys_futex(&m->m, FUTEX_WAIT | m->flags, 257, NULL, NULL, 0);
	}

	if (xchg_32(&c->bcast,0) == 1) {
		int seq = c->seq;
		sys_futex(&c->seq, FUTEX_CMP_REQUEUE | c->flags, 0, (void *) INT_MAX, &m->m, seq);
		c->mid = -1;
	}

	return retval;
}

#else

static inline int mutex_lock(mutex_t *m)
{
	int i, c;

	/* Spin and try to take lock */
	for (i = 0; i < 100; i++)
	{
		c = cmpxchg(&m->m.u, 0, 1);
		if (!c) return 0;

		cpu_relax();
	}

	/* The lock is now contended */
	if (c == 1) c = xchg_32(&m->m.u, 2);

	while (c)
	{
		/* Wait in the kernel */
		sys_futex(&m->m, FUTEX_WAIT | m->flags, 2, NULL, NULL, 0);
		c = xchg_32(&m->m.u, 2);
	}

	return 0;
}

static inline int mutex_timedlock(mutex_t *m, const struct timespec *to)
{
	int i, c;

	/* Spin and try to take lock */
	for (i = 0; i < 100; i++)
	{
		c = cmpxchg(&m->m.u, 0, 1);
		if (!c) return 0;

		cpu_relax();
	}

	/* The lock is now contended */
	if (c == 1) c = xchg_32(&m->m.u, 2);

	int retval = 0;

	while (c && retval==0)
	{
		/* Wait in the kernel */
		retval = sys_futex(&m->m, FUTEX_WAIT | m->flags, 2, to, NULL, 0);
		c = xchg_32(&m->m.u, 2);
	}
	if (retval == -1 && errno == ETIMEDOUT)
		retval = ETIMEDOUT;

	return retval;
}

static inline int mutex_unlock(mutex_t *m)
{
	int i;

	/* Unlock, and if not contended then exit. */
	if (m->m.u == 2)
	{
		m->m.u = 0;
	}
	else if (xchg_32(&m->m.u, 0) == 1) return 0;

	/* Spin and hope someone takes the lock */
	for (i = 0; i < 200; i++)
	{
		if (&m->m.u)
		{
			/* Need to set to state 2 because there may be waiters */
			if (cmpxchg(&m->m.u, 1, 2)) return 0;
		}
		cpu_relax();
	}

	/* We need to wake someone up */
	sys_futex(&m->m, FUTEX_WAKE | m->flags, 1, NULL, NULL, 0);

	return 0;
}

static inline int mutex_trylock(mutex_t *m)
{
	/* Try to take the lock, if is currently unlocked */
	unsigned c = cmpxchg(&m->m.u, 0, 1);
	if (!c) return 0;
	return EBUSY;
}

static inline int cond_signal(cv_t *c)
{
	/* We are waking someone up */
	atomic_add(&c->seq, 1);

	/* Wake up a thread */
	sys_futex(&c->seq, FUTEX_WAKE | c->flags, 1, NULL, NULL, 0);

	return 0;
}

static inline int cond_broadcast(cv_t *c)
{

	/* We are waking everyone up */
	atomic_add(&c->seq, 1);

	/* Signal a broadcast and wake one thread */
	xchg_32(&c->bcast,1);
	sys_futex(&c->seq, FUTEX_WAKE | c->flags, 1, NULL, NULL, 0);

	return 0;
}

static inline int cond_wait(cv_t *c, mutex_t *m)
{
	int seq = c->seq;

	if (c->mid != m->id)
	{
		if (c->flags!=m->flags) return EINVAL;
		if (c->mid!=-1) return EINVAL;
		/* Atomically set mutex inside cv */
		cmpxchg(&c->mid, -1, m->id);
		if (c->mid != m->id) return EINVAL;
	}

	mutex_unlock(m);

	sys_futex(&c->seq, FUTEX_WAIT | c->flags, seq, NULL, NULL, 0);

	while (xchg_32(&m->m.u, 2))
	{
		sys_futex(&m->m, FUTEX_WAIT | m->flags, 2, NULL, NULL, 0);
	}

	if (xchg_32(&c->bcast,0) == 1) {
		sys_futex(&c->seq, FUTEX_REQUEUE | c->flags, 0, (void *) INT_MAX, &m->m, 0);
		c->mid = -1;
	}

	return 0;
}

static inline int cond_timedwait(cv_t *c, mutex_t *m, const struct timespec *to)
{
	int seq = c->seq;

	if (c->mid != m->id)
	{
		if (c->flags!=m->flags) return EINVAL;
		if (c->mid!=-1) return EINVAL;
		/* Atomically set mutex inside cv */
		cmpxchg(&c->mid, -1, m->id);
		if (c->mid != m->id) return EINVAL;
	}

	mutex_unlock(m);

	int retval = sys_futex(&c->seq, FUTEX_WAIT | c->flags, seq, to, NULL, 0);
	if (retval == -1 && errno == ETIMEDOUT)
		retval = ETIMEDOUT;

	while (xchg_32(&m->m.u, 2))
	{
		sys_futex(&m->m, FUTEX_WAIT | m->flags, 2, NULL, NULL, 0);
	}

	if (xchg_32(&c->bcast, 0) == 1) {
		sys_futex(&c->seq, FUTEX_REQUEUE | c->flags, 0, (void *) INT_MAX, &m->m, 0);
		c->mid = -1;
	}

	return retval;
}

#endif

#if defined(POSIXCONDWAIT)
typedef struct darc_condwait_t darc_condwait_t;
struct darc_condwait_t
{
	pthread_mutex_t m;
	pthread_cond_t c;
	unsigned value;
};
#elif defined(MUTEXCONDWAIT)
typedef struct darc_condwait_t darc_condwait_t;
struct darc_condwait_t
{
	mutex_t m;
	cv_t c;
	unsigned value;
};
#else
typedef struct darc_condwait_t darc_condwait_t;
struct darc_condwait_t
{
	unsigned value;
	unsigned flags;
};
#endif

int darc_condwait_init(darc_condwait_t *s);

int darc_condwait_init_tovalue(darc_condwait_t *s, unsigned value);

int darc_condwait_init_shared(darc_condwait_t *s);

int darc_condwait_init_shared_tovalue(darc_condwait_t *s, unsigned value);

int darc_condwait_destroy(darc_condwait_t *s);

#ifdef POSIXCONDWAIT
static inline int darc_condwait_wait(darc_condwait_t *s)
{
	int retval = 0;
	pthread_mutex_lock(&s->m);
	retval = pthread_cond_wait(&s->c, &s->m);
	pthread_mutex_unlock(&s->m);
	return retval;
}

static inline int darc_condwait_wait_ifvalue(darc_condwait_t *s, unsigned value)
{
	int retval = 0;
	pthread_mutex_lock(&s->m);
	if (s->value==value)
		retval = pthread_cond_wait(&s->c, &s->m);
	pthread_mutex_unlock(&s->m);
	return retval;
}

static inline int darc_condwait_timedwait(darc_condwait_t *s, const struct timespec *to)
{
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &now);
	now.tv_sec+=to->tv_sec;
	now.tv_nsec+=to->tv_nsec;
	if (now.tv_nsec > 999999999) {
		now.tv_nsec -= 1000000000;
		now.tv_sec += 1;
	}
	int retval = 0;
	pthread_mutex_lock(&s->m);
	retval = pthread_cond_timedwait(&s->c, &s->m, &now);
	pthread_mutex_unlock(&s->m);
	return retval;
}

static inline int darc_condwait_timedwait_ifvalue(darc_condwait_t *s, const struct timespec *to, unsigned value)
{
	struct timespec now;
	clock_gettime(CLOCK_REALTIME, &now);
	now.tv_sec+=to->tv_sec;
	now.tv_nsec+=to->tv_nsec;
	if (now.tv_nsec > 999999999) {
		now.tv_nsec -= 1000000000;
		now.tv_sec += 1;
	}
	int retval = 0;
	pthread_mutex_lock(&s->m);
	if (s->value==value)
		retval = pthread_cond_timedwait(&s->c, &s->m, &now);
	pthread_mutex_unlock(&s->m);
	return retval;
}

static inline int darc_condwait_signal(darc_condwait_t *s)
{
	return pthread_cond_signal(&s->c);
}

static inline int darc_condwait_signal_withvalue(darc_condwait_t *s, unsigned value)
{
	int retval;
	pthread_mutex_lock(&s->m);
	s->value = value;
	retval = pthread_cond_signal(&s->c);
	pthread_mutex_unlock(&s->m);
	return retval;
}

static inline int darc_condwait_broadcast(darc_condwait_t *s)
{
	return pthread_cond_broadcast(&s->c);
}

static inline int darc_condwait_broadcast_withvalue(darc_condwait_t *s, unsigned value)
{
	int retval;
	pthread_mutex_lock(&s->m);
	s->value = value;
	retval = pthread_cond_broadcast(&s->c);
	pthread_mutex_unlock(&s->m);
	return retval;
}

static inline int darc_condwait_setvalue(darc_condwait_t *s, unsigned value)
{
	pthread_mutex_lock(&s->m);
	s->value = value;
	pthread_mutex_unlock(&s->m);
	return 0;
}

#elif defined(MUTEXCONDWAIT)

static inline int darc_condwait_wait(darc_condwait_t *s)
{
	int retval = 0;
	mutex_lock(&s->m);
	retval = cond_wait(&s->c, &s->m);
	mutex_unlock(&s->m);
	return retval;
}

static inline int darc_condwait_wait_ifvalue(darc_condwait_t *s, unsigned value)
{
	int retval = 0;
	mutex_lock(&s->m);
	if (s->value==value)
		retval = cond_wait(&s->c, &s->m);
	mutex_unlock(&s->m);
	return retval;
}

static inline int darc_condwait_timedwait(darc_condwait_t *s, const struct timespec *to)
{
	int retval = 0;
	mutex_lock(&s->m);
	retval = cond_timedwait(&s->c, &s->m, to);
	mutex_unlock(&s->m);
	return retval;
}

static inline int darc_condwait_timedwait_ifvalue(darc_condwait_t *s, const struct timespec *to, unsigned value)
{
	int retval = 0;
	mutex_lock(&s->m);
	if (s->value==value)
		retval = cond_timedwait(&s->c, &s->m, to);
	mutex_unlock(&s->m);
	return retval;
}

static inline int darc_condwait_signal(darc_condwait_t *s)
{
	return cond_signal(&s->c);
}

static inline int darc_condwait_signal_withvalue(darc_condwait_t *s, unsigned value)
{
	xchg_32(&s->value,value);
	return cond_signal(&s->c);
}

static inline int darc_condwait_broadcast(darc_condwait_t *s)
{
	return cond_broadcast(&s->c);
}

static inline int darc_condwait_broadcast_withvalue(darc_condwait_t *s, unsigned value)
{
	xchg_32(&s->value,value);
	return cond_broadcast(&s->c);
}

static inline int darc_condwait_setvalue(darc_condwait_t *s, unsigned value)
{
	xchg_32(&s->value,value);
	return 0;
}

#else
static inline int darc_condwait_wait(darc_condwait_t *s)
{
	int value = s->value;
	return sys_futex(&s->value, FUTEX_WAIT | s->flags, value, NULL, NULL, 0);
}

static inline int darc_condwait_wait_ifvalue(darc_condwait_t *s, unsigned value)
{
	return sys_futex(&s->value, FUTEX_WAIT | s->flags, value, NULL, NULL, 0);
}

static inline int darc_condwait_timedwait(darc_condwait_t *s, const struct timespec *to)
{
	int value = s->value;
	int retval = sys_futex(&s->value, FUTEX_WAIT | s->flags, value, to, NULL, 0);
	if (retval == -1 && errno == ETIMEDOUT)
		retval = ETIMEDOUT;
	return retval;
}

static inline int darc_condwait_timedwait_ifvalue(darc_condwait_t *s, const struct timespec *to, unsigned value)
{
	int retval = sys_futex(&s->value, FUTEX_WAIT | s->flags, value, to, NULL, 0);
	if (retval == -1 && errno == ETIMEDOUT)
		retval = ETIMEDOUT;
	return retval;
}

static inline int darc_condwait_signal(darc_condwait_t *s)
{
	/* Wake up a thread */
	return sys_futex(&s->value, FUTEX_WAKE | s->flags, 1, NULL, NULL, 0);
}

static inline int darc_condwait_broadcast(darc_condwait_t *s)
{
	/* Wake up INT_MAX threads */
	return sys_futex(&s->value, FUTEX_WAKE | s->flags, INT_MAX, NULL, NULL, 0);
}

static inline int darc_condwait_signal_withvalue(darc_condwait_t *s, unsigned value)
{
	/* Atomically set the value */
	xchg_32(&s->value, value);
	/* Wake up a thread */
	return sys_futex(&s->value, FUTEX_WAKE | s->flags, 1, NULL, NULL, 0);
}

static inline int darc_condwait_broadcast_withvalue(darc_condwait_t *s, unsigned value)
{
	/* Atomically set the value */
	xchg_32(&s->value, value);
	/* Wake up INT_MAX threads */
	return sys_futex(&s->value, FUTEX_WAKE | s->flags, INT_MAX, NULL, NULL, 0);
}

static inline int darc_condwait_setvalue(darc_condwait_t *s, unsigned value)
{
	/* Atomically set the value */
	xchg_32(&s->value, value);
	return 0;
}

#endif

typedef struct pool_barrier pool_barrier_t;
struct pool_barrier
{
	union
	{
		struct
		{
			unsigned seq;
			unsigned count;
		};
		unsigned long long reset;
	};
	unsigned refcount;
	unsigned total;
	unsigned flags;
	unsigned pad;
};

int pool_barrier_init(pool_barrier_t *b, pthread_barrierattr_t *a, unsigned count);

int pool_barrier_destroy(pool_barrier_t *b);

static inline int pool_barrier_wait(pool_barrier_t *b)
{
	int ret;

	atomic_add(&b->refcount, 1);

	while (1)
	{
		unsigned seq = atomic_read(b->seq);
		unsigned count = atomic_xadd(&b->count, 1);

		if (count < b->total)
		{
			/* Can we proceed? */
			while (atomic_read(b->seq) == seq)
			{
				/* Sleep on it instead */
				sys_futex(&b->seq, FUTEX_WAIT | b->flags, seq, NULL, NULL, 0);
			}

			ret = 0;
			break;
		}

		if (count == b->total)
		{
			/* Simultaneously clear count, and increment sequence number */
			barrier();
			b->reset = b->seq + 1;
			barrier();

			/* Wake up sleeping threads */
			sys_futex(&b->seq, FUTEX_WAKE | b->flags, INT_MAX, NULL, NULL, 0);

			ret = PTHREAD_BARRIER_SERIAL_THREAD;
			break;
		}

		/* We were too slow... wait for the barrier to be released */
		sys_futex(&b->seq, FUTEX_WAIT | b->flags, seq, NULL, NULL, 0);
	}

	/* Are we the last to wake up? */
	if (atomic_xadd(&b->refcount, -1) == 1)
	{
		/* Wake destroying thread */
		sys_futex(&b->refcount, FUTEX_WAKE | b->flags, 1, NULL, NULL, 0);
	}
	return ret;
}

typedef struct fast_barrier_t fast_barrier_t;
struct fast_barrier_t
{
	union
	{
		struct
		{
			unsigned seq;
			unsigned count;
		};
		unsigned long long reset;
	};
	unsigned refcount;
	unsigned total;
	int spins;
	unsigned flags;
};

int fast_barrier_init(fast_barrier_t *b, pthread_barrierattr_t *a, unsigned count);

int fast_barrier_destroy(fast_barrier_t *b);

static inline int fast_barrier_wait(fast_barrier_t *b)
{
	int ret;

	atomic_add(&b->refcount, 1);

	while (1)
	{
		unsigned seq = atomic_read(b->seq);
		unsigned count = atomic_xadd(&b->count, 1);

		if (count < b->total)
		{
			int i;
			seq |= 1;

			for (i = 0; i < b->spins; i++)
			{
				if ((atomic_read(b->seq) | 1) != seq) break;
			}

			/* Can we proceed? */
			while ((atomic_read(b->seq) | 1) == seq)
			{
				/* Hack - set a flag that says we are sleeping */
				*(volatile char *) &b->seq = 1;

				/* Sleep on it instead */
				sys_futex(&b->seq, FUTEX_WAIT | b->flags, seq, NULL, NULL, 0);
			}

			ret = 0;
			break;
		}

		if (count == b->total)
		{
			/* Simultaneously clear count, increment sequence number, and clear wait flag */
			seq = atomic_read(b->seq);

			if (xchg_64(&b->reset, (seq | 1) + 255) & 1)
			{
				/* Wake up sleeping threads */
				sys_futex(&b->seq, FUTEX_WAKE | b->flags, INT_MAX, NULL, NULL, 0);
			}

			ret = PTHREAD_BARRIER_SERIAL_THREAD;
			break;
		}

		seq |= 1;
		/* Hack - set a flag that says we are sleeping */
		*(volatile char *) &b->seq = 1;

		/* We were too slow... wait for the barrier to be released */
		sys_futex(&b->seq, FUTEX_WAIT | b->flags, seq, NULL, NULL, 0);
	}

	/* Are we the last to wake up? */
	if (atomic_xadd(&b->refcount, -1) == 1)
	{
		/* Wake destroying thread */
		sys_futex(&b->refcount, FUTEX_WAKE | b->flags, 1, NULL, NULL, 0);
	}
	return ret;
}

typedef struct ticket_barrier_t ticket_barrier_t;
struct ticket_barrier_t
{
	unsigned count_next;
	unsigned total;
	union
	{
		struct
		{
			unsigned count_in;
			unsigned count_out;
		};
		unsigned long long reset;
	};
	unsigned flags;
	unsigned pad;
};

int ticket_barrier_init(ticket_barrier_t *b, pthread_barrierattr_t *a, unsigned count);

int ticket_barrier_destroy(ticket_barrier_t *b);

static inline int ticket_barrier_wait(ticket_barrier_t *b)
{
	unsigned wait = atomic_xadd(&b->count_in, 1);
	unsigned next;
	unsigned long long temp;

	int ret;

	while (1)
	{
		next = atomic_read(b->count_next);

		/* Have the required number of threads entered? */
		if (wait - next == b->total)
		{
			/* Move to next bunch */
			b->count_next += b->total;

			/* Wake up waiters */
			sys_futex(&b->count_next, FUTEX_WAKE | b->flags, INT_MAX, NULL, NULL, 0);

			ret = PTHREAD_BARRIER_SERIAL_THREAD;

			break;
		}

		/* Are we allowed to go? */
		if (wait - next >= (1UL << 31))
		{
			ret = 0;
			break;
		}

		/* Go to sleep until our bunch comes up */
		sys_futex(&b->count_next, FUTEX_WAIT | b->flags, next, NULL, NULL, 0);
	}

	/* Add to count_out, simultaneously reading count_in */
	temp = atomic_xadd(&b->reset, 1ULL << 32);

	/* Does count_out == count_in? */
	if ((temp >> 32) == (unsigned) temp)
	{
		/* Notify destroyer */
		sys_futex(&b->count_out, FUTEX_WAKE | b->flags, 1, NULL, NULL, 0);
	}

	return ret;
}


/* spincond* functions are under construction
   to be used for spinning on a condition */

int spincond_init(spincond_t *c);

int spincond_destroy(spincond_t *c);

static inline int spincond_wait(spincond_t *c, mutex_t *m)
{
	unsigned bcast = c->bcast;
	mutex_unlock(m);
	while (1)
	{
		if (cmpxchg(&c->sig,1,0)) break;
		if (atomic_read(c->bcast)!=bcast) break;
	}
	mutex_lock(m);
	return 0;
}

static inline int spincond_timedwait(spincond_t *c, mutex_t *m, const struct timespec *to)
{
	unsigned bcast = c->bcast;
	struct timespec t1;
	struct timespec t2;
	clock_gettime(CLOCK_REALTIME, &t1);
	t1.tv_sec+=to->tv_sec;
	t1.tv_nsec+=to->tv_nsec;
	if (t1.tv_nsec > 999999999) {
		t1.tv_sec++;
		t1.tv_nsec-=1000000000;
	}
	volatile int rt = 0;
	mutex_unlock(m);
	while (rt==0)
	{
		if (cmpxchg(&c->sig,1,0)) break;
		if (atomic_read(c->bcast)!=bcast) break;
		clock_gettime(CLOCK_REALTIME, &t2);
		rt = (t2.tv_sec>t1.tv_sec || (t2.tv_sec>=t1.tv_sec && t2.tv_nsec>t1.tv_nsec));
	}
	mutex_lock(m);
	return rt*ETIMEDOUT;
}

static inline int spincond_signal(spincond_t *c)
{
	c->sig = 1;
	return 0;
}

static inline int spincond_broadcast(spincond_t *c)
{
	c->bcast++;
	return 0;
}

#endif // header guard