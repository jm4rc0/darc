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

/* atomic option */
#ifdef USEATOMICS
#include <stdatomic.h>
#define darc_set(x,y) atomic_store((x),(y))
#define darc_check(x,y) (atomic_load(x)==(y))
#else
#define darc_set(x,y) (*(x)=(y))
#define darc_check(x,y) (*(x)==(y))
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

#define darc_fast_cond_t volatile char
#define darc_fast_cond_init(x,y) (*(x)=0)
#define darc_fast_cond_wait(x,y) do {\
    const char local_cond = *(x);\
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
#define darc_cond_broadcast(x) pthread_cond_broadcast(x)
#define darc_cond_destroy(x) pthread_cond_destroy(x)
#define darc_cond_signal(x) phread_cond_signal(x)
#else
#define darc_cond_t volatile char
#define darc_cond_init(x,y) (*(x)=0)
#define darc_cond_wait(x,y) do {\
                const char local_cond = *(x);\
                darc_mutex_unlock(y);\
                darc_wait(x,local_cond);\
                darc_mutex_lock(y);\
                } while (0)
#define darc_cond_broadcast(x) do {\
                int local_cond = *(x);\
                local_cond++;\
                darc_set(x,local_cond%10);\
                } while (0)
#define darc_cond_destroy(x) (*x=10)
#endif