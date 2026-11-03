
//  Copyright (c) 2003-2026 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#ifdef _WIN32
	#ifndef WIN32_LEAN_AND_MEAN
	#define WIN32_LEAN_AND_MEAN
	#endif
	#include <Windows.h>
#endif

#include "threading.h"
#include <signal.h>
#include <time.h>
#include <xstypes/xstime.h>
#include <cassert>

#ifdef __GNUC__
	#include <sys/time.h>
	#include <signal.h>
	#include <fcntl.h>
	#include <sys/stat.h>
	#define _strdup	strdup
#endif

namespace xsens
{
#ifdef __GNUC__
pthread_t xsStartThread(void* (func)(void*), void* param, void* pid)
{
	return ::xsStartThread(func, param, pid);
}
#endif

/*! Constructor, creates the necessary signals. Does NOT start the thread.
*/
StandardThread::StandardThread()
	: m_stop(false)
	, m_yieldOnZeroSleep(true)
#ifdef _WIN32
	, m_stopHandle(::CreateEvent(nullptr, TRUE, FALSE, nullptr))
#endif
	, m_thread(XSENS_INVALID_THREAD)
	, m_threadId{}
	, m_priority(XS_THREAD_PRIORITY_NORMAL)
	, m_finished(true)
	, m_killRequested(false)
	, m_name(nullptr)
{
#ifndef _WIN32
	pthread_attr_init(&m_attr);
#endif
}

/*! Destructor, stops the thread if it was running and cleans up afterwards.
*/
StandardThread::~StandardThread()
{
	stopThread(); // NOSONAR cpp:S1699 - stopThread is non-virtual; the virtual signalStopThread it reaches cannot dispatch to a subclass during base destruction, which is the correct semantic here

	if (m_name)
	{
		free(m_name); // NOSONAR cpp:S1231 - pairs with _strdup; replacing char* m_name with std::string is an ABI change in the public threading.h
		m_name = nullptr;
	}

#ifdef _WIN32
	::CloseHandle(m_stopHandle);
#else
	pthread_attr_destroy(&m_attr);
#endif
}

/*! \brief Kill the thread now.
 *
 * Use as a last resort, resources will leak.
 * Call will block until thread is killed and cleaned up to avoid race conditions when restarting or destructing.
 *
 * Warnings:
 * - Thread termination is not immediate on linux as it depends on cancellation points.
 * - Glibc implements this as an exception. Any active noexcept functions may crash the program.
 */
void StandardThread::terminateThread()
{
	// cannot kill self as that would leave everything in a big mess
	if (isCurrentThread())
	{
		assert(false && "attempted to terminate thread from self");
		return;
	}

	// If stopThread() is blocking us while holding m_mux then they will perform the kill for us
	m_killRequested = true;

	// Prevent multiple threads from doing this at the same time
	xsens::Lock locky(&m_mux);

	if (m_thread != XSENS_INVALID_THREAD)
	{
#ifdef _WIN32
		::TerminateThread(m_thread, DWORD(-1));
		::WaitForSingleObject(m_thread, INFINITE);
		::CloseHandle(m_thread);
#elif defined(__ANDROID__)
		// Bionic has never provided pthread_cancel, so a thread cannot be
		// force-terminated on Android. Degrade to waiting for it: join without
		// cancelling. That can block where the other platforms would kill, so this is
		// a deliberate degradation, not a silent equivalence.
		//
		// Nothing reachable changes today: terminateThread() has no callers anywhere in
		// the XME SDK closure, and the only test exercising it
		// (xscommon/test/testthread.cpp) is itself #ifdef _WIN32.
		pthread_join(m_thread, nullptr);
#else
		pthread_cancel(m_thread);
		pthread_join(m_thread, nullptr);
#endif
		m_thread = XSENS_INVALID_THREAD;
		m_threadId = XsThreadId{};
		m_finished = true;
	}

	m_killRequested = false;
}

//! \returns True if the current thread is the thread managed by this StandardThread instance
bool StandardThread::isCurrentThread() const
{
	XsThreadId id = m_threadId;
#ifdef _WIN32
	return id != 0 && xsGetCurrentThreadId() == id;
#else
	return id != XsThreadId{} && pthread_equal(xsGetCurrentThreadId(), id);
#endif
}

/*! \returns True if the thread is alive, false if the thread is already dead or is joinable.
*/
bool StandardThread::isAlive(void) volatile const noexcept
{
	// Do not read m_thread here as there may be a data race on stop/start.
	return !m_finished;
}

/*! \brief Returns whether the thread is currently running the thread main function.
*/
bool StandardThread::isRunning(void) volatile const noexcept
{
	// There is no meaningful difference between isAlive and isRunning.
	// There could have been (isAlive <=> isJoined) but that would
	// change the public contract of this class and we can't do that.
	return isAlive();
}

/*! \brief Returns whether the thread should (have) terminate(d)
*/
bool StandardThread::isTerminating(void) volatile const noexcept
{
	return m_stop;
}

/*! \brief Sets the priority of the thread
	\param pri The thread priority to set
	\return True if successful
*/
bool StandardThread::setPriority(XsThreadPriority pri)
{
	// There's no data race when calling from self so we skip locking to prevent deadlock risks.
	xsens::Lock locky(&m_mux, false);
	if (!isCurrentThread())
		locky.lock();

	m_priority = pri;
	return setPriorityLocked(pri);
}


/*! \brief Sets the priority of the thread.
           Internal version requires lock to be held or to be called from the managed thread.
	\param pri The thread priority to set
	\return True if successful
*/
bool StandardThread::setPriorityLocked(XsThreadPriority pri)
{
	if (!isAlive())
		return false;

#ifdef _WIN32
	::SetThreadPriority(m_thread, (int) pri);
#elif defined(_POSIX_PRIORITY_SCHEDULING)
	int32_t rv;
	int32_t policy;
	struct sched_param param;

	rv = pthread_getschedparam(m_thread, &policy, &param);
	switch (rv)
	{
		case ESRCH:
			/* The value specified by thread does not refer to an existing thread. */
			return false;
		default:
			break;
	}

	switch (pri)
	{
		case XS_THREAD_PRIORITY_HIGHEST:
			param.sched_priority = sched_get_priority_max(policy);
			break;

		case XS_THREAD_PRIORITY_LOWEST:
			param.sched_priority = sched_get_priority_min(policy);
			break;

		default:
			/* we need to map the priority to the values used on this system */
			int32_t min_prio = sched_get_priority_min(policy);
			int32_t max_prio = sched_get_priority_max(policy);

			if (min_prio < 0 || max_prio < 0)
				return false;

			/* divide range up in amount of priority levels */
			float priostep = ((float)(max_prio - min_prio)) / ((int32_t)XS_THREAD_PRIORITY_HIGHEST + 1);

			param.sched_priority = (int32_t)(min_prio + ((int32_t)pri * priostep));
			break;
	}

	rv = pthread_setschedparam(m_thread, policy, &param);
	switch (rv)
	{
		case ESRCH:
		/* The value specified by thread does not refer to an existing thread. */
		case EINVAL:
		/*  The  value  specified by policy or one of the scheduling parameters
			 associated with the scheduling policy policy is invalid.
		*/
		case ENOTSUP:
		/*  An attempt was made to set the policy or scheduling parameters to an
			 unsupported value.
			 An attempt was made to dynamically change the scheduling policy to
			 SCHED_SPORADIC, and the implementation does not support this change.
		*/
		case EPERM:
			/*  The caller does not have the appropriate permission to set either the
				 scheduling parameters or the scheduling policy of the specified thread.
				 The implementation does not allow the application to modify one of the
				 parameters to the value specified.
			*/
			return false;
		default:
			break;
	}
#endif
	return true;
}

/*! \brief Starts the thread.
	\param name The name of the thread as shown in the debugger, may be nullptr in which case the system determines the name.
	\returns True if successful
*/
bool StandardThread::startThread(const char* name)
{
	// Prevent multiple threads from starting a new thread at the same time
	xsens::Lock locky(&m_mux);

	// Do nothing if the old thread is still active
	if (!m_finished)
		return false;

	// Join the old thread if it hasn't happened yet
	stopThreadLocked();

	if (m_name)
	{
		free(m_name); // NOSONAR cpp:S1231 - pairs with _strdup; replacing char* m_name with std::string is an ABI change in the public threading.h
		m_name = nullptr;
	}
	if (name)
		m_name = _strdup(name);

	m_stop = false;
	m_finished = false;
	m_killRequested = false;

#ifdef _WIN32
	::ResetEvent(m_stopHandle);
	XsThreadId tid = 0;
	m_thread = xsStartThread(&threadInit, this, &tid);
	if (!m_thread)
	{
		/* something went wrong */
		m_thread = XSENS_INVALID_THREAD;
		m_finished = true;
		return false;
	}
	m_threadId = tid;
#else
	if (pthread_create(&m_thread, &m_attr, threadInit, this))
	{
		/* something went wrong */
		m_thread = XSENS_INVALID_THREAD;
		m_finished = true;
		return false;
	}
	m_threadId = m_thread;
#endif

	setPriorityLocked(m_priority);
	return true;
}

/*! \brief Tells the thread to stop but does not wait for it to end.
*/
void StandardThread::signalStopThread(void)
{
	m_stop = true;
#ifdef _WIN32
	::SetEvent(m_stopHandle);
#endif

	// Need a lock before accessing m_thread. Non-critical so best effort attempt.
	if (xsens::Lock locky(&m_mux, false); locky.tryLock())
		setPriorityLocked(XS_THREAD_PRIORITY_HIGH);
}

/*! \brief Tells the thread to stop and waits for it to end.
*/
void StandardThread::stopThread(void) noexcept
{
	// When called by ourselves we can only signal, never block, so we bail out early
	if (isCurrentThread())
	{
		signalStopThread();
		return;
	}

	// Prevent m_thread being stopped by multiple callers
	xsens::Lock locky(&m_mux);
	stopThreadLocked();
}

/*! \brief Tells the thread to stop and waits for it to end. Internal version requires lock to be held.
*/
void StandardThread::stopThreadLocked(void) noexcept
{
	// Only continue if the thread has not been joined yet
	if (m_thread == XSENS_INVALID_THREAD)
		return;

	// Signal the main loop to terminate
	signalStopThread();

#ifdef _WIN32
	// Wait in slices so an emergency terminateThread request can be honored
	while (::WaitForSingleObject(m_thread, 10) == WAIT_TIMEOUT)
	{
		if (m_killRequested)
		{
			::TerminateThread(m_thread, DWORD(-1));
			::WaitForSingleObject(m_thread, INFINITE);
			break;
		}
	}
	::CloseHandle(m_thread);
#else
	// pthread timed join is not portable so instead we wait and poll
	while (!m_finished)
	{
		if (m_killRequested)
		{
#ifndef __ANDROID__
			pthread_cancel(m_thread);
#endif
			// On Android there is no pthread_cancel (see terminateThread above), so we
			// stop polling and fall through to the join, which waits for the thread
			// instead of killing it. m_killRequested is only ever set by
			// terminateThread(), which has no callers in the SDK closure.
			break;
		}
		XsTime::msleep(1);
	}

	// pthread blocks until the thread finishes
	pthread_join(m_thread, nullptr);
#endif

	m_thread = XSENS_INVALID_THREAD;
	m_threadId = XsThreadId{};
	m_finished = true;
}

XSENS_THREAD_RETURN StandardThread::threadInit(void* obj)
{
	StandardThread* thread = static_cast<StandardThread*>(obj);
	thread->m_threadId = xsGetCurrentThreadId();
	if (thread->m_name)
		xsNameThisThread(thread->m_name);

	thread->threadMain();

	// Clear threadId to prevent races with threadid reuse.
	thread->m_threadId = XsThreadId{};
	thread->m_finished = true;
	return 0;
}

/*! \brief The inner loop of the thread, calls innerFunction repeatedly and sleeps when necessary
*/
void StandardThread::threadMain(void)
{
	initFunction();
	do
	{
		int64_t rv = innerFunction();
		if (rv > 0)
		{
			int64_t sleepStart = XsTimeStamp::nowMs();
			while (!m_stop)
			{
				int64_t timePassed = XsTimeStamp::nowMs() - sleepStart;
				// system clock might be changed (e.g. to a point in time in the past)
				// then XsTimeStamp::nowMs() would be smaller than the sleepStart
				if (timePassed < 0)
					break;

				// sleep max 100ms at a time so we can terminate the thread quickly if necessary
				int64_t remaining = rv - timePassed;
				if (remaining > 100)
					XsTime::msleep(100);
				else if (remaining <= 0)
					break;
				else
					XsTime::msleep((uint32_t) remaining);
			}
		}
		else if (m_yieldOnZeroSleep && !m_stop)
			xsYield();
	} while (!m_stop);
	exitFunction();
}

//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

WatchDogThread::WatchDogThread(WatchDogFunction func, void* param)
	: m_thread(XSENS_INVALID_THREAD)
#ifdef _WIN32
	, m_stop(::CreateEvent(nullptr, TRUE, FALSE, nullptr))
	, m_running(::CreateEvent(nullptr, TRUE, FALSE, nullptr))
	, m_reset(::CreateEvent(nullptr, TRUE, FALSE, nullptr))
#else
	, m_running(false)
	, m_reset(false)
	, m_stop(false)
	, m_finished(false)
#endif
	, m_timeout(10000)
	, m_func(func)
	, m_param(param)
	, m_name(nullptr)
	, m_threadId(0)
{
#ifndef _WIN32
	pthread_attr_init(&m_attr);
#endif
}

WatchDogThread::~WatchDogThread()
{
	stopTimer();

	if (m_name)
	{
		free(m_name); // NOSONAR cpp:S1231 - pairs with _strdup; replacing char* m_name with std::string is an ABI change in the public threading.h
		m_name = nullptr;
	}

#ifdef _WIN32
	::CloseHandle(m_reset);
	::CloseHandle(m_running);
	::CloseHandle(m_stop);
#else
	pthread_attr_destroy(&m_attr);
#endif
}

bool WatchDogThread::isAlive(void) volatile const noexcept
{
	if (m_thread == XSENS_INVALID_THREAD)
		return false;

#ifdef _WIN32
	if (DWORD exitCode; ::GetExitCodeThread(m_thread, &exitCode))
		return (exitCode == STILL_ACTIVE);

	return false;
#else
	return !m_finished;
#endif
}

bool WatchDogThread::isRunning(void) volatile const noexcept
{
	if (!isAlive())
		return false;
#ifdef _WIN32
	switch (::WaitForSingleObject(m_running, 0))
	{
		case WAIT_ABANDONED:
		case WAIT_TIMEOUT:
			return false;
		case WAIT_OBJECT_0:
			switch (::WaitForSingleObject(m_stop, 0))
			{
				case WAIT_ABANDONED:
				case WAIT_TIMEOUT:
					return true;
				case WAIT_OBJECT_0:
					return false;
				default:
					return false;
			}
		default:
			return false;
	}
#else
	return m_running;
#endif
}

/*! \brief Starts a timer using some parameters
	\param timeout The timeout value to use
	\param name The name of a thread to set
	\returns True if successful
*/
bool WatchDogThread::startTimer(uint32_t timeout, const char* name)
{
	if (isAlive())
		return false;

	if (timeout != 0)
		m_timeout = timeout;

	if (m_name)
		free(m_name); // NOSONAR cpp:S1231 - pairs with _strdup; replacing char* m_name with std::string is an ABI change in the public threading.h
	if (name)
		m_name = _strdup(name);
	else
		m_name = nullptr;

#ifdef _WIN32
	::ResetEvent(m_stop);
	::SetEvent(m_running);
	::ResetEvent(m_reset);
	m_thread = xsStartThread(&threadInit, this, &m_threadId);
	if (!m_thread)
	{
		m_thread = XSENS_INVALID_THREAD;
		::ResetEvent(m_running);
		return false;
	}

#else
	m_running = true;
	m_reset = false;
	m_stop = false;
	m_finished = false;
	if (pthread_create(&m_thread, &m_attr, threadInit, this) != 0)
	{
		/* something went wrong */
		m_thread = XSENS_INVALID_THREAD;
		m_finished = true;
		return false;
	}
#endif
	return true;
}

/*! \brief Stops the timer
	\returns True if successful
*/
bool WatchDogThread::stopTimer(void) noexcept
{
	if (m_thread == XSENS_INVALID_THREAD)
		return true;

#ifdef _WIN32
	::SetEvent(m_stop);
	::SetThreadPriority(m_thread, THREAD_PRIORITY_ABOVE_NORMAL);

	while (isAlive())
		xsYield();
	if (::CloseHandle(m_thread) == 0)
	{
		DWORD lastErr = GetLastError();
		(void)lastErr;
		return false;
	}
#else
	int rv;
	int32_t policy;
	struct sched_param param;

	m_stop = true;
	rv = pthread_getschedparam(m_thread, &policy, &param);
	if (rv)
	{
		switch (errno)
		{
			case ESRCH:
				return false;
			default:
				break;
		}
	}
	param.sched_priority = sched_get_priority_max(policy);

	rv = pthread_setschedparam(m_thread, policy, &param);
	if (rv)
	{
		switch (errno)
		{
			case ESRCH:
			case EINVAL:
			case ENOTSUP:
			case EPERM:
				return false;
			default:
				break;
		}
	}

	rv = pthread_join(m_thread, nullptr);
	if (rv)
	{
		switch (errno)
		{
			case EINVAL:
			case ESRCH:
			case EDEADLK:
			default:
				break;
		}
	}
	m_running = false;
#endif
	m_thread = XSENS_INVALID_THREAD;
	return true;
}

XSENS_THREAD_RETURN WatchDogThread::threadInit(void* obj)
{
	WatchDogThread* thread = static_cast<WatchDogThread*>(obj);
	if (thread->m_name)
		xsNameThisThread(thread->m_name);

	thread->threadMain();
#ifndef _WIN32
	thread->m_finished = true;
#endif
	return 0;
}

void WatchDogThread::threadMain(void)
{
	XsTimeStamp toTime(XsTimeStamp::now().msTime() + m_timeout);

#ifdef _WIN32
	HANDLE hlist[2];
#endif
	bool go = true;
	while (go)
	{
#ifdef _WIN32
		uint32_t timeout = (uint32_t)(toTime - XsTimeStamp::now()).msTime();
		if (timeout > m_timeout)
			break;
		hlist[0] = m_reset;
		hlist[1] = m_stop;

		switch (::WaitForMultipleObjects(2, hlist, FALSE, timeout))
		{
			case WAIT_OBJECT_0:	// m_reset
				// received an update for the timer
				toTime = XsTimeStamp::now() + XsTimeStamp((int) m_timeout);
				::ResetEvent(m_reset);
				continue;

			case WAIT_TIMEOUT:
				break;

			case WAIT_OBJECT_0+1:	// m_stop
			case WAIT_ABANDONED:
			default:
				go = false;
				break;
		}
#else
		Lock lock(&m_mutex, true);
		lock.unlock();

		uint32_t timeout;

		while (go)
		{
			timeout = static_cast<uint32_t>((toTime - XsTimeStamp::now()).msTime());
			if (timeout > m_timeout)
				break;

			lock.lock();
			if (m_reset)
			{
				toTime = XsTimeStamp((XsTimeStamp::now().msTime() + m_timeout));
				m_reset = false;
			}
			lock.unlock();

			if (m_stop)
				go = false;
		}
#endif
		break;
	}
	if (go)
		m_func(m_param);
}

/*! \brief Resets the timer and sets a timeout
	\param timeout The timeout value to set
	\returns True if successful
*/
bool WatchDogThread::resetTimer(uint32_t timeout)
{
	if (!isRunning())
		return false;

#ifdef _WIN32
	if (timeout != 0)
		m_timeout = timeout;
	::SetEvent(m_reset);
#else
	Lock lock(&m_mutex);

	m_timeout = timeout;
	m_reset = true;
#endif
	return true;
}

#ifdef _WIN32
Semaphore::Semaphore(int32_t initVal, uint32_t nofOtherHandles, HANDLE* otherHandles)
{
	m_nofHandles = nofOtherHandles + 1;
	m_handleList = new HANDLE[m_nofHandles];
	for (uint32_t i = 0; i < m_nofHandles - 1; i++)
		m_handleList[i] = otherHandles[i];
	m_handleList[m_nofHandles - 1] = CreateSemaphore(nullptr, initVal, 0x7fffffff, nullptr);
}

Semaphore::~Semaphore(void)
{
	post();
	CloseHandle(m_handleList[m_nofHandles - 1]);
	delete[] m_handleList;
}

bool Semaphore::wait1()
{
	return wait1(static_cast<uint32_t>(-1));
}

bool Semaphore::wait1(uint32_t timeout)
{
	for (;;) // loop + timeout for debugging only
	{
		DWORD r = WaitForMultipleObjects(m_nofHandles, m_handleList, FALSE, timeout);
		if (r == WAIT_TIMEOUT)
			return false;
		if (r < WAIT_OBJECT_0 + m_nofHandles)
			return r - WAIT_OBJECT_0 == m_nofHandles - 1;
		if (r != WAIT_TIMEOUT)
			return false;
		xsYield();	// prevent race condition
	}
}

int32_t Semaphore::post(int32_t increment) noexcept
{
	LONG prev;
	ReleaseSemaphore(m_handleList[m_nofHandles - 1], increment, &prev);
	return (int32_t) prev;
}

#else // _WIN32

Semaphore::Semaphore(int32_t initVal, uint32_t, sem_t*) :
	m_semname(nullptr),
	m_handle(SEM_FAILED)
{
#if defined(SEM_VALUE_MAX) && INT32_MAX > SEM_VALUE_MAX
	if (initVal > SEM_VALUE_MAX)
		initVal = SEM_VALUE_MAX;
#endif

	uint64_t id = (uint64_t)this;
	char semname[20];

	while (true)
	{
		sprintf(semname, "%" PRINTF_INT64_MODIFIER "x", id);
		m_semname = strdup(semname);
		m_handle = sem_open(semname, O_EXCL | O_CREAT, S_IRWXU, initVal);
		if (m_handle != SEM_FAILED)
			break;

		if (errno != EEXIST)
		{
			perror("opening semaphore");
			exit(-1);
		}

		id++;
		free(m_semname);
	}
}

Semaphore::~Semaphore(void)
{
	post();
	sem_unlink(m_semname);
	free(m_semname);
}

bool Semaphore::wait1()
{
	return wait1(static_cast<uint32_t>(-1));
}

bool Semaphore::wait1(uint32_t ms)
{
	if (ms == UINT32_MAX)
		return sem_wait(m_handle);

#if _POSIX_C_SOURCE >= 200112L || _XOPEN_SOURCE >= 600
	timespec ts;

	clock_gettime(CLOCK_REALTIME, &ts);

	int64_t s = ms / 1000;
	ts.tv_sec += s;

	ms -= s * 1000;
	int64_t ns = ts.tv_nsec + (ms * 1000000);

	ts.tv_sec += ns / 1000000000L;
	ts.tv_nsec = ns % 1000000000L;

	return sem_timedwait(m_handle, &ts) == 0;
#else
	XsTimeStamp end = XsTimeStamp::now() + XsTimeStamp(int64_t(ms));

	while (XsTimeStamp::now() < end)
	{
		if (sem_trywait(m_handle) == 0)
			return true;
		else
			xsYield();
	}

	return false;
#endif
}

#ifndef __APPLE__
int32_t Semaphore::post(int32_t increment) throw()
{
	int prev;
	sem_getvalue(m_handle, &prev);
	for (int i = 0; i < increment; i++)
		sem_post(m_handle);
	return (int32_t)prev;
}
#else
int32_t Semaphore::post(int32_t increment) throw()
{
	int prev = 0;
	for (int i = 0; i < increment; i++)
		sem_post(m_handle);
	return (int32_t)prev;
}
#endif
#endif // _WIN32

#ifdef __APPLE__
/*! \brief An implementation of linux' clock_gettime()

	clock_gettime() is not available on Apple/Darwin platforms. This function helps
	maintaining compatibility with Linux code.

	This comes straight from xstime. We should probably move thread primitives into xstypes.
*/
#ifndef CLOCK_REALTIME
	#define CLOCK_REALTIME 0
#endif
static int clock_gettime(int clk_id, struct timespec* tp)
{
	(void)clk_id;
	struct timeval now;

	int rv = gettimeofday(&now, nullptr);
	if (rv != 0)
		return rv;

	tp->tv_sec = now.tv_sec;
	tp->tv_nsec = now.tv_usec * 1000;

	return 0;
}
#endif

/*! \brief Create a wait condition */
WaitCondition::WaitCondition(Mutex& m) : m_mutex(m)
{
#ifdef _WIN32
	InitializeConditionVariable(&m_cond);
#else
	pthread_condattr_init(&m_condattr);
#if !defined(__APPLE__) && (defined(_POSIX_CLOCK_SELECTION) || defined(_SC_CLOCK_SELECTION)) && !defined(ANDROID)
	m_clockId = CLOCK_MONOTONIC;
	if (pthread_condattr_setclock(&m_condattr, m_clockId) != 0)
		pthread_condattr_getclock(&m_condattr, &m_clockId);
#else
	/*  we'll fall back to the wallclock, but we may be influenced by
		 time keeping software (ntp, date), which doesn't happen with
		 the monotonic clock
	*/
	m_clockId = CLOCK_REALTIME;
#endif
	pthread_cond_init(&m_cond, &m_condattr);
#endif
}

/*! \brief Destroy the wait condition */
WaitCondition::~WaitCondition()
{
	try
	{
		broadcast();
#ifndef _WIN32
		pthread_cond_destroy(&m_cond);
		pthread_condattr_destroy(&m_condattr);
#endif
	}
	catch (...) // NOSONAR cpp:S2486 - destructor; an escaping exception would terminate the process
	{
		// Deliberately swallowed: a destructor must not let an exception escape.
	}
}

/*! \brief Unblock a single waiting thread */
void WaitCondition::signal()
{
#ifdef _WIN32
	WakeConditionVariable(&m_cond);
#else
	pthread_cond_signal(&m_cond);
#endif
}

/*! \brief Unblock all waiting threads */
void WaitCondition::broadcast()
{
#ifdef _WIN32
	WakeAllConditionVariable(&m_cond);
#else
	pthread_cond_broadcast(&m_cond);
#endif
}

/*! \brief Wait until we're signalled to continue

	 \details Before calling this function, it is required that the mutex
	 provided during construction is _locked_. This function unlocks the mutex
	 internally, and returns with the mutex locked again.

	 \return true if we were signalled, false otherwise
*/
bool WaitCondition::wait()
{
	// The native lock is held exactly once per ownership, so the wait can release it; the owner bookkeeping is ours to park
	Mutex::Ownership const parked = m_mutex.suspendOwnership();
#ifdef _WIN32
	bool const rv = SleepConditionVariableCS(&m_cond, &m_mutex.m_mutex, INFINITE) != 0;
#else
	bool const rv = pthread_cond_wait(&m_cond, &m_mutex.m_mutex) == 0;
#endif
	m_mutex.resumeOwnership(parked);
	return rv;
}

/*! \brief Wait until we're signalled to continue, or until timeout [ms] has passed

	 \details Before calling this function, it is required that the mutex
	 provided during construction is _locked_. This function unlocks the mutex
	 internally, and returns with the mutex locked again.

	 \param timeout The timeout value in ms

	 \return true if we were signalled, false otherwise
*/
bool WaitCondition::wait(uint32_t timeout)
{
	Mutex::Ownership const parked = m_mutex.suspendOwnership();
#ifdef _WIN32
	bool const rv = SleepConditionVariableCS(&m_cond, &m_mutex.m_mutex, timeout) != 0;
#else
	static const int64_t NANOS_PER_MILLI = 1000000;
	static const int64_t NANOS_PER_ONE = NANOS_PER_MILLI * 1000;

	struct timespec time;
	int64_t nsec;

	clock_gettime(m_clockId, &time);

	nsec = time.tv_nsec + timeout * NANOS_PER_MILLI;
	time.tv_nsec = nsec % NANOS_PER_ONE;
	time.tv_sec += nsec / NANOS_PER_ONE;

	bool const rv = pthread_cond_timedwait(&m_cond, &m_mutex.m_mutex, &time) == 0;
#endif
	m_mutex.resumeOwnership(parked);
	return rv;
}

#if defined(_WIN32)
/*! \brief Constructor, initializes the event in the reset state */
WaitEvent::WaitEvent()
	: m_waiterCount(0)
	, m_terminating(false)
{
	m_event = ::CreateEventA(nullptr, TRUE, FALSE, nullptr);
}

/*! \brief Destructor, terminates the event and waits for waiting threads to be notified before finishing */
WaitEvent::~WaitEvent()
{
	try
	{
		terminate();
	}
	catch (...) // NOSONAR cpp:S2486 - destructor; an escaping exception would terminate the process
	{
		// Deliberately swallowed: a destructor must not let an exception escape.
	}
	assert(m_waiterCount == 0);
	::CloseHandle(m_event);
	m_event = INVALID_HANDLE_VALUE;
}

/*! \brief Wait for the event to be set or object termination
	\return true if the event is set and the object is not being terminated
*/
bool WaitEvent::wait()
{
	if (m_terminating)
		return false;

	++m_waiterCount;

	const auto result = ::WaitForSingleObject(m_event, INFINITE);
	--m_waiterCount;

	// Any outcome other than a normal signal (abandoned, timeout, failure) means the wait did not succeed
	return result == WAIT_OBJECT_0 && !m_terminating;
}

/*! \brief Set the event.
	\details Waiting items will (all) be notified and allowed to run
*/
void WaitEvent::set()
{
	::SetEvent(m_event);
}

/*! \brief Reset the event.
	\details Any wait() call will block until set() or terminate() is called.
	\note After terminate() has been called, reset() will not reset the event anymore
*/
void WaitEvent::reset()
{
	if (!m_terminating)
		::ResetEvent(m_event);
}
#else
WaitEvent::WaitEvent()
	: m_waiterCount(0)
	, m_terminating(false)
{
	pthread_mutex_init(&m_mutex, 0);
	pthread_cond_init(&m_cond, 0);
	m_triggered = false;
}

WaitEvent::~WaitEvent()
{
	terminate();
	pthread_cond_destroy(&m_cond);
	pthread_mutex_destroy(&m_mutex);
}

/*! \brief Wait for the event to be set or object termination
	\return true if the event is set and the object is not being terminated
*/
bool WaitEvent::wait()
{
	if (m_terminating)
		return false;

	++m_waiterCount;
	pthread_mutex_lock(&m_mutex);
	while (!m_triggered && !m_terminating)
		pthread_cond_wait(&m_cond, &m_mutex);
	pthread_mutex_unlock(&m_mutex);
	--m_waiterCount;
	return !m_terminating;
}

/*! \brief Set the event.
	\details Waiting items will (all) be notified and allowed to run
*/
void WaitEvent::set()
{
	pthread_mutex_lock(&m_mutex);
	m_triggered = true;
	pthread_cond_signal(&m_cond);
	pthread_mutex_unlock(&m_mutex);
}

/*! \brief Reset the event.
	\details Any wait() call will block until set() or terminate() is called.
	\note After terminate() has been called, reset() will not reset the event anymore
*/
void WaitEvent::reset()
{
	if (!m_terminating)
	{
		pthread_mutex_lock(&m_mutex);
		m_triggered = false;
		pthread_mutex_unlock(&m_mutex);
	}
}

#endif

/*! \brief Terminates the thread
*/
void WaitEvent::terminate()
{
	m_terminating = true;
	set();

	while (m_waiterCount > 0)
		XsTime_msleep(2);
}

} // namespace xsens
