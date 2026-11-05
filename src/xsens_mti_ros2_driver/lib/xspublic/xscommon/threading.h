
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

#ifndef THREADING_H
#define THREADING_H

#include "xsens_mutex.h"
#include <functional>
#include <utility>

#ifndef __GNUC__
	#pragma warning(push)
	#pragma warning(disable: 4127)
#endif

namespace xsens
{
/*!	\class StandardThread
	\brief A class for a standard thread that has to perform the same action repeatedly.
	\details The class has three virtual functions, of which the innerFunction is the most important.
	innerFunction gets called repeatedly and is expected to return so that StandardThread can
	check for thread termination.
*/
class StandardThread
{
protected:
	std::atomic_bool m_stop;                //!< Indicates that the thread should stop. Derived classes should check isTerminating() instead of directly polling this value when checking if the thread should stop. However, there are some cases (tests, SignallingThread) where direct access from within the class is desired, which is why the value is protected instead of private.
	std::atomic_bool m_yieldOnZeroSleep;    //!< When true, a sleep value of 0 returned by innerFunction will trigger a thread yield operation. When false, the next cycle is started immediately.
#ifdef _WIN32
	HANDLE m_stopHandle;    //!< Duplicates m_stop functionality for external dependent classes such as Semaphore
#else
	pthread_attr_t m_attr;
#endif
private:
	mutable xsens::Mutex m_mux;
	XsThread m_thread;
	std::atomic<XsThreadId> m_threadId;
	std::atomic<XsThreadPriority> m_priority;
	std::atomic_bool m_finished;            //!< Indicates that the thread main function has finished
	std::atomic_bool m_killRequested;       //!< Set by terminateThread, honored by whoever holds mutex during a join
	char* m_name;
	static XSENS_THREAD_RETURN threadInit(void* obj);
	void threadMain(void);
	void stopThreadLocked() noexcept;
	bool setPriorityLocked(XsThreadPriority pri);
protected:
	//! Virtual initialization function
	virtual void initFunction(void) { /* Intentional no-op: an optional hook for subclasses. */ }

	//! Virtual exit function
	virtual void exitFunction(void) { /* Intentional no-op: an optional hook for subclasses. */ }

	//! Virtual inner function
	virtual int32_t innerFunction(void)
	{
		return 0;
	}

	//! Return the thread handle
	inline XsThread threadHandle() const
	{
		return m_thread;
	}
public:
	StandardThread();
	virtual ~StandardThread();

	bool startThread(const char* name = nullptr);
	virtual void signalStopThread(void);
	void stopThread(void) noexcept;
	bool isAlive(void) volatile const noexcept;
	bool isRunning(void) volatile const noexcept;
	bool setPriority(XsThreadPriority pri);
	bool isTerminating() volatile const noexcept;
	void terminateThread();
	bool isCurrentThread() const;

	//! \returns The thread ID
	inline XsThreadId getThreadId(void) const
	{
		return m_threadId;
	}
};
#define XSENS_THREAD_CHECK()	if (isTerminating()) return 0

#if defined(ANDROID) || defined(__aarch64__)
	#define CDECL_XS
#else
	#define CDECL_XS	__cdecl
#endif

#ifndef SWIG
typedef void (CDECL_XS* WatchDogFunction)(void*);

/*! \class WatchDogThread
	\brief A class that keeps an eye on a threads timer
*/
class WatchDogThread
{
private:
	XsThread m_thread;
#ifdef _WIN32
	HANDLE m_stop;
	HANDLE m_running;
	HANDLE m_reset;
#else
	pthread_attr_t m_attr;
	Mutex m_mutex;
	bool m_running;
	bool m_reset;
	bool m_stop;
	bool m_finished;
#endif
	volatile std::atomic<std::uint32_t> m_timeout;
	WatchDogFunction m_func;
	void* m_param;
	char* m_name;
	static XSENS_THREAD_RETURN threadInit(void* obj);

	XsThreadId m_threadId;
	void threadMain(void);
	bool isAlive(void) volatile const noexcept;
	bool isRunning(void) volatile const noexcept;
public:

	/*! \brief Constructor
	*/
	WatchDogThread(WatchDogFunction func, void* param = nullptr);

	/*! \brief Destructor
	*/
	~WatchDogThread();

	bool resetTimer(uint32_t timeout = 0);
	bool startTimer(uint32_t timeout = 10000, const char* name = nullptr);
	bool stopTimer(void) noexcept;

	//! \returns The thread ID
	XsThreadId getThreadId(void) const
	{
		return m_threadId;
	}
};

/*!	\class TaskThread
	\brief Class for handling small tasks
	\details Use this class if you have small tasks that need to be performed out of the main thread
	The thread uses tasks supplied in a TaskType struct.
*/
class TaskThread : public StandardThread
{
public:
	typedef void (CDECL_XS* TaskFunction)(void*); //!< A function prototype for a task
private:
	struct TaskType
	{
		TaskFunction m_function;
		void*	m_param;
	};

	std::deque<TaskType> m_queue;
	Mutex m_safe;
	bool m_inFunc;
protected:

	/*! \brief The inner function of the task thread.
		\details The function checks if there is a task in the queue and executes it.
		Then it returns to the StandardThread main loop to check for termination.
		If there are no tasks in the queue, the thread will terminate itself.
		\returns Not 0 if successful
	*/
	int32_t innerFunction(void) override
	{
		Lock safety(&m_safe);
		if (!m_queue.empty())
		{
			TaskType& task = m_queue.front();
			m_queue.pop_front();
			m_inFunc = true;
			safety.unlock();
			task.m_function(task.m_param);
			m_inFunc = false;
			return 0;	// no sleep
		}
		// notify thread to terminate when there are no more tasks
		signalStopThread();
		return 0;
	}
public:
	TaskThread() : m_inFunc(false) {}
	~TaskThread()
	{
		while (isRunning() && getLength())
			XsTime::msleep(100);
	}

	/*! \brief Adds a task to a queue
		\param func The function to add
		\param param The parameters to add
	*/
	void addTask(TaskFunction func, void* param)
	{
		Lock safety(&m_safe);
		TaskType tmp = {func, param};
		m_queue.push_back(tmp);
	}

	//! \returns The length of a queue
	int32_t getLength(void) noexcept
	{
		Lock safety(&m_safe);
		return (int32_t) m_queue.size() + (m_inFunc ? 1 : 0);
	}

	//! \brief Clears a queue
	void clear(void)
	{
		Lock safety(&m_safe);
		m_queue.clear();
	}
};
#endif
#ifndef __APPLE__
// the std::function is not properly available in all iOS versions
/*!	\class ThreadedFunction
	\brief A class that will run a single (lambda) function in a separate thread.
*/
class ThreadedFunction : public StandardThread
{
	std::function<void()> m_func;
public:
	/*! \brief Construct the object
		\param f The desired (lambda) function to run
		\param runNow Start the thread immediately (default = true)
	*/
	ThreadedFunction(std::function<void()> const& f, bool runNow = true)
		: m_func(f)
	{
		if (runNow)
			startThread(); // NOSONAR cpp:S1699 - startThread is non-virtual and innerFunction is final, so no subclass override can be missed here
	}
	~ThreadedFunction()
	{
		stopThread(); // NOSONAR cpp:S1699 - stopThread is non-virtual; the virtual signalStopThread it reaches must resolve to this class during destruction
	}

	/*! \brief The internal thread function, runs the supplied function */
	int32_t innerFunction(void) final
	{
		m_func();
		signalStopThread();
		return 0;
	}
};

/*!	\class StandardFunctionThread
	\brief A StandardThread that repeatedly runs a supplied (lambda) function
	\details The function plays the role of StandardThread::innerFunction: its return value is the time in ms
	to sleep before the next call. The thread runs until stopThread() is called or the object is destroyed.
*/
class StandardFunctionThread : public StandardThread
{
	std::function<int32_t()> m_func;
public:
	/*! \brief Construct the object
		\param f The function to call repeatedly, returning the number of ms to sleep before the next call
		\param runNow Start the thread immediately (default = true)
	*/
	explicit StandardFunctionThread(std::function<int32_t()> f, bool runNow = true)
		: m_func(std::move(f))
	{
		if (runNow)
			startThread(); // NOSONAR cpp:S1699 - startThread is non-virtual and innerFunction is final, so no subclass override can be missed here
	}
	~StandardFunctionThread()
	{
		stopThread(); // NOSONAR cpp:S1699 - stopThread is non-virtual; the virtual signalStopThread it reaches must resolve to this class during destruction
	}

	/*! \brief The internal thread function, runs the supplied function */
	int32_t innerFunction(void) final
	{
		return m_func();
	}
};
#endif
} // namespace xsens

#ifndef __GNUC__
	#pragma warning(pop)
#endif

#endif
