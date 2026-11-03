
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

#ifndef XSENS_THREADPOOL_H
#define XSENS_THREADPOOL_H

#include "xsens_mutex.h"

#include <atomic>
#include <map>
#include <set>
#include <deque>
#include <exception>
#include <future>
#include <list>
#include <memory>
#include <tuple>
#include <type_traits>
#include <utility>
#include <version>
// Test the library feature macros, not the language version: NDK r27c's libc++ 18 (Android) is C++20 but only has
// std::stop_token behind -fexperimental-library, while it does ship <coroutine>
#ifdef __cpp_lib_coroutine
	#include <coroutine>
#endif
#ifdef __cpp_lib_jthread
	#include <stop_token>
#endif

namespace xsens
{

int processorCount();

class PooledTask;
class ThreadPoolTask
{
public:
	virtual bool exec() = 0;				//!< \returns True if the task completed or false to reschedule the task
	virtual unsigned int needToWaitFor();
	ThreadPoolTask() : m_container(nullptr) {}
	virtual ~ThreadPoolTask() = default;
	virtual void onError() { /* Intentional no-op: an optional hook for subclasses. */ }				//!< Callback for when an error occurred during task execution. Any implementation of this function should NEVER throw an exception.
	virtual void onCancel() noexcept { /* Intentional no-op: an optional hook for subclasses. */ }	//!< Callback from ThreadPool::cancelTask(), on the cancelling thread, before any wait. Called without the pool lock held.
	bool isCanceling() const;
	unsigned int taskId() const;

private:
	friend class ThreadPool;
	PooledTask* m_container;
};

class PooledThread;
class ThreadPool
{
public:
	using TaskId = unsigned int;			//!< A type definition of a task ID

private:
	void reportTaskComplete(std::shared_ptr<PooledTask>);
	void reportTaskPaused(std::shared_ptr<PooledTask>);
	std::shared_ptr<PooledTask> getNextTask();
	friend class PooledThread;

	std::set<PooledThread*> m_threads;
	std::deque<std::shared_ptr<PooledTask>> m_tasks;
	std::map<TaskId, std::shared_ptr<PooledTask>> m_tasksSearch;
	std::map<TaskId, std::shared_ptr<PooledTask>> m_executing;
	std::map<TaskId, std::shared_ptr<PooledTask>> m_delaying;
	mutable Mutex m_safe;
	TaskId m_nextId;
	std::atomic_bool m_suspended;
	std::atomic_bool m_terminating;

	std::shared_ptr<PooledTask> findTask(TaskId id);

protected:
	ThreadPool();
	~ThreadPool();

public:
	TaskId addTask(ThreadPoolTask* task, TaskId afterId = 0);
	unsigned int count();
	void setPoolSize(unsigned int poolsize);
	unsigned int poolSize() const;
	bool doesTaskExist(TaskId id);
	void cancelTask(TaskId id, bool wait = true) noexcept;
	void waitForCompletion(TaskId id);
	void suspend(bool wait = false) noexcept;
	void resume();
	unsigned int executedCount(unsigned int thread) const;
	unsigned int completedCount(unsigned int thread) const;
	unsigned int failedCount(unsigned int thread) const;
	XsThreadId taskThreadId(TaskId id);

	template <typename F, typename... Args>
	auto submit(F&& func, Args&&... args);

	static ThreadPool* instance() noexcept;
	static void destroy();
	static void setPool(ThreadPool* pool);
};

class TaskCompletionWaiter : public ThreadPoolTask
{
public:
	bool exec() override;
	unsigned int needToWaitFor() override;

	TaskCompletionWaiter(ThreadPool* pool = ThreadPool::instance());
	TaskCompletionWaiter(const std::list<unsigned int>& waitlist, ThreadPool* pool = ThreadPool::instance());
	virtual ~TaskCompletionWaiter();
	void addWaitId(unsigned int id);

private:
	ThreadPool* m_pool;
	std::list<unsigned int> m_waitList;
};

/*! \brief Task that will delete its object parameter
	\details This class can be used to schedule a delete of a single object in the ThreadPool.
	Normally you don't need to use this class directly, but can use the \ref deleteThreaded function.
	\tparam T The type of the object to delete
*/
template <typename T>
class ThreadPoolObjectDeleter : public ThreadPoolTask
{
	T* m_object;
	ThreadPoolObjectDeleter(ThreadPoolObjectDeleter const&) = delete;
	ThreadPoolObjectDeleter& operator = (ThreadPoolObjectDeleter const&) = delete;
public:
	/*! \brief Constructor, creates the task to delete object a obj, but doesn't schedule it.
	*/
	ThreadPoolObjectDeleter(T* obj) : m_object(obj) {}
	/*! \brief deletes the managed object */
	bool exec() override
	{
		delete m_object;
		return true;
	}
};

/*! \brief Delete \a obj in the ThreadPool.
	\details This function will schedule a task to be executed by the ThreadPool which will delete obj
	\param obj The object to be deleted
	\tparam T The type of the object to delete. Typically not required as the compiler should detect it automatically.
	\return The task ID of the deletion task
	\relates ThreadPoolObjectDeleter
*/
template <typename T>
inline static ThreadPool::TaskId deleteThreaded(T* obj)
{
	return ThreadPool::instance()->addTask(new ThreadPoolObjectDeleter<T>(obj));
}

/////////////////////////////////////////////////////////////////////////////////////////
// std:: interoperability: ThreadPool::submit(), and with C++20 std::stop_token cancellation and co_await resumeOn()
/////////////////////////////////////////////////////////////////////////////////////////

/*! \brief The result of ThreadPool::submit()
	\tparam R The return type of the submitted callable
*/
template <typename R>
struct SubmittedTask
{
	ThreadPool::TaskId id = 0;	//!< The id of the scheduled task, for ThreadPool::cancelTask() and ThreadPool::waitForCompletion()
	std::future<R> future;		//!< Receives the callable's result or exception. Holds std::future_errc::broken_promise if the task was cancelled or discarded before it started.
};

namespace threadpool_detail
{
//! Run \a call and store its result or exception in \a promise
template <typename R, typename Call>
void fulfil(std::promise<R>& promise, Call&& call)
{
	try
	{
		if constexpr (std::is_void_v<R>)
		{
			std::forward<Call>(call)();
			promise.set_value();
		}
		else
			promise.set_value(std::forward<Call>(call)());
	}
	catch (...)
	{
		promise.set_exception(std::current_exception());
	}
}

//! A ThreadPoolTask that runs a callable once and publishes its outcome through a std::future
template <typename R, typename Func, typename ArgTuple>
class CallableTask final : public ThreadPoolTask
{
public:
	//! Take over \a func and the \a args it will be called with
	CallableTask(Func&& func, ArgTuple&& args) : m_func(std::move(func)), m_args(std::move(args)) {}
	//! The future that receives the outcome of exec(). Can be retrieved only once
	std::future<R> future() { return m_promise.get_future(); }
	bool exec() override
	{
		fulfil(m_promise, [this]() -> R { return std::apply(std::move(m_func), std::move(m_args)); });
		return true;
	}

private:
	Func m_func;
	ArgTuple m_args;
	std::promise<R> m_promise;
};

//! Schedule \a task in \a pool, which takes ownership of it
template <typename R, typename Task>
SubmittedTask<R> schedule(ThreadPool& pool, Task* task)
{
	SubmittedTask<R> submitted;
	submitted.future = task->future();
	submitted.id = pool.addTask(task);
	return submitted;
}

template <typename Func, typename ArgTuple>
auto submitCallable(ThreadPool& pool, Func func, ArgTuple args)
{
	using R = decltype(std::apply(std::move(func), std::move(args)));
	return schedule<R>(pool, new CallableTask<R, Func, ArgTuple>(std::move(func), std::move(args)));
}
} // namespace threadpool_detail

#ifdef __cpp_lib_jthread

namespace threadpool_detail
{
//! A CallableTask whose callable takes a std::stop_token first; ThreadPool::cancelTask() requests the stop
template <typename R, typename Func, typename ArgTuple>
class StoppableCallableTask final : public ThreadPoolTask
{
public:
	StoppableCallableTask(Func&& func, ArgTuple&& args) : m_func(std::move(func)), m_args(std::move(args)) {}
	std::future<R> future() { return m_promise.get_future(); }
	bool exec() override
	{
		fulfil(m_promise, [this]() -> R
			{
				return std::apply(std::move(m_func), std::tuple_cat(std::make_tuple(m_stop.get_token()), std::move(m_args)));
			});
		return true;
	}
	void onCancel() noexcept override
	{
		m_stop.request_stop();
	}

private:
	Func m_func;
	ArgTuple m_args;
	std::stop_source m_stop;
	std::promise<R> m_promise;
};
} // namespace threadpool_detail

/*! \brief Run \a func(\a args...) once in the pool and return its task id and a std::future for its result
	\details \a func and \a args are decay-copied, as by std::thread. Exceptions thrown by \a func are delivered
	through the future instead of ThreadPoolTask::onError().
	If \a func is invocable as func(std::stop_token, args...) it receives a token that is stopped by
	cancelTask() on the returned id, so a long-running callable can end early.
	\note Don't block on the future from inside another pool task: with every pool thread waiting, the awaited task never starts.
	Use co_await resumeOn() or a TaskCompletionWaiter for that instead.
*/
template <typename F, typename... Args>
auto ThreadPool::submit(F&& func, Args&&... args)
{
	using Func = std::decay_t<F>;
	using ArgTuple = std::tuple<std::decay_t<Args>...>;
	if constexpr (std::is_invocable_v<Func, std::stop_token, std::decay_t<Args>...>)
	{
		using R = std::invoke_result_t<Func, std::stop_token, std::decay_t<Args>...>;
		return threadpool_detail::schedule<R>(*this, new threadpool_detail::StoppableCallableTask<R, Func, ArgTuple>(Func(std::forward<F>(func)), ArgTuple(std::forward<Args>(args)...)));
	}
	else
		return threadpool_detail::submitCallable(*this, Func(std::forward<F>(func)), ArgTuple(std::forward<Args>(args)...));
}

#else

/*! \brief Run \a func(\a args...) once in the pool and return its task id and a std::future for its result
	\details \a func and \a args are decay-copied, as by std::thread. Exceptions thrown by \a func are delivered
	through the future instead of ThreadPoolTask::onError().
	\note Don't block on the future from inside another pool task: with every pool thread waiting, the awaited task never starts.
*/
template <typename F, typename... Args>
auto ThreadPool::submit(F&& func, Args&&... args)
{
	using Func = std::decay_t<F>;
	using ArgTuple = std::tuple<std::decay_t<Args>...>;
	return threadpool_detail::submitCallable(*this, Func(std::forward<F>(func)), ArgTuple(std::forward<Args>(args)...));
}

#endif

#ifdef __cpp_lib_coroutine

/*! \brief The awaitable returned by resumeOn() */
class ResumeOnPool
{
public:
	explicit ResumeOnPool(ThreadPool& pool) noexcept : m_pool(pool) {}
	bool await_ready() const noexcept { return false; }
	void await_suspend(std::coroutine_handle<> coroutine) { m_pool.addTask(new ResumeTask(coroutine)); }
	void await_resume() const noexcept { /* Intentional no-op: resuming delivers no value. */ }

private:
	class ResumeTask final : public ThreadPoolTask
	{
	public:
		explicit ResumeTask(std::coroutine_handle<> coroutine) noexcept : m_coroutine(coroutine) {}
		bool exec() override
		{
			m_coroutine.resume();
			return true;
		}

	private:
		std::coroutine_handle<> m_coroutine;
	};

	ThreadPool& m_pool;
};

/*! \brief co_await resumeOn(pool) suspends the calling coroutine and continues it on a thread of \a pool
	\note If the resumption task is cancelled or the pool is destroyed before it runs, the coroutine is never resumed.
*/
inline ResumeOnPool resumeOn(ThreadPool& pool = *ThreadPool::instance()) noexcept
{
	return ResumeOnPool(pool);
}

#endif

} // namespace xsens

#endif
