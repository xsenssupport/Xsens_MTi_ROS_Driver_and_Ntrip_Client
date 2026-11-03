
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

#include "journalthreader.h"
#include "journalfile.h"
#include "journaller.h"

#ifndef XSENS_WINDOWS
#include <atomic>
#include <cstring>
#include <unistd.h>

//! Descriptor the debug channel writes to; -1 means the default, stderr. \sa journalSetDebugChannelFd
static std::atomic<int> g_debugChannelFd{-1};

static void OutputDebugStringA(const char* msg)
{
	// write() rather than fprintf() when redirected: no FILE buffering to flush, which also
	// means a crash handler's last lines still make it out.
	if (int fd = g_debugChannelFd.load(); fd >= 0)
	{
		[[maybe_unused]] auto written = ::write(fd, msg, std::strlen(msg));
		return;
	}
	(void)fprintf(stderr, "%s", msg);
}
#endif

void journalSetDebugChannelFd(int fd)
{
#ifndef XSENS_WINDOWS
	g_debugChannelFd.store(fd);
#else
	(void)fd;
#endif
}

#ifndef XSENS_WINDOWS
/*! \brief Weak default for xsens_unittest's "stdout/stderr are now captured" hook.

	Every Journaller-backed test library needs the same thing here, and a library that forgets
	it hangs its Linux and AArch64 test jobs (#4117) - so supply it centrally instead of
	waiting for each one to be visited.

	Why this is reliable rather than a coin toss on link order. The definition lives in THIS
	translation unit on purpose: any link that constructs a Journaller pulls journalthreader
	in, because JournalThreader is a Journaller member - so what decides the outcome is
	inclusion, not archive order. xsens_unittest's own default sits in an archive member that
	the linker pulls only to resolve an otherwise-undefined symbol, and this definition has
	already resolved it, so that member is never pulled. Weak, so a test library that wants
	different behaviour still wins with a plain definition of its own (mvn's xme_test has one,
	doing exactly this).

	POSIX only, for two reasons: the Windows debug channel is OutputDebugString, so there is
	nothing to move, and MSVC has no weak functions - two strong definitions would be a
	duplicate-symbol error rather than an override.

	A test library that links xscommon but never constructs a Journaller may not pull this TU
	at all; xsens_unittest's no-op default then applies, which is correct - with no Journaller
	there is no loop to break.
*/
__attribute__((weak)) void xsensUnitTestLogRedirectReady(int consoleStdoutFd, int consoleStderrFd)
{
	(void)consoleStdoutFd;
	journalUseConsoleStderrFd(consoleStderrFd);
}
#endif

void journalUseConsoleStderrFd(int consoleStderrFd)
{
#ifdef XSENS_WINDOWS
	(void)consoleStderrFd;
#else
	if (consoleStderrFd < 0)
		return;
	// dup() so the channel outlives the descriptor the test runner owns: it closes that one
	// during teardown while other threads may still be logging. One descriptor for the rest
	// of the process; the OS reclaims it at exit.
	if (int channelFd = ::dup(consoleStderrFd); channelFd >= 0)
		journalSetDebugChannelFd(channelFd);
#endif
}

/*! \class JournalThreader
	\brief Manages threaded writes for the Journaller objects
*/

/*! \brief Constructor, sets up the necessary structures for threaded logging */
JournalThreader::JournalThreader() = default;

/*! \brief Destructor, flushes all remaining data
*/
JournalThreader::~JournalThreader() = default;

/*! \brief Flush all available non-empty lines to the supplied file, clearing lines as they are flushed
	\param file The file to write to. Supply nullptr to write to the debug output
*/
void JournalThreader::flushAll(JournalFile* file)
{
	while (true)
	{
		ThreadLine& line = nextLine();
		if (line.m_line.empty())
			return;

		if (file)
			*file << line.m_line;
		else
			OutputDebugStringA(line.m_line.c_str());
		line.m_line.clear();
	}
}

/*! \brief Write the line for thread \a thread to file \a file
	\param thread The thread id associated with the line
	\param file The file to write to. Supply nullptr to write to the debug output
*/
void JournalThreader::writeLine(int thread, JournalFile* file)
{
	const ThreadLine& line = threadLine(thread);
	if (line.m_line.empty())
		return;

	if (file)
		*file << line.m_line;
	else
		OutputDebugStringA(line.m_line.c_str());
}

/*! \brief Set the log level of the queued line
	\param thread The thread id associated with the line
	\param level The log level to set
	\returns The old log level
*/
JournalLogLevel JournalThreader::setLineLevel(int thread, JournalLogLevel level)
{
	ThreadLine& line = threadLine(thread);
	auto old = line.m_level;
	line.m_level = level;
	return old;
}

/*! \returns The log level of the queued line
	\param thread The thread id associated with the line
*/
JournalLogLevel JournalThreader::lineLevel(int thread)
{
	const ThreadLine& line = threadLine(thread);
	return line.m_level;
}

/*! \returns The reference to the line associated with the given file and thread
	\param thread The thread id associated with the line, ignored when JOURNALLER_WITH_THREAD_SUPPORT == 0
*/
std::string& JournalThreader::line(int thread)
{
	ThreadLine& line = threadLine(thread);
	return line.m_line;
}

/*! \brief Remove the thread-local storage for thread \a id
	\param id The thread to remove
*/
void JournalThreader::cleanup(int id)
{
	xsens::Lock lock(&m_mutex);
	m_map.erase(id);
	lock.unlock();
}
