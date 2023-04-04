#pragma once
#include "fmt/core.h"
#include "fmt/format.h"
#include "pros/apix.h"
#include <memory>
#include <string_view>
#include <unordered_map>

#define sLogger Logger::getInstance()


class LogSource;
using LoggerPtr = std::shared_ptr<LogSource>;

// To add extra 🌶️ to the logger
// Naming convention is a little off but whatever
namespace LoggerColor {
	extern const char* RED;
	extern const char* GREEN;
	extern const char* YELLOW;
	extern const char* BLUE;
	extern const char* MAGENTA;
	extern const char* CYAN;
	extern const char* WHITE;
	extern const char* BRIGHT_RED;
	extern const char* BRIGHT_GREEN;
	extern const char* BRIGHT_YELLOW;
	extern const char* BRIGHT_BLUE;
	extern const char* BRIGHT_MAGENTA;
	extern const char* BRIGHT_CYAN;
	extern const char* BRIGHT_WHITE;
	extern const char* NONE;
}// namespace LoggerColor


// Overarching logging manager
// Sink for the logging sources
class Logger {
	friend class LogSource;

private:
	static Logger* INSTANCE;
	pros::task_t task;
	FILE* logFile;
	std::string filename;
	std::unordered_map<std::string, std::shared_ptr<LogSource>> sources;

	Logger();

	// thread that runs in backend, logging to file/handler
	void backend();

public:
	static Logger* getInstance() {
		if (!INSTANCE) { INSTANCE = new Logger(); }

		return INSTANCE;
	}

	/**
	 * @brief Initializes the logger.
	 * Starts the internal worker thread
	 * and opens the file.
	 *
	 * For now: Everytime logger gets initialized, contents of the specified file
	 * get wiped. And the different modes that we are logging in are demarked by
	 * either AUTON or OPCONTROl within the file. Maybe split it into two files in
	 * the future
	 *
	 * @param filename - Name of file. Don't need to include "/usd/"
	 */
	void initialize(std::string filename);

	/**
	 * @brief Just closes the output file of the logger.
	 * Depending on what we decide to do with handling closing the file, when the
	 * robot switches to opctrl or auton, the file may be reopened again.
	 *
	 */
	void close();

	/**
	 * @brief Kills the logger from running and closes any opened file.
	 * Must call initialize to restart logger.
	 */
	void terminate();

	// looks little iffy, but we do create copy in ctor of LogSource
	std::shared_ptr<LogSource> createSource(std::string name, uint32_t timeout = 0,
	                                        const char* color = LoggerColor::NONE);

	/**
	 * @brief Get the Source object
	 *
	 * @param name
	 * @return std::shared_ptr<LogSource>
	 */
	std::shared_ptr<LogSource> getSource(std::string name);

	/**
	 * @brief Removes a LogSource from the list of LogSources that the Logger
	 * logs. Thereby stopping any output from this source from being logged at
	 * all.
	 *
	 * @param name - Name of the LogSource to delete.
	 */
	void deleteSource(std::string name);
};

class LogSource {
	friend class Logger;

public:
	enum LogLevel : uint8_t { DEBUG = 1, INFO = 4, WARNING = 8, ERROR = 16 };
	enum Source : uint8_t { CONSOLE = 1, FILE = 2 };

private:
	struct Message {
		// in ms - might have to do μs to sort out contentions on synchronization of
		// messages from multiple sinks
		uint32_t timestamp;

		// this must be a pointer due to  how the RTOS's queue implementation is
		// done because it's copied, the RTOS does a memcpy. memcpying the contents
		// of a string would result in interesting behavior especially if it is no
		// longer doing a short string optimization because you'd just be copying a
		// ptr which will be dangling when the dtor of the string gets called God i
		// love the risk of memory leaks
		char* msg;
		uint32_t len;// num of chars - excludes the \0
	};

	pros::c::queue_t mailboxConsole;
	pros::c::queue_t mailboxFile;

	const char* loggerColor;
	std::string name;

	uint32_t timeout;

	// Which log levels are enabled and which ones we ignore
	LogLevel logLevels;

	// To which sinks are we outputting
	Source outputSources;

	/**
	 * @brief Construct a new Log Source object
	 * By default enables all log levels, and logs to all sources.
	 *
	 * We only want logger to be able to create these log sources as it manages
	 * these objects and we don't want people to willy nilly do these things.
	 */
	LogSource(std::string sourceName, uint32_t timeout = 0, const char* color = LoggerColor::NONE);

	std::string_view levelToString(LogLevel level);

	void log(LogLevel level, uint32_t timestamp, std::string_view fmt, fmt::format_args args);

public:
	/**
	 * @brief Sets which levels of logging will actually be logged and which will
	 * be ignroed.
	 *
	 * @param level - Set level via bit operations. ex: DEBUG | INFO | WARNING
	 */
	void setLevel(LogLevel level);

	/**
	 * @brief Sets which levels of logging will actually be logged and which will
	 * be ignroed.
	 *
	 * @param sources - Which output sources this log source will log to.
	 */
	void setOutput(Source sources);

	void setTimeout(uint32_t timeout);

	// Can be treated sortof like a printf
	// but formatting must follow fmtlib's specs
	// https://fmt.dev/9.1.0/syntax.html

	template<class... Args>
	void debug(std::string fmt, Args&&... args) {
		if (!(logLevels & DEBUG)) { return; }
		log(DEBUG, pros::millis(), fmt, fmt::make_format_args(args...));
	}

	template<class... Args>
	void info(std::string fmt, Args&&... args) {
		if (!(logLevels & INFO)) { return; }
		log(INFO, pros::millis(), fmt, fmt::make_format_args(args...));
	}

	template<class... Args>
	void warning(std::string fmt, Args&&... args) {
		if (!(logLevels & WARNING)) { return; }
		log(WARNING, pros::millis(), fmt, fmt::make_format_args(args...));
	}

	// DONT USE!
	// causes program to hang entirely for some reason
	// sometimes causes prefetch error and other times program just crashes somewhere
	template<class... Args>
	void error(std::string fmt, Args&&... args) {
		if (!(logLevels & ERROR)) { return; }
		log(ERROR, pros::millis(), fmt, fmt::make_format_args(args...));
		// warning(fmt, std::forward<Args>(args)...);
	}
};