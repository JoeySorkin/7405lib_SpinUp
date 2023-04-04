#include "Logger.h"
#include "fmt/format.h"
#include "pros/apix.h"
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <type_traits>

// ASCII Escape Codes - should be compatible with most Unix terminals, and
// windows terminal should support it if using normal console, on windows, the
// application that opens console window must enable virtual processing bs
// thingimajig
#define RED_TERM "\x1B[31m"
#define BR_RED "\x1B[91m"
#define GRN "\x1B[32m"
#define BR_GREEN "\x1B[92m"
#define YEL "\x1B[33m"
#define BR_YEL "\x1B[93m"
#define BLU "\x1B[34m"
#define BR_BLU "\x1B[94m"
#define MAG "\x1B[35m"
#define BR_MAG "\x1B[95m"
#define CYN "\x1B[36m"
#define BR_CYN "\x1B[96m"
#define WHT "\x1B[37m"
#define BR_WHT "\x1B[97m"
#define BOLD "\x1B[1m"
#define RESET "\x1B[0m"

namespace LoggerColor {
	const char* RED = "\x1B[41m";
	const char* GREEN = "\x1B[42m";
	const char* YELLOW = "\x1B[43m";
	const char* BLUE = "\x1B[44m";
	const char* MAGENTA = "\x1B[45m";
	const char* CYAN = "\x1B[46m";
	const char* WHITE = "\x1B[47m";
	const char* BRIGHT_RED = "\x1B[101m";
	const char* BRIGHT_GREEN = "\x1B[102m";
	const char* BRIGHT_YELLOW = "\x1B[103m";
	const char* BRIGHT_BLUE = "\x1B[104m";
	const char* BRIGHT_MAGENTA = "\x1B[105m";
	const char* BRIGHT_CYAN = "\x1B[106m";
	const char* BRIGHT_WHITE = "\x1B[107m";
	const char* NONE = "";
}// namespace LoggerColor


Logger* Logger::INSTANCE = nullptr;

Logger::Logger() : task(nullptr), logFile(nullptr), filename(), sources() {}

void Logger::backend() {
	while (true) {
		// TODO: Do we want to close/flush file everytime we get disabled?
		// but if we get moved into disabled, we will have to first finish logging
		// everything in queue before closing file might just have to occasionally
		// close and reopen the file

		// idk i just put in random value
		// but loop exists because we want to print more than just 1 message per
		// 10ms otherwise queues would overflow and loop breaks when there are no
		// more messages to process
		for (int i = 0; i < 10; i++) {
			// info regarding the lowest timestamped message queued to process

			pros::c::queue_t consoleQueue = nullptr;
			uint32_t consoleTimestamp = std::numeric_limits<uint32_t>::max();

			pros::c::queue_t fileQueue = nullptr;
			uint32_t fileTimestamp = std::numeric_limits<uint32_t>::max();

			// finds next message to print - one whose timestamp is smallest
			for (auto const& [key, val] : sources) {
				// part
				LogSource::Message msg{};

				if (pros::c::queue_get_waiting(val->mailboxConsole)) {
					pros::c::queue_peek(val->mailboxConsole, &msg, 0);

					if (msg.timestamp < consoleTimestamp) {
						consoleTimestamp = msg.timestamp;
						consoleQueue = val->mailboxConsole;
					}
				}

				if (logFile && pros::c::queue_get_waiting(val->mailboxFile)) {
					pros::c::queue_peek(val->mailboxFile, &msg, 0);

					if (msg.timestamp < fileTimestamp) {
						fileTimestamp = msg.timestamp;
						fileQueue = val->mailboxFile;
					}
				}
			}

			// flag to break out if there is no message to log to either stdout or
			// file
			bool messageExists = false;

			if (consoleQueue) {
				messageExists = true;
				LogSource::Message msg{};

				pros::c::queue_recv(consoleQueue, &msg, 0);

				fwrite(msg.msg, 1, msg.len, stdout);

				// free string buffer so we don't have memory leak
				delete msg.msg;
			}

			if (fileQueue) {
				messageExists = true;
				LogSource::Message msg{};

				pros::c::queue_recv(fileQueue, &msg, 0);

				fwrite(msg.msg, 1, msg.len, logFile);

				// free string buffer
				delete msg.msg;
			}

			if (!messageExists) { break; }
		}

		pros::delay(10);
	}
}

void Logger::initialize(std::string filename) {
	// probably should set the prioirty to be TASK_PRIORITY_DEFAULT - 1
	task = pros::c::task_create([](void* ign) { sLogger->backend(); }, nullptr, TASK_PRIORITY_DEFAULT,
	                            TASK_STACK_DEPTH_DEFAULT, "Logger");

	this->filename = fmt::format("/usd/{}", filename);
	logFile = fopen(filename.c_str(), "w");
}

void Logger::terminate() {
	if (task) {
		pros::c::task_delete(task);
		task = nullptr;
	}

	if (logFile) {
		fclose(logFile);
		logFile = nullptr;
	}
}

std::shared_ptr<LogSource> Logger::createSource(std::string name, uint32_t timeout, const char* color) {
	std::shared_ptr<LogSource> source = std::shared_ptr<LogSource>(new LogSource(name, timeout, color));

	sources.insert({name, source});

	return source;
}

void Logger::deleteSource(std::string name) {
	sources.erase(name);
}

LogSource::LogSource(std::string sourceName, uint32_t timeout, const char* color)
    : loggerColor(color), name(sourceName), timeout(timeout),
      logLevels(static_cast<LogLevel>(DEBUG | INFO | WARNING | ERROR)),
      outputSources(static_cast<Source>(CONSOLE | FILE)) {

	mailboxConsole = pros::c::queue_create(100, sizeof(Message));
	mailboxFile = pros::c::queue_create(100, sizeof(Message));
}

std::string_view LogSource::levelToString(LogLevel level) {
	static constexpr std::array<std::string_view, 5> logLevelsMap = {"DEBUG", "INFO", "WARNING", "ERROR", "UNKNOWN"};

	using LogLevelType = std::underlying_type<LogLevel>::type;

	int logLevel = std::sqrt(static_cast<LogLevelType>(level)) - 1;

	if (logLevel > logLevelsMap.size() - 1) {
		log(ERROR, pros::millis(),
		    "Logger {} encountered an error in LogSource::levelToString. logLevel "
		    "specified is not proper log level: "
		    "{}",
		    fmt::make_format_args(name, static_cast<LogLevelType>(level)));
		return logLevelsMap.back();
	}

	return logLevelsMap[logLevel];
}

const char* levelToColor(LogSource::LogLevel level) {
	switch (level) {
		case LogSource::DEBUG:
			return BOLD BR_BLU;
		case LogSource::INFO:
			return BOLD BR_GREEN;
		case LogSource::WARNING:
			return BOLD BR_YEL;
		case LogSource::ERROR:
			return BOLD BR_RED;
		default:
			return BOLD BR_MAG;
	}
}

void LogSource::log(LogLevel level, uint32_t timestamp, std::string_view fmt, fmt::format_args args) {
	std::string_view levelName = levelToString(level);
	const char* levelColor = levelToColor(level);

	if (outputSources & CONSOLE) {
		// AHHHHHHHH! WE FORMAT TO ANOTHER FORMAT
		std::string formatted = fmt::format("[{:.2F} {}{}{}{} {}{}{}] {}", timestamp / 1000.0, BOLD BR_WHT, loggerColor,
		                                    name, RESET, levelColor, levelName, RESET, fmt::vformat(fmt, args));

		// allocates # of chars + 1 for \0
		Message msg = {.timestamp = timestamp, .msg = new char[formatted.length() + 1], .len = formatted.length()};
		strcpy(msg.msg, formatted.data());

		pros::c::queue_append(mailboxConsole, &msg, timeout);
	}

	// don't even want to flood queue if a file isn't even open yet
	if (outputSources & FILE && sLogger->logFile) {
		// only difference is we get rid of color formatting
		std::string formatted =
		        fmt::format("[{:.2F} {} {}] {}", timestamp / 1000.0, name, levelName, fmt::vformat(fmt, args));

		// allocates # of chars + 1 for \0
		Message msg = {.timestamp = timestamp, .msg = new char[formatted.length() + 1], .len = formatted.length()};
		strcpy(msg.msg, formatted.data());

		pros::c::queue_append(mailboxFile, &msg, timeout);
	}
}

void LogSource::setLevel(LogLevel level) {
	logLevels = level;
}

void LogSource::setOutput(Source sources) {
	outputSources = sources;
}

void LogSource::setTimeout(uint32_t timeout) {
	this->timeout = timeout;
}