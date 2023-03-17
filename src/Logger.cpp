#include "Logger.h"

Logger *Logger::INSTANCE = nullptr;

void Logger::initialize()
{
    task = pros::c::task_create([](void *_)
                                { sLogger->runner(_); },
                                nullptr,
                                TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT,
                                "Logger");
    queue = pros::c::queue_create(100, sizeof(Message));
    logFile = fopen("/usd/robot_logs.txt", "w");
}

void Logger::runner(void *ignored)
{
    while (true)
    {
        if (pros::c::queue_get_waiting(queue))
        {
            Message req;
            pros::c::queue_recv(queue, &req, 0);

            fprintf(logFile, "%s: %s \t %s\n", std::to_string(req.time).c_str(), req.author.c_str(), req.message.c_str());
        }
        pros::delay(20);
    }
}


void Logger::log(std::string author, std::string message)
{
    Message msg = {.author = author, .time = pros::millis(), .message = message};
    pros::c::queue_append(queue, &msg, 10);
}
void Logger::close() {
  this->log("Logger", "Logger closed!");
  fclose(logFile);
  pros::delay(20);
  pros::c::task_delete(task);
}
