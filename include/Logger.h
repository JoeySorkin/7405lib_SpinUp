#include "main.h"
#include "pros/apix.h"
#define sLogger Logger::getInstance()
class Logger
{
private:
    static Logger *INSTANCE;
    pros::task_t task;

    void runner(void *ignored);
    pros::c::queue_t queue;

    FILE* logFile;
public:
    struct Message
    {
        std::string author = "NULL";
        uint32_t time = 0;
        std::string message = "NULL";
    };
    /**
     * Can be called from any thread, used to append messages to the logfile on the sd card
     * @param author
     * @param message
     */
    void log(std::string author, std::string message);
    static Logger *getInstance()
    {
        if (!INSTANCE)
        {
            INSTANCE = new Logger();
        }
        return INSTANCE;
    }
    void initialize();
    void close();
};
