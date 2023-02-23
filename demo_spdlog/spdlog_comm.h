#pragma once

#include <queue>
#include <string>
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

// #define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_DEBUG

#define LOGGER_NAME "robot_logger"
#define LOGGER_FILE "logs/puncture_robot_log.txt"
#define MAX_LOG_SIZE    1024*1024*5
#define MAX_LOG_FILES   20

#define __FILENAME__ (strrchr(__FILE__, '/') ? (strrchr(__FILE__, '/') + 1):__FILE__)

#define SPD_TRACE(...) SPDLOG_LOGGER_TRACE(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_TRACE(spdlog::get(LOGGER_NAME), __VA_ARGS__)
#define SPD_DEBUG(...) SPDLOG_LOGGER_DEBUG(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_DEBUG(spdlog::get(LOGGER_NAME), __VA_ARGS__)
#define SPD_INFO(...) SPDLOG_LOGGER_INFO(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_INFO(spdlog::get(LOGGER_NAME), __VA_ARGS__)
#define SPD_WARN(...) SPDLOG_LOGGER_WARN(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_WARN(spdlog::get(LOGGER_NAME), __VA_ARGS__)
#define SPD_ERROR(...) SPDLOG_LOGGER_ERROR(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_ERROR(spdlog::get(LOGGER_NAME), __VA_ARGS__)
#define SPD_CRITICAL(...) SPDLOG_LOGGER_CRITICAL(spdlog::default_logger_raw(), __VA_ARGS__);SPDLOG_LOGGER_CRITICAL(spdlog::get(LOGGER_NAME), __VA_ARGS__)


class RTLog
{
private:
    std::shared_ptr<spdlog::logger> _spdlog;
    std::queue<spdlog::level::level_enum> _log_level;
    std::queue<std::string> _log_msg;
    /* data */
public:
    RTLog(/* args */);
    ~RTLog();
    void warn(const std::string& filename,const std::string& func,
        const int& line,const std::string& msg);

    void log();


};

RTLog::RTLog(/* args */)
{
    try
    {
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_st>();
        console_sink->set_level(spdlog::level::debug);

        auto rotate_file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_st>(LOGGER_FILE,MAX_LOG_SIZE, MAX_LOG_FILES);
        rotate_file_sink->set_level(spdlog::level::info);
        // rotate_file_sink->flush_on(spdlog::level::warn);

        spdlog::sinks_init_list sink_list = {console_sink,rotate_file_sink};
        _spdlog = std::make_shared<spdlog::logger>(LOGGER_NAME, sink_list.begin(),sink_list.end());
        _spdlog->set_level(spdlog::level::info);
        _spdlog->flush_on(spdlog::level::warn);
        // _spdlog->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%s,%!,%#]: %v");

    }
    catch (const spdlog::spdlog_ex& ex)
    {
        std::cout<<"Log initialization failed: "<<ex.what()<<std::endl;
    }    
}

void RTLog::warn(const std::string& filename,const std::string& func,
    const int& line,const std::string& msg)
{
    std::string flname = filename;
    std::string fcname = func;
    std::string linname = std::to_string(line);
    std::string prefix = "["+flname+","+fcname+"]<"+linname+">";
    _log_level.push(spdlog::level::warn);
    _log_msg.push(prefix+msg);
}

void RTLog::log()
{
    if(!_log_level.empty())
    {
        _spdlog->warn(_log_msg.front());
        _log_level.pop();
        _log_msg.pop();
    }
}



RTLog::~RTLog()
{
}


