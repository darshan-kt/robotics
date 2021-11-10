#include <local_planner/date_time.h>
std::string CurrentDateTime()
{
    auto now = std::chrono::system_clock::now();
    // printf("%s",now);
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
    return ss.str();
}
