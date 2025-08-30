#pragma once

#define LOG_ERROR(fmt, ...) \
    printf("[ERROR] %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define LOG_WARN(fmt, ...) \
    printf("[WARN] %s:%d: " fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define RCLCPP_ASSERT(logger, condition) do { \
        if(!(condition)) { \
            RCLCPP_FATAL(logger, "FATAL, file: %s, line: %u", __FILE__, __LINE__); \
            assert(condition); \
        } \
    } while(0)

#define RCLCPP_BREAK(logger) \
    do { \
        RCLCPP_FATAL(logger, "FATAL, file: %s, line: %u", __FILE__, __LINE__); \
        assert(false); \
    } while (0)
