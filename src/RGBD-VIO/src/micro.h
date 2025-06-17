#pragma once

#define RCLCPP_ASSERT(logger, condition) do { \
        RCLCPP_FATAL(logger, "Assert Faile"); \
        assert(condition); \
    } while(0)

#define RCLCPP_BREAK(logger) \
    do { \
        RCLCPP_FATAL(logger, "ROS_BREAK() called!"); \
        assert(false); \
    } while (0)
