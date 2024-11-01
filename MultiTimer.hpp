#ifndef MULTI_TIMER_HPP
#define MULTI_TIMER_HPP

#include <iostream>
#include <chrono>
#include <unordered_map>
#include <string>
#include <iomanip>

class MultiTimer {
public:
    // 获取单例实例
    static MultiTimer& getInstance() {
        static MultiTimer instance;
        return instance;
    }

    // 开始计时
    void start(const std::string& label) {
        auto startTime = std::chrono::high_resolution_clock::now();
        timings[label].start = startTime;
    }

    // 停止计时并记录时间
    void stop(const std::string& label) {
        auto endTime = std::chrono::high_resolution_clock::now();
        auto& timing = timings[label];
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(endTime - timing.start).count();

        timing.totalDuration += duration;
        timing.count++;
    }

    // 获取某段代码的平均执行时间
    double getAverageTime(const std::string& label) const {
        auto it = timings.find(label);
        if (it != timings.end() && it->second.count > 0) {
            return static_cast<double>(it->second.totalDuration) / it->second.count;
        }
        return 0.0; // 如果没有记录，则返回 0
    }

    // 重置某段代码的计时器
    void reset(const std::string& label) {
        timings[label] = TimingData();
    }

    // 重置所有代码段的计时器
    void resetAll() {
        timings.clear();
    }
    // 输出所有代码段的统计信息
    void printStatistics() const {
        std::cout << std::setw(20) << "Label"
            << std::setw(15) << "Count"
            << std::setw(20) << "Average Time (µs)" << std::endl;
        std::cout << "------------------------------------------------------------" << std::endl;
        for (const auto& pair : timings) {
            const std::string& label = pair.first;
            const TimingData& timing = pair.second;

            if (timing.count > 0) {
                double averageTime = static_cast<double>(timing.totalDuration) / timing.count;
                std::cout << std::setw(20) << label
                    << std::setw(15) << timing.count
                    << std::setw(20) << averageTime << std::endl;
            }
        }
    }
    // 删除拷贝构造和赋值运算符，确保单例
    MultiTimer(const MultiTimer&) = delete;
    MultiTimer& operator=(const MultiTimer&) = delete;

private:
    MultiTimer() = default; // 私有构造函数

    struct TimingData {
        std::chrono::high_resolution_clock::time_point start;
        long long totalDuration = 0;
        int count = 0;
    };

    std::unordered_map<std::string, TimingData> timings; // 使用 label 管理不同代码段的计时
};


#endif