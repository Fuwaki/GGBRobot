#pragma once

#include "esp_timer.h"
#include "esp_log.h"
#include <map>
#include <string>
#include <vector>

/**
 * @brief 一个简单的计时分析器, 用于测量代码块的执行时间
 */
class TimingProfiler {
public:
    /**
     * @brief 开始一个计时段
     */
    void start() {
        start_time_ = esp_timer_get_time();
    }

    /**
     * @brief 结束一个计时段并记录耗时
     * @param name 计时段的名称
     */
    void end(const std::string& name) {
        if (start_time_ == 0) {
            return; // start() 未被调用
        }
        int64_t elapsed = esp_timer_get_time() - start_time_;
        
        // 如果是新的计时段名称, 将其添加到有序列表中
        if (timings_.find(name) == timings_.end()) {
            ordered_keys_.push_back(name);
        }
        timings_[name] += elapsed;
        start_time_ = 0; // 为下一次测量重置
    }

    /**
     * @brief 打印指定间隔内所有计时段的平均耗时
     * @param tag 用于ESP_LOG的日志标签
     * @param interval_count 间隔内的计次 (例如, 帧数)
     */
    void log_results(const char* tag, int interval_count) {
        if (interval_count == 0) {
            return;
        }
        ESP_LOGI(tag, "--- 性能分析 (最近 %d 帧平均耗时) ---", interval_count);
        for (const auto& key : ordered_keys_) {
            int64_t avg_time_us = timings_[key] / interval_count;
            ESP_LOGI(tag, "  - %s: %lld us", key.c_str(), avg_time_us);
        }
    }

    /**
     * @brief 重置所有累计的计时数据
     */
    void reset() {
        for (auto& pair : timings_) {
            pair.second = 0;
        }
    }

private:
    int64_t start_time_ = 0;
    std::map<std::string, int64_t> timings_;
    std::vector<std::string> ordered_keys_; // 保持插入顺序以便有序打印
};