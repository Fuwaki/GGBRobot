#pragma once

#include "esp_timer.h"
#include "esp_log.h"
#include <map>
#include <string>
#include <vector>

class TimingProfiler {
public:
    // Starts the timer for a specific section.
    void start() {
        start_time_ = esp_timer_get_time();
    }

    // Ends the timer for a section and records the duration.
    void end(const std::string& name) {
        if (start_time_ == 0) {
            return; // start() was not called
        }
        int64_t elapsed = esp_timer_get_time() - start_time_;
        
        // If the name is new, add it to our ordered list
        if (timings_.find(name) == timings_.end()) {
            ordered_keys_.push_back(name);
        }
        timings_[name] += elapsed;
        start_time_ = 0; // Reset for the next measurement
    }

    // Logs the average time for all recorded sections over a given interval.
    void log_results(const char* tag, int interval_count) {
        if (interval_count == 0) {
            return;
        }
        ESP_LOGI(tag, "--- Timing Profile (avg over %d frames) ---", interval_count);
        for (const auto& key : ordered_keys_) {
            int64_t avg_time = timings_[key] / interval_count;
            ESP_LOGI(tag, "  %s: %lld us", key.c_str(), avg_time);
        }
    }

    // Resets all accumulated timings.
    void reset() {
        for (auto& pair : timings_) {
            pair.second = 0;
        }
    }

private:
    int64_t start_time_ = 0;
    std::map<std::string, int64_t> timings_;
    std::vector<std::string> ordered_keys_; // Maintain insertion order for logging
};
