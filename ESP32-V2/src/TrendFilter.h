// TrendFilter.h
#pragma once
#include <string.h>

class TrendFilter {
public:
    static constexpr int WINDOW = 30; // 600 ms @ 50 Hz
    float buf[WINDOW];
    int idx, count;

    TrendFilter() { reset(); }

    void reset() {
        idx = count = 0;
        memset(buf, 0, sizeof(buf));
    }

    float update(float x) {
        buf[idx] = x;
        idx = (idx + 1) % WINDOW;
        if (count < WINDOW) count++;

        float sum = 0.0f;
        for (int i = 0; i < count; i++) sum += buf[i];
        return sum / count;
    }
};
