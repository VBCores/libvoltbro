#pragma once

class AbstractFilter {
public:
    virtual float operator () (float prev_value, float new_value) const = 0;
};
