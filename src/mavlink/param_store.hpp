#pragma once

#include <cstdint>

struct Param {
    char  name[17]; // null-terminated; MAVLink param_id is 16 chars max
    float value;
};

class ParamStore {
public:
    static constexpr uint16_t COUNT = 8;

    ParamStore();  // init from Config defaults, then load file

    // Returns index [0, COUNT) or -1 if not found. Uses strncmp(..., 16) for MAVLink compat.
    int         find_by_name(const char* id) const;

    float       get(uint16_t i)  const { return params_[i].value; }
    const char* name(uint16_t i) const { return params_[i].name;  }
    void        set(uint16_t i, float v) { params_[i].value = v;  }

    void save(const char* path) const;

private:
    void load(const char* path);

    Param params_[COUNT];
};
