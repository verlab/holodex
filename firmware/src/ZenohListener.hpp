#pragma once
#include <string> // to include size_t

extern "C" {
#include <zenoh-pico.h>
}

class IZenohListener {
public:

    virtual std::string getTopic() = 0;
    virtual void handle(const z_loaned_sample_t* sample) = 0;
    virtual size_t getSize() = 0;
    
    virtual ~IZenohListener() = 0;
};