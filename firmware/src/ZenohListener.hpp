#pragma once
#include <string> // to include size_t


class IZenohListener {
public:

    virtual std::string getTopic() = 0;
    virtual void handle(void* data) = 0;
    virtual size_t getSize() = 0;
    
    virtual ~IZenohListener() = 0;
};