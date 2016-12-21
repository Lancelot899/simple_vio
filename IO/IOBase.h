#ifndef IOBASE_H
#define IOBASE_H

#include <memory>

template<typename DataType>
class IOBase
{
public:
    IOBase() {}
    virtual void push(std::shared_ptr<DataType> data) {
        data.reset();
        return;
    }

    virtual std::shared_ptr<DataType> pop() = 0;
};

#endif // IOBASE_H
