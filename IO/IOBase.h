#ifndef IOBASE_H
#define IOBASE_H

#include <memory>

namespace okvis {
    class Time;
}

template<typename DataType>
class IOBase {
public:
    IOBase() {}
    virtual ~IOBase() {}
    virtual void push(std::shared_ptr<DataType> data) {
        data.reset();
        return;
    }

    virtual std::shared_ptr<DataType> pop(okvis::Time&, okvis::Time&) {
        return std::shared_ptr<DataType>();
    }

    virtual std::shared_ptr<DataType> pop(okvis::Time&) {
        return std::shared_ptr<DataType>();
    }

    virtual std::shared_ptr<DataType> pop() {
        return std::shared_ptr<DataType>();
    }
};

#endif // IOBASE_H
