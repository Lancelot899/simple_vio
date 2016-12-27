#ifndef IOBASE_H
#define IOBASE_H

#include <memory>

template<typename DataType>
class IOBase {
public:
    typedef DataType                  data_t;
    typedef DataType&                 refference_t;
    typedef const refference_t        const_refference_t;
    typedef std::shared_ptr<data_t>   pData_t;
    typedef const pData_t             const_pData_t;

    IOBase() {}
    virtual ~IOBase() {}
};

#endif // IOBASE_H
