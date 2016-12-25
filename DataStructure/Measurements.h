#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

#include <okvis_time/include/Time.hpp>

template <typename MeasurementType>
struct MeasurementBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~MeasurementBase() {}
    int             sensorId;
    okvis::Time     timeStamp;
    MeasurementType measurement;
};


#endif // MEASUREMENTS_H
