#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

template <typename MeasurementType>
struct MeasurementBase {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~MeasurementBase() {}
    int    sensorId;
    double timeStamp;
    MeasurementType measurement;
};


#endif // MEASUREMENTS_H
