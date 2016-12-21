#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

template <typename MeasurementType>
struct MeasurementBase {
    virtual ~MeasurementBase() {}
    int    sensorId;
    double timeStamp;
    MeasurementType measurement;
};


#endif // MEASUREMENTS_H
