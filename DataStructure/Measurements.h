#ifndef MEASUREMENTS_H
#define MEASUREMENTS_H

template <typename MeasurementType>
struct MeasurementBase {
    int    sensorId;
    double timeStamp;
    MeasurementType measurement;
};


#endif // MEASUREMENTS_H
