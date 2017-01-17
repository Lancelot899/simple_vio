#include "imuFactor.h"
#include "DataStructure/viFrame.h"

imuFactor::imuFactor()
{

}

void imuFactor::makeConncect(const connection_t &from, const connection_t &to) {
    this->from = from;
    this->to = to;
}

bool imuFactor::checkConnect(const imuFactor::connection_t &from, const imuFactor::connection_t &to) {
    if(from->getID() != this->from->getID())
        return false;

    if(to->getID() != this->to->getID())
        return false;

    return true;
}

