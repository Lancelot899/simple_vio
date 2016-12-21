#include "imuFactor.h"
#include "DataStructure/viFrame.h"

imuFactor::imuFactor()
{

}

bool imuFactor::checkConnect(const imuFactor::connection_t &from, const imuFactor::connection_t &to) {
    if(from->getID() != this->from->getID())
        return false;
    if(to->getID() != this->to->getID())
        return false;

    return true;
}

