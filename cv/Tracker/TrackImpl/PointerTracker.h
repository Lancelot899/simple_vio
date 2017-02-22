//
// Created by lancelot on 2/20/17.
//

#ifndef SIMPLE_VIO_POINTERTRACKER_H
#define SIMPLE_VIO_POINTERTRACKER_H

#include "AbstractTracker.h"

namespace direct_tracker {
    class PointerTracker : public AbstractTracker {
    public:
        PointerTracker();
        ~PointerTracker();
        virtual bool tracking(std::shared_ptr<viFrame> &viframe_i, std::shared_ptr<viFrame> &viframe_j, Translation &T_ij);

    };
}


#endif //SIMPLE_VIO_POINTERTRACKER_H
