/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _arm_command_sink_h
#define _arm_command_sink_h

#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstParameterTypes/prmStateJoint.h>

class mtsIntuitiveResearchKitArm;

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {

    class CISST_EXPORT arm_command_sink {
    public:
        arm_command_sink(mtsIntuitiveResearchKitArm & arm);
        ~arm_command_sink();

        void servo_jp(const vctDoubleVec & jp,
                      const vctDoubleVec & jv);
        void servo_jf(const vctDoubleVec & jf);
        void servo_js(const prmStateJoint & js);
        void feed_forward_jf(const vctDoubleVec & jf);

        void enable_torque_mode(const vctBoolVec & enabled);
        void enable_measured_setpoint_check(const bool enabled);
        void enforce_position_limits(const bool enabled);

    private:
        mtsIntuitiveResearchKitArm & m_arm;
    };
}

#endif // _arm_command_sink_h
