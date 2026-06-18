/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _arm_state_h
#define _arm_state_h

#include <cisstParameterTypes/prmActuatorJointCoupling.h>
#include <cisstParameterTypes/prmConfigurationJoint.h>
#include <cisstParameterTypes/prmForceCartesianGet.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstParameterTypes/prmStateJoint.h>
#include <cisstParameterTypes/prmStateCartesian.h>
#include <cisstParameterTypes/prmVelocityCartesianGet.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {

    class CISST_EXPORT arm_state {
    public:
        arm_state();

        void resize(const size_t number_of_joints,
                    const size_t number_of_joints_kinematics);
        void resize_kinematics(const size_t number_of_joints_kinematics);
        void apply_actuator_to_joint_coupling(const prmActuatorJointCoupling & coupling);
        void invalidate_joint_state(void);
        void invalidate_cartesian_state(void);

        prmStateJoint pid_measured_js;
        prmStateJoint pid_setpoint_js;
        prmStateJoint kin_measured_js;
        prmStateJoint kin_setpoint_js;
        prmStateJoint gravity_compensation_setpoint_js;
        prmConfigurationJoint configuration_js;

        prmPositionCartesianGet local_measured_cp;
        vctFrm4x4 local_measured_cp_frame;
        prmPositionCartesianGet local_setpoint_cp;
        vctFrm4x4 local_setpoint_cp_frame;

        prmPositionCartesianGet measured_cp;
        vctFrm4x4 measured_cp_frame;
        prmPositionCartesianGet setpoint_cp;
        vctFrm4x4 setpoint_cp_frame;
        prmStateCartesian measured_cs;

        prmVelocityCartesianGet local_measured_cv;
        prmVelocityCartesianGet measured_cv;
        prmVelocityCartesianGet local_setpoint_cv;
        prmVelocityCartesianGet setpoint_cv;

        vctDoubleMat body_jacobian;
        vctDoubleMat body_jacobian_transpose;
        vctDoubleMat spatial_jacobian;
        vctDoubleMat spatial_jacobian_transpose;

        prmForceCartesianGet body_measured_cf;
        prmForceCartesianGet spatial_measured_cf;
    };
}

#endif // _arm_state_h
