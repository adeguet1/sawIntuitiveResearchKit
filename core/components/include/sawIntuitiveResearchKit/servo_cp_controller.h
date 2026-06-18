/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _servo_cp_controller_h
#define _servo_cp_controller_h

#include <functional>
#include <string>

#include <cisstNumerical/nmrPInverse.h>
#include <cisstParameterTypes/prmStateCartesian.h>
#include <cisstRobot/robManipulator.h>

#include <sawIntuitiveResearchKit/arm_controller_manager.h>
#include <sawIntuitiveResearchKit/arm_controller_resource.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {

    class CISST_EXPORT servo_cp_controller: public arm_controller {
    public:
        typedef std::function<robManipulator::Errno(vctDoubleVec &, const vctFrm4x4 &)> inverse_kinematics_function;
        typedef std::function<const vctFrm4x4 &(void)> base_frame_function;
        typedef std::function<void(const std::string &)> error_function;

        servo_cp_controller();
        ~servo_cp_controller() override;

        void configure(const size_t number_of_joints,
                       inverse_kinematics_function inverse_kinematics,
                       base_frame_function base_frame,
                       error_function error);
        void configure(const size_t number_of_joints,
                       const arm_controller_joint_group & joint_group,
                       inverse_kinematics_function inverse_kinematics,
                       base_frame_function base_frame,
                       error_function error);
        void set_goal(const prmStateCartesian & goal);
        bool has_pending_goal(void) const;
        const arm_controller_resource_claim & claims(void) const override;
        void update(arm_state & state,
                    arm_command_sink & command_sink) override;

    private:
        bool m_has_pending_goal = false;
        size_t m_number_of_joints = 0;
        prmStateCartesian m_goal;
        arm_controller_resource_claim m_claims;
        inverse_kinematics_function m_inverse_kinematics;
        base_frame_function m_base_frame;
        error_function m_error;
        nmrPInverseDynamicData m_jacobian_pinverse_data;
    };
}

#endif // _servo_cp_controller_h
