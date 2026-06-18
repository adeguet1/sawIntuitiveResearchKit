/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _servo_cf_controller_h
#define _servo_cf_controller_h

#include <functional>

#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstVector/vctDynamicVectorTypes.h>

#include <sawIntuitiveResearchKit/arm_controller_manager.h>
#include <sawIntuitiveResearchKit/arm_controller_resource.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {

    class CISST_EXPORT servo_cf_controller: public arm_controller {
    public:
        enum class reference_frame { body, spatial };

        typedef std::function<void(vctDoubleVec &, vctDoubleVec &)> preload_function;
        typedef std::function<bool(const arm_state &, prmForceCartesianSet &)> cartesian_impedance_function;
        typedef std::function<void(void)> orientation_lock_function;

        servo_cf_controller();
        ~servo_cf_controller() override;

        void configure(const size_t number_of_joints,
                       preload_function preload,
                       cartesian_impedance_function cartesian_impedance,
                       orientation_lock_function orientation_lock);
        void configure(const size_t number_of_joints,
                       const arm_controller_joint_group & joint_group,
                       preload_function preload,
                       cartesian_impedance_function cartesian_impedance,
                       orientation_lock_function orientation_lock);
        void set_goal(const prmForceCartesianSet & goal,
                      const reference_frame frame,
                      const bool body_orientation_absolute,
                      const bool orientation_locked);
        const arm_controller_resource_claim & claims(void) const override;
        void update(arm_state & state,
                    arm_command_sink & command_sink) override;

    private:
        size_t m_number_of_joints = 0;
        prmForceCartesianSet m_goal;
        reference_frame m_reference_frame = reference_frame::body;
        bool m_body_orientation_absolute = false;
        bool m_orientation_locked = false;
        arm_controller_resource_claim m_claims;
        preload_function m_preload;
        cartesian_impedance_function m_cartesian_impedance;
        orientation_lock_function m_orientation_lock;
        vctDoubleVec m_joint_effort;
    };
}

#endif // _servo_cf_controller_h
