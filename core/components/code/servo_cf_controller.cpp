/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <sawIntuitiveResearchKit/arm_command_sink.h>
#include <sawIntuitiveResearchKit/arm_state.h>
#include <sawIntuitiveResearchKit/servo_cf_controller.h>

dvrk::servo_cf_controller::servo_cf_controller():
    arm_controller("servo_cf")
{
}

dvrk::servo_cf_controller::~servo_cf_controller()
{
}

void dvrk::servo_cf_controller::configure(const size_t number_of_joints,
                                          preload_function preload,
                                          cartesian_impedance_function cartesian_impedance,
                                          orientation_lock_function orientation_lock)
{
    configure(number_of_joints,
              arm_controller_joint_group::all(number_of_joints),
              preload,
              cartesian_impedance,
              orientation_lock);
}

void dvrk::servo_cf_controller::configure(const size_t number_of_joints,
                                          const arm_controller_joint_group & joint_group,
                                          preload_function preload,
                                          cartesian_impedance_function cartesian_impedance,
                                          orientation_lock_function orientation_lock)
{
    m_number_of_joints = number_of_joints;
    m_preload = preload;
    m_cartesian_impedance = cartesian_impedance;
    m_orientation_lock = orientation_lock;
    m_joint_effort.SetSize(number_of_joints);
    m_joint_effort.Zeros();

    m_claims.clear();
    m_claims.claim_cartesian_wrench();
    m_claims.claim_joint_effort(joint_group);
}

void dvrk::servo_cf_controller::set_goal(const prmForceCartesianSet & goal,
                                         const reference_frame frame,
                                         const bool body_orientation_absolute,
                                         const bool orientation_locked)
{
    m_goal = goal;
    m_reference_frame = frame;
    m_body_orientation_absolute = body_orientation_absolute;
    m_orientation_locked = orientation_locked;
}

const dvrk::arm_controller_resource_claim & dvrk::servo_cf_controller::claims(void) const
{
    return m_claims;
}

void dvrk::servo_cf_controller::update(arm_state & state,
                                       arm_command_sink & command_sink)
{
    vctDoubleVec wrench(6);
    vctDoubleVec effort_preload(m_number_of_joints);
    vctDoubleVec wrench_preload(6);

    m_preload(effort_preload, wrench_preload);

    if (m_reference_frame == reference_frame::body) {
        prmForceCartesianSet goal(m_goal);
        const bool impedance_wrench = m_cartesian_impedance(state, goal);
        if (impedance_wrench) {
            wrench.Assign(goal.Force());
        } else if (m_body_orientation_absolute) {
            vct3 relative, absolute;
            relative.Assign(goal.Force().Ref<3>(0));
            state.measured_cp_frame.Rotation().ApplyInverseTo(relative, absolute);
            wrench.Ref(3, 0).Assign(absolute);

            relative.Assign(goal.Force().Ref<3>(3));
            state.measured_cp_frame.Rotation().ApplyInverseTo(relative, absolute);
            wrench.Ref(3, 3).Assign(absolute);
        } else {
            wrench.Assign(goal.Force());
        }
        m_joint_effort.ProductOf(state.body_jacobian.Transpose(), wrench + wrench_preload);
    } else {
        wrench.Assign(m_goal.Force());
        m_joint_effort.ProductOf(state.spatial_jacobian.Transpose(), wrench + wrench_preload);
    }

    m_joint_effort.Add(effort_preload);
    command_sink.servo_jf(m_joint_effort);

    if (m_orientation_locked) {
        m_orientation_lock();
    }
}
