/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <cisstNumerical/nmrPInverse.h>

#include <sawIntuitiveResearchKit/arm_command_sink.h>
#include <sawIntuitiveResearchKit/arm_state.h>
#include <sawIntuitiveResearchKit/servo_cp_controller.h>

dvrk::servo_cp_controller::servo_cp_controller():
    arm_controller("servo_cp")
{
}

dvrk::servo_cp_controller::~servo_cp_controller()
{
}

void dvrk::servo_cp_controller::configure(const size_t number_of_joints,
                                          inverse_kinematics_function inverse_kinematics,
                                          base_frame_function base_frame,
                                          error_function error)
{
    configure(number_of_joints,
              arm_controller_joint_group::all(number_of_joints),
              inverse_kinematics,
              base_frame,
              error);
}

void dvrk::servo_cp_controller::configure(const size_t number_of_joints,
                                          const arm_controller_joint_group & joint_group,
                                          inverse_kinematics_function inverse_kinematics,
                                          base_frame_function base_frame,
                                          error_function error)
{
    m_number_of_joints = number_of_joints;
    m_inverse_kinematics = inverse_kinematics;
    m_base_frame = base_frame;
    m_error = error;

    m_claims.clear();
    m_claims.claim_cartesian_pose();
    m_claims.claim_joint_position(joint_group);
}

void dvrk::servo_cp_controller::set_goal(const prmStateCartesian & goal)
{
    m_goal = goal;
    m_has_pending_goal = true;
}

bool dvrk::servo_cp_controller::has_pending_goal(void) const
{
    return m_has_pending_goal;
}

const dvrk::arm_controller_resource_claim & dvrk::servo_cp_controller::claims(void) const
{
    return m_claims;
}

void dvrk::servo_cp_controller::update(arm_state & state,
                                       arm_command_sink & command_sink)
{
    if (!m_has_pending_goal) {
        return;
    }

    m_has_pending_goal = false;

    vctDoubleVec jp(state.kin_measured_js.Position());
    if (m_goal.PositionIsValid()) {
        vctFrm4x4 cartesian_position;
        cartesian_position.From(m_goal.Position());

        if (m_inverse_kinematics(jp, m_base_frame().Inverse() * cartesian_position) != robManipulator::ESUCCESS) {
            m_error("unable to solve inverse kinematics");
            return;
        }
    } else if (m_goal.VelocityIsValid()) {
        jp = state.kin_measured_js.Position();
    } else {
        jp.Zeros();
    }

    vctDoubleVec jv(m_number_of_joints, 0.0);
    if (m_goal.VelocityIsValid()) {
        auto transform = state.measured_cp.Position().Rotation();

        vctDouble6 body_velocity;
        body_velocity.Ref<3>(0) = transform.ApplyInverseTo(m_goal.Velocity().Ref<3>(0));
        body_velocity.Ref<3>(3) = transform.ApplyInverseTo(m_goal.Velocity().Ref<3>(3));

        vctDoubleMat jacobian_copy(state.body_jacobian.rows(), state.body_jacobian.cols());
        jacobian_copy.Assign(state.body_jacobian);
        nmrPInverse(jacobian_copy, m_jacobian_pinverse_data);

        vctDoubleVec v(6);
        v.Assign(body_velocity);
        jv.ProductOf(m_jacobian_pinverse_data.PInverse(), v);
    } else {
        jv.Zeros();
    }

    vctDoubleVec jf(m_number_of_joints, 0.0);
    if (m_goal.ForceIsValid()) {
        auto transform = state.measured_cp.Position().Rotation();

        vctDoubleVec local_force(6);
        local_force.Ref(3, 0).Assign(transform.ApplyInverseTo(m_goal.Force().Ref<3>(0)));
        local_force.Ref(3, 3).Assign(transform.ApplyInverseTo(m_goal.Force().Ref<3>(3)));

        jf.ProductOf(state.body_jacobian.Transpose(), local_force);
    } else {
        jf.Zeros();
    }

    prmStateJoint js;
    js.Position() = jp;
    js.Velocity() = jv;
    js.Effort() = jf;

    command_sink.servo_js(js);
}
