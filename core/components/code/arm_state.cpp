/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <sawIntuitiveResearchKit/arm_state.h>

dvrk::arm_state::arm_state()
{
    pid_measured_js.SetAutomaticTimestamp(false);
    pid_setpoint_js.SetAutomaticTimestamp(false);
    kin_measured_js.SetAutomaticTimestamp(false);
    kin_setpoint_js.SetAutomaticTimestamp(false);
    gravity_compensation_setpoint_js.SetAutomaticTimestamp(false);

    local_measured_cp.SetAutomaticTimestamp(false);
    local_setpoint_cp.SetAutomaticTimestamp(false);
    measured_cp.SetAutomaticTimestamp(false);
    setpoint_cp.SetAutomaticTimestamp(false);
    measured_cs.SetAutomaticTimestamp(false);

    local_measured_cv.SetAutomaticTimestamp(false);
    local_measured_cv.SetVelocityLinear(vct3(0.0));
    local_measured_cv.SetVelocityAngular(vct3(0.0));
    local_measured_cv.SetValid(false);

    measured_cv.SetAutomaticTimestamp(false);
    measured_cv.SetVelocityLinear(vct3(0.0));
    measured_cv.SetVelocityAngular(vct3(0.0));
    measured_cv.SetValid(false);

    local_setpoint_cv.SetAutomaticTimestamp(false);
    local_setpoint_cv.SetVelocityLinear(vct3(0.0));
    local_setpoint_cv.SetVelocityAngular(vct3(0.0));
    local_setpoint_cv.SetValid(false);

    setpoint_cv.SetAutomaticTimestamp(false);
    setpoint_cv.SetVelocityLinear(vct3(0.0));
    setpoint_cv.SetVelocityAngular(vct3(0.0));
    setpoint_cv.SetValid(false);

    body_measured_cf.SetAutomaticTimestamp(false);
    body_measured_cf.SetValid(false);
    spatial_measured_cf.SetAutomaticTimestamp(false);
    spatial_measured_cf.SetValid(false);
}

void dvrk::arm_state::resize(const size_t number_of_joints,
                             const size_t number_of_joints_kinematics)
{
    pid_measured_js.Position().SetSize(number_of_joints);
    pid_measured_js.Velocity().SetSize(number_of_joints);
    pid_measured_js.Effort().SetSize(number_of_joints);

    pid_setpoint_js.Position().SetSize(number_of_joints);
    pid_setpoint_js.Velocity().SetSize(number_of_joints);
    pid_setpoint_js.Effort().SetSize(number_of_joints);

    resize_kinematics(number_of_joints_kinematics);

    configuration_js.Type().SetSize(number_of_joints_kinematics);
    configuration_js.PositionMin().SetSize(number_of_joints_kinematics);
    configuration_js.PositionMax().SetSize(number_of_joints_kinematics);
    configuration_js.EffortMin().SetSize(number_of_joints_kinematics);
    configuration_js.EffortMax().SetSize(number_of_joints_kinematics);
}

void dvrk::arm_state::resize_kinematics(const size_t number_of_joints_kinematics)
{
    kin_measured_js.Position().SetSize(number_of_joints_kinematics);
    kin_measured_js.Velocity().SetSize(number_of_joints_kinematics);
    kin_measured_js.Effort().SetSize(number_of_joints_kinematics);

    kin_setpoint_js.Position().SetSize(number_of_joints_kinematics);
    kin_setpoint_js.Velocity().SetSize(number_of_joints_kinematics);
    kin_setpoint_js.Effort().SetSize(number_of_joints_kinematics);

    gravity_compensation_setpoint_js.Effort().SetSize(number_of_joints_kinematics);
    gravity_compensation_setpoint_js.Effort().Zeros();
    gravity_compensation_setpoint_js.SetValid(false);

    body_jacobian.SetSize(6, number_of_joints_kinematics);
    spatial_jacobian.SetSize(6, number_of_joints_kinematics);
    body_jacobian_transpose.ForceAssign(body_jacobian.Transpose());
    spatial_jacobian_transpose.ForceAssign(spatial_jacobian.Transpose());
}

void dvrk::arm_state::apply_actuator_to_joint_coupling(const prmActuatorJointCoupling & coupling)
{
    pid_measured_js.Position() = coupling.ActuatorToJointPosition() * pid_measured_js.Position();
    pid_measured_js.Velocity() = coupling.ActuatorToJointPosition() * pid_measured_js.Velocity();
    pid_measured_js.Effort() = coupling.ActuatorToJointEffort() * pid_measured_js.Effort();

    pid_setpoint_js.Position() = coupling.ActuatorToJointPosition() * pid_setpoint_js.Position();
    pid_setpoint_js.Velocity() = coupling.ActuatorToJointPosition() * pid_setpoint_js.Velocity();
    pid_setpoint_js.Effort() = coupling.ActuatorToJointEffort() * pid_setpoint_js.Effort();
}

void dvrk::arm_state::invalidate_joint_state(void)
{
    pid_measured_js.Position().Zeros();
    pid_measured_js.Velocity().Zeros();
    pid_measured_js.Effort().Zeros();
    pid_measured_js.SetValid(false);

    kin_measured_js.Position().Zeros();
    kin_measured_js.Velocity().Zeros();
    kin_measured_js.Effort().Zeros();
    kin_measured_js.SetValid(false);

    gravity_compensation_setpoint_js.SetValid(false);
}

void dvrk::arm_state::invalidate_cartesian_state(void)
{
    local_measured_cp_frame.Assign(vctFrm4x4::Identity());
    measured_cp_frame.Assign(vctFrm4x4::Identity());
    local_measured_cp.SetValid(false);
    measured_cp.SetValid(false);

    local_measured_cv.SetValid(false);
    measured_cv.SetValid(false);
    body_measured_cf.SetValid(false);
    spatial_measured_cf.SetValid(false);

    local_setpoint_cp_frame.Assign(vctFrm4x4::Identity());
    setpoint_cp_frame.Assign(vctFrm4x4::Identity());
    local_setpoint_cp.SetValid(false);
    setpoint_cp.SetValid(false);

    local_setpoint_cv.SetValid(false);
    setpoint_cv.SetValid(false);

    measured_cs.SetValid(false);
}
