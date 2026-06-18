/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <sawIntuitiveResearchKit/arm_kinematics.h>

dvrk::arm_kinematics::arm_kinematics():
    m_base_frame(vctFrm4x4::Identity()),
    m_base_frame_valid(true)
{
}

dvrk::arm_kinematics::~arm_kinematics()
{
}

bool dvrk::arm_kinematics::has_manipulator(void) const
{
    return static_cast<bool>(m_manipulator);
}

robManipulator & dvrk::arm_kinematics::manipulator(void)
{
    return *m_manipulator;
}

robManipulator & dvrk::arm_kinematics::manipulator(void) const
{
    return *m_manipulator;
}

robManipulator * dvrk::arm_kinematics::manipulator_pointer(void)
{
    return m_manipulator.get();
}

robManipulator * dvrk::arm_kinematics::manipulator_pointer(void) const
{
    return m_manipulator.get();
}

void dvrk::arm_kinematics::set_manipulator(std::unique_ptr<robManipulator> manipulator)
{
    m_manipulator = std::move(manipulator);
}

vctFrm4x4 & dvrk::arm_kinematics::base_frame(void)
{
    return m_base_frame;
}

const vctFrm4x4 & dvrk::arm_kinematics::base_frame(void) const
{
    return m_base_frame;
}

void dvrk::arm_kinematics::set_base_frame(const vctFrm4x4 & base_frame)
{
    m_base_frame = base_frame;
}

bool dvrk::arm_kinematics::base_frame_valid(void) const
{
    return m_base_frame_valid;
}

void dvrk::arm_kinematics::set_base_frame_valid(const bool valid)
{
    m_base_frame_valid = valid;
}

nmrPInverseDynamicData & dvrk::arm_kinematics::jacobian_pinverse_data(void)
{
    return m_jacobian_pinverse_data;
}

nmrPInverseDynamicData & dvrk::arm_kinematics::jacobian_transpose_pinverse_data(void)
{
    return m_jacobian_transpose_pinverse_data;
}

void dvrk::arm_kinematics::update_cartesian_state(arm_state & state,
                                                  const bool body_cf_orientation_absolute)
{
    state.local_measured_cp_frame = manipulator().ForwardKinematics(state.kin_measured_js.Position());
    state.measured_cp_frame = base_frame() * state.local_measured_cp_frame;

    state.local_measured_cp_frame.Rotation().NormalizedSelf();
    state.measured_cp_frame.Rotation().NormalizedSelf();

    state.local_measured_cp.Position().From(state.local_measured_cp_frame);
    state.local_measured_cp.SetTimestamp(state.kin_measured_js.Timestamp());
    state.local_measured_cp.SetValid(true);
    state.measured_cp.Position().From(state.measured_cp_frame);
    state.measured_cp.SetTimestamp(state.kin_measured_js.Timestamp());
    state.measured_cp.SetValid(base_frame_valid());

    state.measured_cs.Position().From(state.measured_cp_frame);
    state.measured_cs.SetTimestamp(state.kin_measured_js.Timestamp());
    state.measured_cs.SetValid(base_frame_valid());
    state.measured_cs.PositionIsValid() = true;

    manipulator().JacobianSpatial(state.kin_measured_js.Position(), state.spatial_jacobian);
    manipulator().JacobianBody(state.kin_measured_js.Position(), state.body_jacobian);

    vctDoubleVec cartesian_velocity(6);
    cartesian_velocity.ProductOf(state.body_jacobian, state.kin_measured_js.Velocity());
    vct3 relative, absolute;

    relative.Assign(cartesian_velocity.Ref(3, 0));
    state.local_measured_cv.SetVelocityLinear(relative);
    state.measured_cp_frame.Rotation().ApplyTo(relative, absolute);
    state.measured_cv.SetVelocityLinear(absolute);

    relative.Assign(cartesian_velocity.Ref(3, 3));
    state.local_measured_cv.SetVelocityAngular(relative);
    state.measured_cp_frame.Rotation().ApplyTo(relative, absolute);
    state.measured_cv.SetVelocityAngular(absolute);

    state.local_measured_cv.SetValid(true);
    state.local_measured_cv.SetTimestamp(state.kin_measured_js.Timestamp());
    state.measured_cv.SetValid(true);
    state.measured_cv.SetTimestamp(state.kin_measured_js.Timestamp());

    state.measured_cs.Velocity().Ref<3>(0) = state.measured_cv.VelocityLinear();
    state.measured_cs.Velocity().Ref<3>(3) = state.measured_cv.VelocityAngular();
    state.measured_cs.VelocityIsValid() = true;

    state.body_jacobian_transpose.Assign(state.body_jacobian.Transpose());
    nmrPInverse(state.body_jacobian_transpose, jacobian_transpose_pinverse_data());
    vctDoubleVec wrench(6);
    wrench.ProductOf(jacobian_transpose_pinverse_data().PInverse(), state.kin_measured_js.Effort());

    vct6 absolute_wrench;
    relative.Assign(wrench.Ref(3, 0));
    state.measured_cp_frame.Rotation().ApplyTo(relative, absolute);
    absolute_wrench.Ref<3>(0) = absolute;
    relative.Assign(wrench.Ref(3, 3));
    state.measured_cp_frame.Rotation().ApplyTo(relative, absolute);
    absolute_wrench.Ref<3>(3) = absolute;

    state.measured_cs.Force() = absolute_wrench;
    state.measured_cs.ForceIsValid() = true;

    if (body_cf_orientation_absolute) {
        state.body_measured_cf.Force().Assign(absolute_wrench);
    } else {
        state.body_measured_cf.Force().Assign(wrench);
    }

    state.body_measured_cf.SetValid(true);
    state.body_measured_cf.SetTimestamp(state.kin_measured_js.Timestamp());

    state.spatial_jacobian_transpose.Assign(state.spatial_jacobian.Transpose());
    nmrPInverse(state.spatial_jacobian_transpose, jacobian_transpose_pinverse_data());
    wrench.ProductOf(jacobian_transpose_pinverse_data().PInverse(), state.kin_measured_js.Effort());
    state.spatial_measured_cf.Force().Assign(wrench);
    state.spatial_measured_cf.SetValid(true);
    state.spatial_measured_cf.SetTimestamp(state.kin_measured_js.Timestamp());

    state.local_setpoint_cp_frame = manipulator().ForwardKinematics(state.kin_setpoint_js.Position());
    state.setpoint_cp_frame = base_frame() * state.local_setpoint_cp_frame;

    state.local_setpoint_cp_frame.Rotation().NormalizedSelf();
    state.setpoint_cp_frame.Rotation().NormalizedSelf();

    state.local_setpoint_cp.Position().From(state.local_setpoint_cp_frame);
    state.local_setpoint_cp.SetTimestamp(state.kin_setpoint_js.Timestamp());
    state.local_setpoint_cp.SetValid(true);
    state.setpoint_cp.Position().From(state.setpoint_cp_frame);
    state.setpoint_cp.SetTimestamp(state.kin_setpoint_js.Timestamp());
    state.setpoint_cp.SetValid(base_frame_valid());

    cartesian_velocity.ProductOf(state.body_jacobian, state.kin_setpoint_js.Velocity());

    relative.Assign(cartesian_velocity.Ref(3, 0));
    state.local_setpoint_cv.SetVelocityLinear(relative);
    state.setpoint_cp_frame.Rotation().ApplyTo(relative, absolute);
    state.setpoint_cv.SetVelocityLinear(absolute);

    relative.Assign(cartesian_velocity.Ref(3, 3));
    state.local_setpoint_cv.SetVelocityAngular(relative);
    state.setpoint_cp_frame.Rotation().ApplyTo(relative, absolute);
    state.setpoint_cv.SetVelocityAngular(absolute);

    state.local_setpoint_cv.SetValid(true);
    state.local_setpoint_cv.SetTimestamp(state.kin_setpoint_js.Timestamp());
    state.setpoint_cv.SetValid(true);
    state.setpoint_cv.SetTimestamp(state.kin_setpoint_js.Timestamp());
}

void dvrk::arm_kinematics::invalidate_cartesian_state(arm_state & state)
{
    state.invalidate_cartesian_state();
}
