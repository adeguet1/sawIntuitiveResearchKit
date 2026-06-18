/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <sawIntuitiveResearchKit/arm_command_sink.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKit.h>
#include <sawIntuitiveResearchKit/move_jp_controller.h>

dvrk::move_jp_controller::move_jp_controller():
    arm_controller("move_jp"),
    m_ratio_v(mtsIntuitiveResearchKit::JointTrajectory::ratio_v),
    m_ratio_a(mtsIntuitiveResearchKit::JointTrajectory::ratio_a),
    m_ratio(mtsIntuitiveResearchKit::JointTrajectory::ratio)
{
}

dvrk::move_jp_controller::~move_jp_controller()
{
}

void dvrk::move_jp_controller::configure(const size_t number_of_joints,
                                         time_function time,
                                         stop_function stop,
                                         error_function error)
{
    configure(number_of_joints,
              arm_controller_joint_group::all(number_of_joints),
              time,
              stop,
              error);
}

void dvrk::move_jp_controller::configure(const size_t number_of_joints,
                                         const arm_controller_joint_group & joint_group,
                                         time_function time,
                                         stop_function stop,
                                         error_function error)
{
    m_time = time;
    m_stop = stop;
    m_error = error;

    m_v_max.SetSize(number_of_joints);
    m_v.SetSize(number_of_joints);
    m_a_max.SetSize(number_of_joints);
    m_a.SetSize(number_of_joints);
    m_goal.SetSize(number_of_joints);
    m_goal_v.SetSize(number_of_joints);
    m_goal_error.SetSize(number_of_joints);
    m_goal_tolerance.SetSize(number_of_joints);
    m_jp.SetSize(number_of_joints);
    m_jv.SetSize(number_of_joints);

    m_goal_v.Zeros();
    m_goal_tolerance.SetAll(0.0);
    m_is_active = false;
    m_end_time = 0.0;
    m_period = 0.0;

    m_claims.clear();
    m_claims.claim_joint_position(joint_group);
}

void dvrk::move_jp_controller::configure_limits(const vctDoubleVec & v_max,
                                                const vctDoubleVec & a_max,
                                                const vctDoubleVec & goal_tolerance)
{
    m_v_max.Assign(v_max);
    m_a_max.Assign(a_max);
    m_goal_tolerance.Assign(goal_tolerance);
    update_reflexxes();
}

void dvrk::move_jp_controller::configure_period(const double period)
{
    m_period = period;
    update_reflexxes();
}

void dvrk::move_jp_controller::set_ratio_v(const double ratio)
{
    m_ratio_v = ratio;
    update_ratio();
    update_reflexxes();
}

void dvrk::move_jp_controller::set_ratio_a(const double ratio)
{
    m_ratio_a = ratio;
    update_ratio();
    update_reflexxes();
}

void dvrk::move_jp_controller::set_ratio(const double ratio)
{
    m_ratio = ratio;
    m_ratio_v = ratio;
    m_ratio_a = ratio;
    update_reflexxes();
}

double dvrk::move_jp_controller::ratio_v(void) const
{
    return m_ratio_v;
}

double dvrk::move_jp_controller::ratio_a(void) const
{
    return m_ratio_a;
}

double dvrk::move_jp_controller::ratio(void) const
{
    return m_ratio;
}

bool dvrk::move_jp_controller::is_active(void) const
{
    return m_is_active;
}

const vctDoubleVec & dvrk::move_jp_controller::goal(void) const
{
    return m_goal;
}

const vctDoubleVec & dvrk::move_jp_controller::current_position(void) const
{
    return m_jp;
}

const vctDoubleVec & dvrk::move_jp_controller::current_velocity(void) const
{
    return m_jv;
}

void dvrk::move_jp_controller::start(const vctDoubleVec & current_position,
                                     const vctDoubleVec & current_velocity,
                                     const vctDoubleVec & goal)
{
    m_jp.Assign(current_position);
    m_jv.Assign(current_velocity);
    m_goal.Assign(goal);
    m_goal_v.Zeros();
    m_is_active = true;
    m_end_time = 0.0;
}

void dvrk::move_jp_controller::stop(const bool goal_reached)
{
    m_stop(goal_reached);
    m_is_active = false;
}

robReflexxes::ResultType dvrk::move_jp_controller::evaluate(arm_command_sink & command_sink)
{
    m_reflexxes.Evaluate(m_jp,
                         m_jv,
                         m_goal,
                         m_goal_v);
    command_sink.servo_jp(m_jp, m_jv);

    if ((m_reflexxes.ResultValue() == robReflexxes::Reflexxes_WORKING)
        && (m_end_time == 0.0)) {
        m_end_time = m_time() + m_reflexxes.Duration();
    }
    return m_reflexxes.ResultValue();
}

double dvrk::move_jp_controller::end_time(void) const
{
    return m_end_time;
}

const dvrk::arm_controller_resource_claim & dvrk::move_jp_controller::claims(void) const
{
    return m_claims;
}

void dvrk::move_jp_controller::update(arm_state &,
                                      arm_command_sink & command_sink)
{
    if (!m_is_active) {
        return;
    }

    switch (evaluate(command_sink)) {
    case robReflexxes::Reflexxes_WORKING:
        break;
    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        stop(true);
        break;
    default:
        m_error();
        stop(false);
        break;
    }
}

void dvrk::move_jp_controller::update_ratio(void)
{
    if (m_ratio_v == m_ratio_a) {
        m_ratio = m_ratio_v;
    } else {
        m_ratio = 0.0;
    }
}

void dvrk::move_jp_controller::update_reflexxes(void)
{
    m_v.ProductOf(m_ratio_v, m_v_max);
    m_a.ProductOf(m_ratio_a, m_a_max);
    if (m_period > 0.0) {
        m_reflexxes.Set(m_v,
                        m_a,
                        m_period,
                        robReflexxes::Reflexxes_TIME);
    }
}
