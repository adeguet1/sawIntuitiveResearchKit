/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <algorithm>

#include <sawIntuitiveResearchKit/arm_command_sink.h>
#include <sawIntuitiveResearchKit/arm_controller_manager.h>
#include <sawIntuitiveResearchKit/arm_state.h>
#include <sawIntuitiveResearchKit/move_jp_controller.h>
#include <sawIntuitiveResearchKit/servo_cf_controller.h>
#include <sawIntuitiveResearchKit/servo_cp_controller.h>
#include <sawIntuitiveResearchKit/servo_jp_controller.h>

dvrk::legacy_callback_controller::legacy_callback_controller():
    arm_controller("legacy_callback")
{
}

dvrk::legacy_callback_controller::~legacy_callback_controller()
{
    clear();
}

void dvrk::legacy_callback_controller::set_callback(mtsCallableVoidBase * callback)
{
    if (m_callback != callback) {
        clear();
    }
    m_callback = callback;
}

void dvrk::legacy_callback_controller::clear(void)
{
    if (m_callback) {
        delete m_callback;
        m_callback = 0;
    }
}

bool dvrk::legacy_callback_controller::has_callback(void) const
{
    return (m_callback != 0);
}

const dvrk::arm_controller_resource_claim & dvrk::legacy_callback_controller::claims(void) const
{
    return m_claims;
}

void dvrk::legacy_callback_controller::update(arm_state &,
                                              arm_command_sink &)
{
    if (m_callback) {
        m_callback->Execute();
    }
}

dvrk::arm_controller_manager::arm_controller_manager()
{
    m_move_jp_controller.reset(new move_jp_controller());
    m_servo_cf_controller.reset(new servo_cf_controller());
    m_servo_cp_controller.reset(new servo_cp_controller());
    m_servo_jp_controller.reset(new servo_jp_controller());
}

dvrk::arm_controller_manager::~arm_controller_manager()
{
    clear();
}

void dvrk::arm_controller_manager::set_legacy_callback(mtsCallableVoidBase * callback)
{
    m_legacy_controller.set_callback(callback);
    activate_controller(&m_legacy_controller);
}

void dvrk::arm_controller_manager::configure_move_jp_controller(const size_t number_of_joints,
                                                                std::function<double(void)> time,
                                                                std::function<void(bool)> stop,
                                                                std::function<void(void)> error)
{
    configure_move_jp_controller(number_of_joints,
                                 arm_controller_joint_group::all(number_of_joints),
                                 time,
                                 stop,
                                 error);
}

void dvrk::arm_controller_manager::configure_move_jp_controller(const size_t number_of_joints,
                                                                const arm_controller_joint_group & joint_group,
                                                                std::function<double(void)> time,
                                                                std::function<void(bool)> stop,
                                                                std::function<void(void)> error)
{
    m_move_jp_controller->configure(number_of_joints,
                                    joint_group,
                                    time,
                                    stop,
                                    error);
}

void dvrk::arm_controller_manager::configure_move_jp_limits(const vctDoubleVec & v_max,
                                                            const vctDoubleVec & a_max,
                                                            const vctDoubleVec & goal_tolerance)
{
    m_move_jp_controller->configure_limits(v_max, a_max, goal_tolerance);
}

void dvrk::arm_controller_manager::configure_move_jp_period(const double period)
{
    m_move_jp_controller->configure_period(period);
}

void dvrk::arm_controller_manager::move_jp_set_ratio_v(const double ratio)
{
    m_move_jp_controller->set_ratio_v(ratio);
}

void dvrk::arm_controller_manager::move_jp_set_ratio_a(const double ratio)
{
    m_move_jp_controller->set_ratio_a(ratio);
}

void dvrk::arm_controller_manager::move_jp_set_ratio(const double ratio)
{
    m_move_jp_controller->set_ratio(ratio);
}

double dvrk::arm_controller_manager::move_jp_ratio_v(void) const
{
    return m_move_jp_controller->ratio_v();
}

double dvrk::arm_controller_manager::move_jp_ratio_a(void) const
{
    return m_move_jp_controller->ratio_a();
}

double dvrk::arm_controller_manager::move_jp_ratio(void) const
{
    return m_move_jp_controller->ratio();
}

bool dvrk::arm_controller_manager::move_jp_is_active(void) const
{
    return m_move_jp_controller->is_active();
}

const vctDoubleVec & dvrk::arm_controller_manager::move_jp_goal(void) const
{
    return m_move_jp_controller->goal();
}

const vctDoubleVec & dvrk::arm_controller_manager::move_jp_current_position(void) const
{
    return m_move_jp_controller->current_position();
}

const vctDoubleVec & dvrk::arm_controller_manager::move_jp_current_velocity(void) const
{
    return m_move_jp_controller->current_velocity();
}

void dvrk::arm_controller_manager::configure_servo_jp_controller(const size_t number_of_joints)
{
    configure_servo_jp_controller(arm_controller_joint_group::all(number_of_joints));
}

void dvrk::arm_controller_manager::configure_servo_jp_controller(const arm_controller_joint_group & joint_group)
{
    m_servo_jp_controller->configure(joint_group);
}

void dvrk::arm_controller_manager::configure_servo_cp_controller(const size_t number_of_joints,
                                                                 std::function<robManipulator::Errno(vctDoubleVec &, const vctFrm4x4 &)> inverse_kinematics,
                                                                 std::function<const vctFrm4x4 &(void)> base_frame,
                                                                 std::function<void(const std::string &)> error)
{
    configure_servo_cp_controller(number_of_joints,
                                  arm_controller_joint_group::all(number_of_joints),
                                  inverse_kinematics,
                                  base_frame,
                                  error);
}

void dvrk::arm_controller_manager::configure_servo_cp_controller(const size_t number_of_joints,
                                                                 const arm_controller_joint_group & joint_group,
                                                                 std::function<robManipulator::Errno(vctDoubleVec &, const vctFrm4x4 &)> inverse_kinematics,
                                                                 std::function<const vctFrm4x4 &(void)> base_frame,
                                                                 std::function<void(const std::string &)> error)
{
    m_servo_cp_controller->configure(number_of_joints,
                                     joint_group,
                                     inverse_kinematics,
                                     base_frame,
                                     error);
}

void dvrk::arm_controller_manager::configure_servo_cf_controller(const size_t number_of_joints,
                                                                 std::function<void(vctDoubleVec &, vctDoubleVec &)> preload,
                                                                 std::function<bool(const arm_state &, prmForceCartesianSet &)> cartesian_impedance,
                                                                 std::function<void(void)> orientation_lock)
{
    configure_servo_cf_controller(number_of_joints,
                                  arm_controller_joint_group::all(number_of_joints),
                                  preload,
                                  cartesian_impedance,
                                  orientation_lock);
}

void dvrk::arm_controller_manager::configure_servo_cf_controller(const size_t number_of_joints,
                                                                 const arm_controller_joint_group & joint_group,
                                                                 std::function<void(vctDoubleVec &, vctDoubleVec &)> preload,
                                                                 std::function<bool(const arm_state &, prmForceCartesianSet &)> cartesian_impedance,
                                                                 std::function<void(void)> orientation_lock)
{
    m_servo_cf_controller->configure(number_of_joints,
                                     joint_group,
                                     preload,
                                     cartesian_impedance,
                                     orientation_lock);
}

void dvrk::arm_controller_manager::move_jp(const vctDoubleVec & current_position,
                                           const vctDoubleVec & current_velocity,
                                           const vctDoubleVec & goal)
{
    m_move_jp_controller->start(current_position,
                                current_velocity,
                                goal);
    activate_controller(m_move_jp_controller.get());
}

dvrk::arm_controller_manager::trajectory_result dvrk::arm_controller_manager::move_jp_evaluate(arm_command_sink & command_sink)
{
    switch (m_move_jp_controller->evaluate(command_sink)) {
    case robReflexxes::Reflexxes_WORKING:
        return trajectory_result::working;
    case robReflexxes::Reflexxes_FINAL_STATE_REACHED:
        return trajectory_result::final_state_reached;
    default:
        return trajectory_result::error;
    }
}

double dvrk::arm_controller_manager::move_jp_end_time(void) const
{
    return m_move_jp_controller->end_time();
}

void dvrk::arm_controller_manager::move_jp_stop(const bool goal_reached)
{
    m_move_jp_controller->stop(goal_reached);
}

void dvrk::arm_controller_manager::servo_jp(const vctDoubleVec & jp,
                                            const vctDoubleVec & jv)
{
    m_servo_jp_controller->set_goal(jp, jv);
    activate_controller(m_servo_jp_controller.get());
}

void dvrk::arm_controller_manager::hold_position(const arm_state & state)
{
    vctDoubleVec zero_velocity(state.pid_setpoint_js.Position().size(), 0.0);
    servo_jp(state.pid_setpoint_js.Position(), zero_velocity);
}

void dvrk::arm_controller_manager::free_motion(const size_t number_of_joints,
                                               arm_command_sink & command_sink)
{
    m_active_controllers.clear();
    vctDoubleVec zero_effort(number_of_joints, 0.0);
    command_sink.servo_jf(zero_effort);
}

void dvrk::arm_controller_manager::servo_cs(const prmStateCartesian & cs)
{
    m_servo_cp_controller->set_goal(cs);
    activate_controller(m_servo_cp_controller.get());
}

void dvrk::arm_controller_manager::body_servo_cf(const prmForceCartesianSet & cf,
                                                 const bool body_orientation_absolute,
                                                 const bool orientation_locked)
{
    m_servo_cf_controller->set_goal(cf,
                                    servo_cf_controller::reference_frame::body,
                                    body_orientation_absolute,
                                    orientation_locked);
    activate_controller(m_servo_cf_controller.get());
}

void dvrk::arm_controller_manager::spatial_servo_cf(const prmForceCartesianSet & cf,
                                                    const bool orientation_locked)
{
    m_servo_cf_controller->set_goal(cf,
                                    servo_cf_controller::reference_frame::spatial,
                                    false,
                                    orientation_locked);
    activate_controller(m_servo_cf_controller.get());
}

void dvrk::arm_controller_manager::clear(void)
{
    m_legacy_controller.clear();
    m_active_controllers.clear();
}

bool dvrk::arm_controller_manager::has_active_controller(void) const
{
    return !m_active_controllers.empty();
}

void dvrk::arm_controller_manager::update(arm_state & state,
                                          arm_command_sink & command_sink)
{
    const std::vector<arm_controller *> active_controllers = m_active_controllers;
    for (auto controller : active_controllers) {
        if (std::find(m_active_controllers.begin(), m_active_controllers.end(), controller) != m_active_controllers.end()) {
            controller->update(state, command_sink);
        }
    }
    for (auto controller : m_active_controllers) {
        if (std::find(active_controllers.begin(), active_controllers.end(), controller) == active_controllers.end()) {
            controller->update(state, command_sink);
        }
    }
}

void dvrk::arm_controller_manager::activate_controller(arm_controller * controller)
{
    if (!controller) {
        return;
    }

    if (controller == &m_legacy_controller) {
        m_active_controllers.clear();
        m_active_controllers.push_back(controller);
        return;
    }

    remove_controller(&m_legacy_controller);
    stop_conflicting_controllers(*controller);
    if (std::find(m_active_controllers.begin(), m_active_controllers.end(), controller) == m_active_controllers.end()) {
        m_active_controllers.push_back(controller);
    }
}

void dvrk::arm_controller_manager::remove_controller(arm_controller * controller)
{
    m_active_controllers.erase(std::remove(m_active_controllers.begin(),
                                           m_active_controllers.end(),
                                           controller),
                               m_active_controllers.end());
}

void dvrk::arm_controller_manager::stop_conflicting_controllers(const arm_controller & controller)
{
    auto iterator = m_active_controllers.begin();
    while (iterator != m_active_controllers.end()) {
        arm_controller * active_controller = *iterator;
        if ((active_controller != &controller)
            && active_controller->claims().conflicts_with(controller.claims())) {
            active_controller->stop(false);
            iterator = m_active_controllers.erase(iterator);
        } else {
            ++iterator;
        }
    }
}
