/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <sawIntuitiveResearchKit/arm_command_sink.h>
#include <sawIntuitiveResearchKit/mtsIntuitiveResearchKitArm.h>

dvrk::arm_command_sink::arm_command_sink(mtsIntuitiveResearchKitArm & arm):
    m_arm(arm)
{
}

dvrk::arm_command_sink::~arm_command_sink()
{
}

void dvrk::arm_command_sink::servo_jp(const vctDoubleVec & jp,
                                      const vctDoubleVec & jv)
{
    m_arm.servo_jp_internal(jp, jv);
}

void dvrk::arm_command_sink::servo_jf(const vctDoubleVec & jf)
{
    m_arm.servo_jf_internal(jf);
}

void dvrk::arm_command_sink::servo_js(const prmStateJoint & js)
{
    m_arm.servo_js_internal(js);
}

void dvrk::arm_command_sink::feed_forward_jf(const vctDoubleVec & jf)
{
    m_arm.feed_forward_jf_internal(jf);
}

void dvrk::arm_command_sink::enable_torque_mode(const vctBoolVec & enabled)
{
    m_arm.PID.EnableTorqueMode(enabled);
}

void dvrk::arm_command_sink::enable_measured_setpoint_check(const bool enabled)
{
    m_arm.PID.enable_measured_setpoint_check(enabled);
}

void dvrk::arm_command_sink::enforce_position_limits(const bool enabled)
{
    m_arm.PID.enforce_position_limits(enabled);
}
