/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <sawIntuitiveResearchKit/arm_command_sink.h>
#include <sawIntuitiveResearchKit/servo_jp_controller.h>

dvrk::servo_jp_controller::servo_jp_controller():
    arm_controller("servo_jp")
{
}

dvrk::servo_jp_controller::~servo_jp_controller()
{
}

void dvrk::servo_jp_controller::configure(const size_t number_of_joints)
{
    configure(arm_controller_joint_group::all(number_of_joints));
}

void dvrk::servo_jp_controller::configure(const arm_controller_joint_group & joint_group)
{
    m_claims.clear();
    m_claims.claim_joint_position(joint_group);
}

void dvrk::servo_jp_controller::set_goal(const vctDoubleVec & jp,
                                         const vctDoubleVec & jv)
{
    m_jp.Assign(jp);
    m_jv.Assign(jv);
    m_has_pending_goal = true;
}

bool dvrk::servo_jp_controller::has_pending_goal(void) const
{
    return m_has_pending_goal;
}

const dvrk::arm_controller_resource_claim & dvrk::servo_jp_controller::claims(void) const
{
    return m_claims;
}

void dvrk::servo_jp_controller::update(arm_state &,
                                       arm_command_sink & command_sink)
{
    if (m_has_pending_goal) {
        command_sink.servo_jp(m_jp, m_jv);
        m_has_pending_goal = false;
    }
}
