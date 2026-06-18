/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#include <sstream>

#include <sawIntuitiveResearchKit/arm_controller_resource.h>

dvrk::arm_controller_joint_group::arm_controller_joint_group()
{
}

dvrk::arm_controller_joint_group::arm_controller_joint_group(const std::string & name,
                                                             const std::vector<size_t> & joints):
    m_name(name),
    m_joints(joints)
{
}

dvrk::arm_controller_joint_group dvrk::arm_controller_joint_group::all(const size_t number_of_joints)
{
    std::vector<size_t> joints(number_of_joints);
    for (size_t index = 0; index < number_of_joints; ++index) {
        joints[index] = index;
    }
    return arm_controller_joint_group("all", joints);
}

dvrk::arm_controller_joint_group dvrk::arm_controller_joint_group::range(const std::string & name,
                                                                         const size_t start,
                                                                         const size_t count)
{
    std::vector<size_t> joints(count);
    for (size_t index = 0; index < count; ++index) {
        joints[index] = start + index;
    }
    return arm_controller_joint_group(name, joints);
}

dvrk::arm_controller_joint_group dvrk::arm_controller_joint_group::single(const std::string & name,
                                                                          const size_t joint_index)
{
    return arm_controller_joint_group(name, std::vector<size_t>(1, joint_index));
}

const std::string & dvrk::arm_controller_joint_group::name(void) const
{
    return m_name;
}

const std::vector<size_t> & dvrk::arm_controller_joint_group::joints(void) const
{
    return m_joints;
}

bool dvrk::arm_controller_joint_group::empty(void) const
{
    return m_joints.empty();
}

void dvrk::arm_controller_resource_claim::clear(void)
{
    m_resources.clear();
}

void dvrk::arm_controller_resource_claim::claim_joint_position(const size_t joint_index)
{
    std::stringstream resource;
    resource << "joint/" << joint_index << "/position";
    m_resources.insert(resource.str());
}

void dvrk::arm_controller_resource_claim::claim_joint_position(const arm_controller_joint_group & joint_group)
{
    for (const auto joint_index : joint_group.joints()) {
        claim_joint_position(joint_index);
    }
}

void dvrk::arm_controller_resource_claim::claim_joint_effort(const size_t joint_index)
{
    std::stringstream resource;
    resource << "joint/" << joint_index << "/effort";
    m_resources.insert(resource.str());
}

void dvrk::arm_controller_resource_claim::claim_joint_effort(const arm_controller_joint_group & joint_group)
{
    for (const auto joint_index : joint_group.joints()) {
        claim_joint_effort(joint_index);
    }
}

void dvrk::arm_controller_resource_claim::claim_cartesian_position(void)
{
    m_resources.insert("cartesian/position");
}

void dvrk::arm_controller_resource_claim::claim_cartesian_orientation(void)
{
    m_resources.insert("cartesian/orientation");
}

void dvrk::arm_controller_resource_claim::claim_cartesian_pose(void)
{
    claim_cartesian_position();
    claim_cartesian_orientation();
}

void dvrk::arm_controller_resource_claim::claim_cartesian_force(void)
{
    m_resources.insert("cartesian/force");
}

void dvrk::arm_controller_resource_claim::claim_cartesian_torque(void)
{
    m_resources.insert("cartesian/torque");
}

void dvrk::arm_controller_resource_claim::claim_cartesian_wrench(void)
{
    claim_cartesian_force();
    claim_cartesian_torque();
}

bool dvrk::arm_controller_resource_claim::conflicts_with(const arm_controller_resource_claim & other) const
{
    for (const auto & resource : m_resources) {
        if (other.m_resources.find(resource) != other.m_resources.end()) {
            return true;
        }
    }
    return false;
}

const std::set<std::string> & dvrk::arm_controller_resource_claim::resources(void) const
{
    return m_resources;
}
