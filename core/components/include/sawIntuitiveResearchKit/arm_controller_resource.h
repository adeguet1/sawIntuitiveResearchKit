/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _arm_controller_resource_h
#define _arm_controller_resource_h

#include <set>
#include <string>
#include <vector>

#include <cisstVector/vctDynamicVectorTypes.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {

    class CISST_EXPORT arm_controller_joint_group {
    public:
        arm_controller_joint_group();
        arm_controller_joint_group(const std::string & name,
                                   const std::vector<size_t> & joints);

        static arm_controller_joint_group all(const size_t number_of_joints);
        static arm_controller_joint_group range(const std::string & name,
                                                const size_t start,
                                                const size_t count);
        static arm_controller_joint_group single(const std::string & name,
                                                 const size_t joint_index);

        const std::string & name(void) const;
        const std::vector<size_t> & joints(void) const;
        bool empty(void) const;

    private:
        std::string m_name;
        std::vector<size_t> m_joints;
    };

    class CISST_EXPORT arm_controller_resource_claim {
    public:
        void clear(void);
        void claim_joint_position(const size_t joint_index);
        void claim_joint_position(const arm_controller_joint_group & joint_group);
        void claim_joint_effort(const size_t joint_index);
        void claim_joint_effort(const arm_controller_joint_group & joint_group);
        void claim_cartesian_position(void);
        void claim_cartesian_orientation(void);
        void claim_cartesian_pose(void);
        void claim_cartesian_force(void);
        void claim_cartesian_torque(void);
        void claim_cartesian_wrench(void);
        bool conflicts_with(const arm_controller_resource_claim & other) const;
        const std::set<std::string> & resources(void) const;

    private:
        std::set<std::string> m_resources;
    };
}

#endif // _arm_controller_resource_h
