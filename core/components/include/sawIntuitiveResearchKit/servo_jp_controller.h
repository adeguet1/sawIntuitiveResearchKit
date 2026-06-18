/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _servo_jp_controller_h
#define _servo_jp_controller_h

#include <sawIntuitiveResearchKit/arm_controller_manager.h>
#include <sawIntuitiveResearchKit/arm_controller_resource.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {

    class CISST_EXPORT servo_jp_controller: public arm_controller {
    public:
        servo_jp_controller();
        ~servo_jp_controller() override;

        void configure(const size_t number_of_joints);
        void configure(const arm_controller_joint_group & joint_group);
        void set_goal(const vctDoubleVec & jp,
                      const vctDoubleVec & jv);
        bool has_pending_goal(void) const;
        const arm_controller_resource_claim & claims(void) const override;
        void update(arm_state & state,
                    arm_command_sink & command_sink) override;

    private:
        bool m_has_pending_goal = false;
        vctDoubleVec m_jp;
        vctDoubleVec m_jv;
        arm_controller_resource_claim m_claims;
    };
}

#endif // _servo_jp_controller_h
