/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _move_jp_controller_h
#define _move_jp_controller_h

#include <functional>

#include <cisstRobot/robReflexxes.h>
#include <cisstVector/vctDynamicVectorTypes.h>

#include <sawIntuitiveResearchKit/arm_controller_manager.h>
#include <sawIntuitiveResearchKit/arm_controller_resource.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {

    class CISST_EXPORT move_jp_controller: public arm_controller {
    public:
        typedef std::function<void(bool)> stop_function;
        typedef std::function<void(void)> error_function;
        typedef std::function<double(void)> time_function;

        move_jp_controller();
        ~move_jp_controller() override;

        void configure(const size_t number_of_joints,
                       time_function time,
                       stop_function stop,
                       error_function error);
        void configure(const size_t number_of_joints,
                       const arm_controller_joint_group & joint_group,
                       time_function time,
                       stop_function stop,
                       error_function error);
        void configure_limits(const vctDoubleVec & v_max,
                              const vctDoubleVec & a_max,
                              const vctDoubleVec & goal_tolerance);
        void configure_period(const double period);
        void set_ratio_v(const double ratio);
        void set_ratio_a(const double ratio);
        void set_ratio(const double ratio);
        double ratio_v(void) const;
        double ratio_a(void) const;
        double ratio(void) const;
        bool is_active(void) const;
        const vctDoubleVec & goal(void) const;
        const vctDoubleVec & current_position(void) const;
        const vctDoubleVec & current_velocity(void) const;
        void start(const vctDoubleVec & current_position,
                   const vctDoubleVec & current_velocity,
                   const vctDoubleVec & goal);
        void stop(const bool goal_reached);
        robReflexxes::ResultType evaluate(arm_command_sink & command_sink);
        double end_time(void) const;
        const arm_controller_resource_claim & claims(void) const override;
        void update(arm_state & state,
                    arm_command_sink & command_sink) override;

    private:
        stop_function m_stop;
        error_function m_error;
        time_function m_time;
        robReflexxes m_reflexxes;
        vctDoubleVec m_v_max;
        vctDoubleVec m_v;
        double m_ratio_v;
        vctDoubleVec m_a_max;
        vctDoubleVec m_a;
        double m_ratio_a;
        double m_ratio;
        vctDoubleVec m_goal;
        vctDoubleVec m_goal_v;
        vctDoubleVec m_goal_error;
        vctDoubleVec m_goal_tolerance;
        vctDoubleVec m_jp;
        vctDoubleVec m_jv;
        bool m_is_active = false;
        double m_end_time = 0.0;
        double m_period = 0.0;
        arm_controller_resource_claim m_claims;

        void update_ratio(void);
        void update_reflexxes(void);
    };
}

#endif // _move_jp_controller_h
