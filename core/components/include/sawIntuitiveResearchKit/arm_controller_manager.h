/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _arm_controller_manager_h
#define _arm_controller_manager_h

#include <functional>
#include <vector>
#include <memory>
#include <string>

#include <cisstMultiTask/mtsCallableVoidBase.h>
#include <cisstParameterTypes/prmForceCartesianSet.h>
#include <cisstParameterTypes/prmStateCartesian.h>
#include <cisstRobot/robManipulator.h>
#include <cisstVector/vctDynamicVectorTypes.h>

#include <sawIntuitiveResearchKit/arm_controller_resource.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {
    class arm_command_sink;
    class arm_state;
    class move_jp_controller;
    class servo_cf_controller;
    class servo_cp_controller;
    class servo_jp_controller;

    class CISST_EXPORT arm_controller {
    public:
        inline arm_controller(const std::string & name):
            m_name(name)
        {}

        virtual ~arm_controller() {}

        inline const std::string & name(void) const {
            return m_name;
        }

        virtual void update(arm_state & state,
                            arm_command_sink & command_sink) = 0;
        virtual void stop(const bool) {}
        virtual const arm_controller_resource_claim & claims(void) const = 0;

    private:
        std::string m_name;
    };

    class CISST_EXPORT legacy_callback_controller: public arm_controller {
    public:
        legacy_callback_controller();
        ~legacy_callback_controller() override;

        void set_callback(mtsCallableVoidBase * callback);
        void clear(void);
        bool has_callback(void) const;
        const arm_controller_resource_claim & claims(void) const override;
        void update(arm_state & state,
                    arm_command_sink & command_sink) override;

    private:
        mtsCallableVoidBase * m_callback = 0;
        arm_controller_resource_claim m_claims;
    };

    class CISST_EXPORT arm_controller_manager {
    public:
        enum class trajectory_result { working, final_state_reached, error };

        arm_controller_manager();
        ~arm_controller_manager();

        void set_legacy_callback(mtsCallableVoidBase * callback);
        void configure_move_jp_controller(const size_t number_of_joints,
                                          std::function<double(void)> time,
                                          std::function<void(bool)> stop,
                                          std::function<void(void)> error);
        void configure_move_jp_controller(const size_t number_of_joints,
                                          const arm_controller_joint_group & joint_group,
                                          std::function<double(void)> time,
                                          std::function<void(bool)> stop,
                                          std::function<void(void)> error);
        void configure_move_jp_limits(const vctDoubleVec & v_max,
                                      const vctDoubleVec & a_max,
                                      const vctDoubleVec & goal_tolerance);
        void configure_move_jp_period(const double period);
        void move_jp_set_ratio_v(const double ratio);
        void move_jp_set_ratio_a(const double ratio);
        void move_jp_set_ratio(const double ratio);
        double move_jp_ratio_v(void) const;
        double move_jp_ratio_a(void) const;
        double move_jp_ratio(void) const;
        bool move_jp_is_active(void) const;
        const vctDoubleVec & move_jp_goal(void) const;
        const vctDoubleVec & move_jp_current_position(void) const;
        const vctDoubleVec & move_jp_current_velocity(void) const;
        void configure_servo_jp_controller(const size_t number_of_joints);
        void configure_servo_jp_controller(const arm_controller_joint_group & joint_group);
        void configure_servo_cp_controller(const size_t number_of_joints,
                                           std::function<robManipulator::Errno(vctDoubleVec &, const vctFrm4x4 &)> inverse_kinematics,
                                           std::function<const vctFrm4x4 &(void)> base_frame,
                                           std::function<void(const std::string &)> error);
        void configure_servo_cp_controller(const size_t number_of_joints,
                                           const arm_controller_joint_group & joint_group,
                                           std::function<robManipulator::Errno(vctDoubleVec &, const vctFrm4x4 &)> inverse_kinematics,
                                           std::function<const vctFrm4x4 &(void)> base_frame,
                                           std::function<void(const std::string &)> error);
        void configure_servo_cf_controller(const size_t number_of_joints,
                                           std::function<void(vctDoubleVec &, vctDoubleVec &)> preload,
                                           std::function<bool(const arm_state &, prmForceCartesianSet &)> cartesian_impedance,
                                           std::function<void(void)> orientation_lock);
        void configure_servo_cf_controller(const size_t number_of_joints,
                                           const arm_controller_joint_group & joint_group,
                                           std::function<void(vctDoubleVec &, vctDoubleVec &)> preload,
                                           std::function<bool(const arm_state &, prmForceCartesianSet &)> cartesian_impedance,
                                           std::function<void(void)> orientation_lock);
        void move_jp(const vctDoubleVec & current_position,
                     const vctDoubleVec & current_velocity,
                     const vctDoubleVec & goal);
        trajectory_result move_jp_evaluate(arm_command_sink & command_sink);
        double move_jp_end_time(void) const;
        void move_jp_stop(const bool goal_reached);
        void servo_jp(const vctDoubleVec & jp,
                      const vctDoubleVec & jv);
        void hold_position(const arm_state & state);
        void free_motion(const size_t number_of_joints,
                         arm_command_sink & command_sink);
        void servo_cs(const prmStateCartesian & cs);
        void body_servo_cf(const prmForceCartesianSet & cf,
                           const bool body_orientation_absolute,
                           const bool orientation_locked);
        void spatial_servo_cf(const prmForceCartesianSet & cf,
                              const bool orientation_locked);
        void clear(void);
        bool has_active_controller(void) const;
        void update(arm_state & state,
                    arm_command_sink & command_sink);

    private:
        void activate_controller(arm_controller * controller);
        void remove_controller(arm_controller * controller);
        void stop_conflicting_controllers(const arm_controller & controller);

        legacy_callback_controller m_legacy_controller;
        std::unique_ptr<move_jp_controller> m_move_jp_controller;
        std::unique_ptr<servo_cf_controller> m_servo_cf_controller;
        std::unique_ptr<servo_cp_controller> m_servo_cp_controller;
        std::unique_ptr<servo_jp_controller> m_servo_jp_controller;
        std::vector<arm_controller *> m_active_controllers;
    };
}

#endif // _arm_controller_manager_h
