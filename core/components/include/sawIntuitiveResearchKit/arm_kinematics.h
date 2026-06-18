/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

#ifndef _arm_kinematics_h
#define _arm_kinematics_h

#include <memory>

#include <cisstNumerical/nmrPInverse.h>
#include <cisstRobot/robManipulator.h>

#include <sawIntuitiveResearchKit/arm_state.h>

// Always include last
#include <sawIntuitiveResearchKit/sawIntuitiveResearchKitExport.h>

namespace dvrk {

    class CISST_EXPORT arm_kinematics {
    public:
        arm_kinematics();
        ~arm_kinematics();

        bool has_manipulator(void) const;
        robManipulator & manipulator(void);
        robManipulator & manipulator(void) const;
        robManipulator * manipulator_pointer(void);
        robManipulator * manipulator_pointer(void) const;
        void set_manipulator(std::unique_ptr<robManipulator> manipulator);

        vctFrm4x4 & base_frame(void);
        const vctFrm4x4 & base_frame(void) const;
        void set_base_frame(const vctFrm4x4 & base_frame);
        bool base_frame_valid(void) const;
        void set_base_frame_valid(const bool valid);

        nmrPInverseDynamicData & jacobian_pinverse_data(void);
        nmrPInverseDynamicData & jacobian_transpose_pinverse_data(void);

        void update_cartesian_state(arm_state & state,
                                    const bool body_cf_orientation_absolute);
        void invalidate_cartesian_state(arm_state & state);

    private:
        std::unique_ptr<robManipulator> m_manipulator;
        vctFrm4x4 m_base_frame;
        bool m_base_frame_valid = false;
        nmrPInverseDynamicData m_jacobian_pinverse_data;
        nmrPInverseDynamicData m_jacobian_transpose_pinverse_data;
    };
}

#endif // _arm_kinematics_h
