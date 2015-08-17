#ifndef __SWEEP_PARAMETERS__H
#define __SWEEP_PARAMETERS__H

#include <iostream>

class SweepParameters {

public:

    SweepParameters(bool reverse_motion = true);
    SweepParameters(int pan_start, int pan_step, int pan_end,
                    int tilt_start, int tilt_step, int tilt_end,
                    bool reverse_motion=true);

    ~SweepParameters();

    int getNumberOfIntermediatePositions();
    void getIntermediatePosition(int pan_angle, int tilt_angle, int& position_number);
    void getAnglesForPosition(int& pan_angle, int& tilt_angle, int position_number);
    void findCorrespondingPosition(const SweepParameters other, int my_pan_angle, int my_tilt_angle, int& corresponding_position);
    void findCorrespondingPosition(const SweepParameters other, int my_position_number, int& corresponding_position);
    bool operator==(const SweepParameters& rhs);
    bool operator!=(const SweepParameters& rhs);
    SweepParameters& operator=(const SweepParameters& rhs);

    friend std::ostream& operator<<(std::ostream& os, const SweepParameters& sweep){
        os<<sweep.m_pan_start<<" "<<sweep.m_pan_step<<" "<<sweep.m_pan_end<<" "<<sweep.m_tilt_start<<" "<<sweep.m_tilt_step<<" "<<sweep.m_tilt_end;
        return os;
    }



    int                                              m_pan_start, m_pan_step, m_pan_end;
    int                                              m_tilt_start, m_tilt_step, m_tilt_end;
    bool                                             m_reverseMotionAlternateSweepLevel;
};


#endif
