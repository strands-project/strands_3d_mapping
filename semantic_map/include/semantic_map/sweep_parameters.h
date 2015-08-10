#ifndef __SWEEP_PARAMETERS__H
#define __SWEEP_PARAMETERS__H

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


    int                                              m_pan_start, m_pan_step, m_pan_end;
    int                                              m_tilt_start, m_tilt_step, m_tilt_end;
    bool                                             m_reverseMotionAlternateSweepLevel;
};


#endif
