#ifndef __SWEEP_PARAMETERS__H
#define __SWEEP_PARAMETERS__H

class SweepParameters {

public:

    SweepParameters();
    SweepParameters(int pan_start, int pan_step, int pan_end,
                    int tilt_start, int tilt_step, int tilt_end);

    ~SweepParameters();


    int                                              m_pan_start, m_pan_step, m_pan_end;
    int                                              m_tilt_start, m_tilt_step, m_tilt_end;
};


#endif
