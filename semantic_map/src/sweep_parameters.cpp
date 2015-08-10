#include "semantic_map/sweep_parameters.h"


SweepParameters::SweepParameters()
{
    m_pan_start = -160; m_pan_step = 20; m_pan_end = 160; // 17 horizontal steps
    m_tilt_start = -30; m_tilt_step = 30; m_tilt_end = 30; // 3 vertical steps
}

SweepParameters::SweepParameters(int pan_start, int pan_step, int pan_end,
                int tilt_start, int tilt_step, int tilt_end)
{
    m_pan_start = pan_start;
    m_pan_step = pan_step;
    m_pan_end = pan_end;
    m_tilt_start = tilt_start;
    m_tilt_step = tilt_step;
    m_tilt_end = tilt_end;
}

SweepParameters::~SweepParameters()
{

}
