#include "semantic_map/sweep_parameters.h"


SweepParameters::SweepParameters(bool reverse_motion) : m_reverseMotionAlternateSweepLevel(reverse_motion)
{
    m_pan_start = -160; m_pan_step = 20; m_pan_end = 160; // 17 horizontal steps
    m_tilt_start = -30; m_tilt_step = 30; m_tilt_end = 30; // 3 vertical steps
}

SweepParameters::SweepParameters(int pan_start, int pan_step, int pan_end,
                int tilt_start, int tilt_step, int tilt_end,
                bool reverse_motion) : m_reverseMotionAlternateSweepLevel(reverse_motion)
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

int SweepParameters::getNumberOfIntermediatePositions()
{
    int pan_steps = (m_pan_end - m_pan_start)/m_pan_step + 1;
    int tilt_steps = (m_tilt_end - m_tilt_start)/m_tilt_step + 1;

    return pan_steps * tilt_steps;
}

void SweepParameters::getIntermediatePosition(int pan_angle, int tilt_angle, int& position_number)
{
    position_number = -1;
    // first find tilt match
    int tilt_match = 0;
    bool found_match = false;
    for (int i = m_tilt_start; i<=m_tilt_end; i+=m_tilt_step){
        if (i == tilt_angle){
            found_match = true;
            break;
        } else {
            tilt_match++;
        }
    }
    if (!found_match) { return;}

    bool reverse_pan;
    /* Sweep pattern is like this
     *  ......................................
     *  Start_level_3 -> - - - -    end_lvl_e |
     *  | end_lvl_2    ^ - - - <-   Start_lvl_2
     *  Start_level_1 -> - - - ^    end_lvl_1 |
     *  .......................................
     *  Every second level the order of traversing the pan angles has to be swapped
     */

    if (tilt_match%2 == 0){ // every second sweep is end to beginning
        reverse_pan = false;
    } else {
        reverse_pan = true;
    }

    found_match = false;
    int pan_match = 0;
    if (reverse_pan){
        for (int i=m_pan_end; i>=m_pan_start; i-=m_pan_step){
            if (i==pan_angle){
                found_match = true;
                break;
            } else{
                pan_match++;
            }
        }
    } else{
        for (int i=m_pan_start; i<=m_pan_end; i+=m_pan_step){
            if (i==pan_angle){
                found_match = true;
                break;
            } else{
                pan_match++;
            }
        }
    }

    if (!found_match) { return ;}

    int pan_steps = (m_pan_end - m_pan_start)/m_pan_step + 1;
    position_number = tilt_match * pan_steps + pan_match;
}

void SweepParameters::getAnglesForPosition(int& pan_angle, int& tilt_angle, int position_number)
{
    int pan_steps = (m_pan_end - m_pan_start)/m_pan_step + 1;
    int tilt_match = position_number/pan_steps;

    tilt_angle = m_tilt_start;
    for (int i = 0; i<tilt_match; i++){
        tilt_angle += m_tilt_step;
    }

    int pan_match = position_number%pan_steps;

    bool reverse_pan;
    /* Sweep pattern is like this
     *  ......................................
     *  Start_level_3 -> - - - -    end_lvl_e |
     *  | end_lvl_2    ^ - - - <-   Start_lvl_2
     *  Start_level_1 -> - - - ^    end_lvl_1 |
     *  .......................................
     *  Every second level the order of traversing the pan angles has to be swapped
     */
    if (tilt_match%2 == 0){ // every second sweep is end to beginning
        reverse_pan = false;
    } else {
        reverse_pan = true;
    }


    if (reverse_pan){
        pan_angle = m_pan_end;
        for (int i=0; i<pan_match; i++){
            pan_angle -= m_pan_step;
        }
    } else {
        pan_angle = m_pan_start;
        for (int i=0; i<pan_match; i++){
            pan_angle += m_pan_step;
        }
    }
}
