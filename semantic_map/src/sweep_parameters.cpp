#include "semantic_map/sweep_parameters.h"

using namespace std;

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
    int pan_steps =1,tilt_steps =1;

    if (m_pan_step != 0){
        pan_steps = (m_pan_end - m_pan_start)/m_pan_step + 1;
    }

    if (m_tilt_step != 0){
        tilt_steps = (m_tilt_end - m_tilt_start)/m_tilt_step + 1;
    }

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

void SweepParameters::findCorrespondingPosition(SweepParameters other, int my_pan_angle, int my_tilt_angle, int& corresponding_position)
{
    other.getIntermediatePosition(my_pan_angle, my_tilt_angle, corresponding_position);
}

void SweepParameters::findCorrespondingPosition(SweepParameters other, int my_position_number, int& corresponding_position)
{
    int my_pan_angle, my_tilt_angle;
    this->getAnglesForPosition(my_pan_angle, my_tilt_angle, my_position_number);
    this->findCorrespondingPosition(other, my_pan_angle, my_tilt_angle, corresponding_position);
}

bool SweepParameters::operator==(const SweepParameters& rhs)
{
    if (m_pan_start != rhs.m_pan_start){
        return false;
    }
    if (m_pan_step != rhs.m_pan_step){
        return false;
    }
    if (m_pan_end != rhs.m_pan_end){
        return false;
    }

    if (m_tilt_start != rhs.m_tilt_start){
        return false;
    }
    if (m_tilt_step != rhs.m_tilt_step){
        return false;
    }
    if (m_tilt_end != rhs.m_tilt_end){
        return false;
    }

    return true;
}

bool SweepParameters::operator!=(const SweepParameters& rhs)
{
    return !(*this == rhs);
}

SweepParameters& SweepParameters::operator=(const SweepParameters& rhs)
{
    m_pan_start = rhs.m_pan_start;
    m_pan_step = rhs.m_pan_step;
    m_pan_end = rhs.m_pan_end;

    m_tilt_start = rhs.m_tilt_start;
    m_tilt_step = rhs.m_tilt_step;
    m_tilt_end = rhs.m_tilt_end;
}

//-------------- MAKE THIS INTO A UNIT TEST
//        // test
//        int pan_angle, tilt_angle;
//        pan_angle = 0; tilt_angle = 0;
//        int int_pos; sweepParams.getIntermediatePosition(pan_angle, tilt_angle, int_pos);
//        ROS_INFO_STREAM("Pan angle "<<pan_angle<<" tilt angle "<<tilt_angle<<"  intermediate position  "<<int_pos);
//        sweepParams.getAnglesForPosition(pan_angle, tilt_angle, int_pos);
//        ROS_INFO_STREAM("Pan angle "<<pan_angle<<" tilt angle "<<tilt_angle<<"  intermediate position  "<<int_pos);

//        pan_angle = -160; tilt_angle = 0;
//        sweepParams.getIntermediatePosition(pan_angle, tilt_angle, int_pos);
//        ROS_INFO_STREAM("Pan angle "<<pan_angle<<" tilt angle "<<tilt_angle<<"  intermediate position  "<<int_pos);
//        sweepParams.getAnglesForPosition(pan_angle, tilt_angle, int_pos);
//        ROS_INFO_STREAM("Pan angle "<<pan_angle<<" tilt angle "<<tilt_angle<<"  intermediate position  "<<int_pos);

//        pan_angle = 160; tilt_angle = 0;
//        sweepParams.getIntermediatePosition(pan_angle, tilt_angle, int_pos);
//        ROS_INFO_STREAM("Pan angle "<<pan_angle<<" tilt angle "<<tilt_angle<<"  intermediate position  "<<int_pos);
//        sweepParams.getAnglesForPosition(pan_angle, tilt_angle, int_pos);
//        ROS_INFO_STREAM("Pan angle "<<pan_angle<<" tilt angle "<<tilt_angle<<"  intermediate position  "<<int_pos);

//        pan_angle = -160; tilt_angle = 30;
//        sweepParams.getIntermediatePosition(pan_angle, tilt_angle, int_pos);
//        ROS_INFO_STREAM("Pan angle "<<pan_angle<<" tilt angle "<<tilt_angle<<"  intermediate position  "<<int_pos);
//        sweepParams.getAnglesForPosition(pan_angle, tilt_angle, int_pos);
//        ROS_INFO_STREAM("Pan angle "<<pan_angle<<" tilt angle "<<tilt_angle<<"  intermediate position  "<<int_pos);

//        pan_angle = 160; tilt_angle = 30;
//        sweepParams.getIntermediatePosition(pan_angle, tilt_angle, int_pos);
//        ROS_INFO_STREAM("Pan angle "<<pan_angle<<" tilt angle "<<tilt_angle<<"  intermediate position  "<<int_pos);
//        sweepParams.getAnglesForPosition(pan_angle, tilt_angle, int_pos);
//        ROS_INFO_STREAM("Pan angle "<<pan_angle<<" tilt angle "<<tilt_angle<<"  intermediate position  "<<int_pos);

//        pan_angle = 80; tilt_angle = 30;
//        sweepParams.getIntermediatePosition(pan_angle, tilt_angle, int_pos);
//        ROS_INFO_STREAM("Pan angle "<<pan_angle<<" tilt angle "<<tilt_angle<<"  intermediate position  "<<int_pos);
//        sweepParams.getAnglesForPosition(pan_angle, tilt_angle, int_pos);
//        ROS_INFO_STREAM("Pan angle "<<pan_angle<<" tilt angle "<<tilt_angle<<"  intermediate position  "<<int_pos);


//        // more test
//        SweepParameters test_params(-160,20,160,-30,30,30);
//        int test_pos = test_params.getNumberOfIntermediatePositions();
//        for (int k=0; k<test_pos; k++){
//            int corresp_pos;
//            test_params.findCorrespondingPosition(sweepParams,k, corresp_pos);
//            int my_pan, my_tilt, their_pan, their_tilt;
//            test_params.getAnglesForPosition(my_pan, my_tilt, k);
//            sweepParams.getAnglesForPosition(their_pan, their_tilt, corresp_pos);

//            ROS_INFO_STREAM("My pos "<<k<<" my pan "<<my_pan<<" my tilt "<<my_tilt<<" corresp_pos "<<corresp_pos<<" their_pan "<<their_pan<<" their_tilt "<<their_tilt);
//        }
