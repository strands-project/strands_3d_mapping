//
// Created by chris on 13/01/16.
//
#include <iostream>
#include <vector>
#include <cmath>


template<typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
    out << "[";
    size_t last = v.size() - 1;
    for(size_t i = 0; i < v.size(); ++i) {
        out << v[i];
        if (i != last)
            out << ", ";
    }
    out << "]";
    return out;
}

const double a = 0.93;

int main(int argc, char **argv) {
    std::cout << "Testing something.\n";

    // Vector of cell p(o) values after volume
    std::vector<double> current_map;
    current_map.resize(25,0.5);
    current_map[10]=0.75;
//    for (unsigned i=11;i<25;i++)
//        current_map[i]=0.15;


    // Vector of post-measurement p(x) values
//    std::vector<double> updated_p_x;
//    updated_p_x.resize(25,0);
//    updated_p_x[0]=1;

    // Vector of post-measurement p(o) values
//    std::vector<double> updated_p_o;
//    updated_p_o.resize(25,0);

    // Transition matrix for the visibility state
    // Eqn (9)
    // Row major
    std::vector<double> p_x_plus_1_given_x;
    p_x_plus_1_given_x.resize(4,0);
    p_x_plus_1_given_x[2]=0.1;
    p_x_plus_1_given_x[3]=0.9;

    // Transition matrix for the ocupancy state
    // Eqn (14)
    // Row major
    std::vector<double> p_o_m_given_o_m_minus_1;
    p_o_m_given_o_m_minus_1.resize(4,0);
    p_x_plus_1_given_x[2]=0.000001;
    p_x_plus_1_given_x[3]=0.999999;

    double prev_p_x=1;
    double prev_p_o=0;
    for (unsigned i=1; i<current_map.size(); i++) {
//        double prev_p_x = updated_p_x[i-1];
//        double prev_p_o = updated_p_o[i-1];
        double p_o = current_map[i];

        // Update transition matrix p_x_plus_1_given_x : Eqn (9)
        int number_of_cells_passed = i;
        p_x_plus_1_given_x[0] = std::pow(a, number_of_cells_passed);
        p_x_plus_1_given_x[1] = 1 - p_x_plus_1_given_x[0];
        //std::cout << "Updated transition matrix :" << p_x_plus_1_given_x << std::endl;

        // Use transition matrix to predict p(x_m|o_{o:m-2}) : Equation(5)
        // Don't do full matrix multiple, only look at P(occupied) ignore p(not_occupied)
        double next_p_x = p_x_plus_1_given_x[0]*prev_p_x + p_x_plus_1_given_x[2]*(1-prev_p_x);

        // Do an update on the predicted next value : Eqn (6) & (7)
        double p_o_m_minus_1_given_x_m = 1 - current_map[i-1];
        next_p_x = p_o_m_minus_1_given_x_m*next_p_x / (p_o_m_minus_1_given_x_m*next_p_x +
                                                       (1-p_o_m_minus_1_given_x_m)*(1-next_p_x));

        // Clamp the value. This is ugly and not in the paper but seams necessary...
        //if (next_p_x > prev_p_x)
        //    next_p_x = prev_p_x;

        prev_p_x=next_p_x;

        // Update transition matrix p_o_m_given_o_m_minus_1 : Eqn (14) & (15)
        double t = 0.5+next_p_x/2.0;
        p_o_m_given_o_m_minus_1[0]=t;
        p_o_m_given_o_m_minus_1[1]=1-t;
        p_o_m_given_o_m_minus_1[2]=1-t;
        p_o_m_given_o_m_minus_1[3]=t;
        //std::cout << "Updated transition p_o matrix :" << p_o_m_given_o_m_minus_1 << std::endl;

        // Predict the occupancy of the cell from previous one and visibility state : Eqn 11)
        double next_p_o[2] = {p_o_m_given_o_m_minus_1[0]*prev_p_o + p_o_m_given_o_m_minus_1[2]*(1-prev_p_o), 0};
        next_p_o[1]=1-next_p_o[0];
        std::cout << "Post^: " << next_p_o[0]<<"\t";

        // Do the update on the predicted occupancy using current value as measure : Eqn (12)
        if (p_o>0.5){
            // Measured an occupied
            std::cout << " !!! ";
            next_p_o[0]=0.7 * next_p_o[0]/(1.0-p_o);
            next_p_o[1]=0.3 * next_p_o[1] / p_o;
        } else if (p_o<0.5) {
            // Measured a free
            next_p_o[0]=0.3 * next_p_o[0]/(p_o);
            next_p_o[1]=0.7 * next_p_o[1] / (1-p_o);
        } // else effectively no measurement,
        next_p_o[0] = p_o * next_p_o[0];
        next_p_o[1] = (1-p_o) *   next_p_o[1];
        if (next_p_o[0]+next_p_o[1] == 0)
            next_p_o[0]=1;
        else
            next_p_o[0] /= next_p_o[0]+next_p_o[1];
//        updated_p_o[i]=next_p_o[0];


        prev_p_o = next_p_o[0];

        std::cout << i << ":\tPrior: " <<current_map[i]<<"\tPost: "<< prev_p_o << "\tP(X): "<<next_p_x << std::endl;
    }

//    std::cout << updated_p_x << std::endl;
}
