#include "lg_kf.hpp"

#include <chrono>
#include <random>
#include <cassert>
#include <istream>
#include <ostream>
#include <string>
#include <fstream>
using namespace std;


void show_state(std::vector<double>& x_state, std::vector<double>& y_state){
    assert(x_state.capacity() == y_state.capacity());
    for(int index = 0; index < x_state.size(); index++){
        std::cout << index << "->   x: " << x_state[index] << " y: " << y_state[index] << std::endl;
    }
}

void generate_gt(Eigen::MatrixXd A,
                                    Eigen::MatrixXd C,
                                    Eigen::VectorXd init_state, 
                                    std::vector<Eigen::VectorXd>& input_list,  
                                    std::vector<Eigen::VectorXd>& state_list, 
                                    std::vector<Eigen::VectorXd>& observation_list,  
                                    Eigen::MatrixXd x_noise_var, 
                                    Eigen::MatrixXd y_noise_var, 
                                    int total){
    assert(input_list.capacity() >= total);
    assert(observation_list.capacity() >= total);
    assert(state_list.capacity() >= total);
    Eigen::VectorXd init_input(STATE_DIM);
    init_input << 0;  // -- depend state_dim
    input_list[0]  = init_input;
    for (int index = 1; index < total; index++){
        Eigen::VectorXd input(STATE_DIM);
        input<< 5;   // -- depend state_dim
        input_list[index] = input;
    }

    state_list[0] = init_state;
    observation_list[0] = observation_equation(C, state_list[0], Eigen::VectorXd::Zero(state_list[0].rows()), y_noise_var);
    for (int index = 1; index < total; index++){
        state_list[index] = motion_equation(A, state_list[index-1], input_list[index],  Eigen::VectorXd::Zero(state_list[0].rows()), x_noise_var);
        observation_list[index] = observation_equation(C, state_list[index], Eigen::VectorXd::Zero(state_list[0].rows()), y_noise_var);
    }
}


void save_result_csv(string csv_path , std::vector<Eigen::VectorXd> state_list, std::vector<Eigen::VectorXd> state_estimated, std::vector<Eigen::VectorXd> observation_list){
    assert(state_list.size() == state_estimated.size());
    assert(state_list.size() == observation_list.size());
    std::ofstream csv_file;
    csv_file.open(csv_path);
    // set header;
    csv_file << "state_gt, state_observation, state_estimate\n";
    // set values;
    for(int32_t index = 0; index < state_list.size(); index++){
        csv_file << state_list[index] << "," << observation_list[index] << "," << state_estimated[index] << "\n";
    }

    csv_file.close();
    std::cout << "save result csv finished" << std::endl;
}


int  main(){
    hello_world();
    const int   total = 10;
    Eigen::VectorXd init_state(1);
    init_state(0) = 100;
    Eigen::MatrixXd init_P_prior(1,1);
    init_P_prior << 3;
    Eigen::MatrixXd motion_noise_var(1,1);
    motion_noise_var << 1;
    Eigen::MatrixXd observation_noise_var(1,1);
    observation_noise_var << 0.5;
    std::vector<Eigen::VectorXd> input_list(total);
    std::vector<Eigen::VectorXd> state_list(total);
    std::vector<Eigen::VectorXd> observation_list(total);
    Eigen::MatrixXd A(1,1);
    A << 1;
    Eigen::MatrixXd C(1,1);
    C << 1;
    std::cout << "init finished" << std::endl;
    generate_gt(A, C, init_state, input_list, state_list, observation_list, motion_noise_var, observation_noise_var, total);
    std::cout << "generate_gt finished" << std::endl;
    std::vector<Eigen::VectorXd> state_estimated(total);

    Eigen::VectorXd init_state_new(1);
    init_state_new(0) = 130;
    kf_filter(init_state_new,
                     init_P_prior,
                     input_list,
                     observation_list,
                     state_estimated,
                     A,
                     C,
                     motion_noise_var,
                     observation_noise_var);

    string csv_path = "/home/snowden/workplace/state_estimation_for_robotics/state_estimation/data/kf_result.csv";
    save_result_csv(csv_path, state_list, state_estimated,observation_list);
    // for(int index  = 0; index < state_estimated.size(); index++){
    //         std::cout << "state true -> estimate: " << state_list[index] << " - > " << state_estimated[index] << "   diff : " <<  state_estimated[index] - state_list[index] 
    //             << "     ratio: " << (state_estimated[index] - state_list[index]) /  500 * 100 << "%       "<<  "rel: " << (state_estimated[index] - state_list[index]) /  state_list[index] * 100 << "%       " <<  std::endl;
    // }

    std::cout << "perfect ! " << std::endl << std::endl;
}



