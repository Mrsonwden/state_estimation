using namespace std;
#include <Eigen/Dense>
#include <cassert>
#include <vector>
#include <chrono>
#include <random>
#include <iostream>

#define STATE_DIM 1


int hello_world();

Eigen::VectorXd motion_equation(Eigen::MatrixXd A, 
                                                    Eigen::VectorXd last_state, 
                                                    Eigen::VectorXd input, 
                                                    Eigen::VectorXd noise_mean, 
                                                    Eigen::MatrixXd noise_var);

Eigen::VectorXd observation_equation(Eigen::MatrixXd C,
                                                                                 Eigen::VectorXd state_estimated, 
                                                                                 Eigen::VectorXd noise_mean, 
                                                                                 Eigen::MatrixXd noise_var);

double sample_from_gaussian(double mean, double variance);

void kf_filter(Eigen::VectorXd init_state, 
                            Eigen::MatrixXd init_P_prior,  
                            std::vector<Eigen::VectorXd> input_list, 
                            std::vector<Eigen::VectorXd> observation_list, 
                            std::vector<Eigen::VectorXd>& state_estimated,
                            Eigen::MatrixXd A,
                            Eigen::MatrixXd C,
                            Eigen::MatrixXd  motion_noise_var,
                            Eigen::MatrixXd  observation_noise_var
                            );