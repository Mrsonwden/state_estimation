# include "lg_kf.hpp"

int hello_world(){
    printf("hello_world\n\r");
}

// Function to generate multivariate normal random samples
Eigen::VectorXd generateMultivariateNormalSamples(const Eigen::VectorXd& mean, const Eigen::MatrixXd& covariance) {
    // unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    // std::default_random_engine generator(seed);
    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 1.0); // standard normal distribution

    int dim = mean.size();
    // Compute the Cholesky decomposition of the covariance matrix
    Eigen::MatrixXd transformation = covariance.llt().matrixL();
    
    Eigen::VectorXd sample(dim);
    for (int j = 0; j < dim; ++j) {
        sample[j] = distribution(generator); // generating standard normal random values
    }

        // Transforming the standard normal into our desired multivariate normal distribution
    Eigen::VectorXd sample_result = mean + transformation * sample;

    return sample_result;
}

Eigen::VectorXd motion_equation(Eigen::MatrixXd A, 
                                                    Eigen::VectorXd last_state, 
                                                    Eigen::VectorXd input, 
                                                    Eigen::VectorXd noise_mean, 
                                                    Eigen::MatrixXd noise_var){
    Eigen::VectorXd state_estimated;
    state_estimated = A *  last_state + input + generateMultivariateNormalSamples(noise_mean, noise_var);
    return state_estimated;
}

Eigen::VectorXd observation_equation(Eigen::MatrixXd C,
                                                                                 Eigen::VectorXd state_estimated, 
                                                                                 Eigen::VectorXd noise_mean, 
                                                                                 Eigen::MatrixXd noise_var){
    Eigen::VectorXd state_observed;
    state_observed = C * state_estimated + generateMultivariateNormalSamples(noise_mean, noise_var);
    return state_observed; 
}


void kf_filter(Eigen::VectorXd init_state, 
                            Eigen::MatrixXd init_P_prior,  
                            std::vector<Eigen::VectorXd> input_list, 
                            std::vector<Eigen::VectorXd> observation_list, 
                            std::vector<Eigen::VectorXd>& state_estimated,
                            Eigen::MatrixXd A,
                            Eigen::MatrixXd C,
                            Eigen::MatrixXd  motion_noise_var,
                            Eigen::MatrixXd  observation_noise_var
                            ){
    assert(input_list.size() == observation_list.size());
    assert(input_list.size() > 0);
    assert(state_estimated.capacity() >= (input_list.size()));
    state_estimated[0] = init_state;
    Eigen::MatrixXd P_prior;
    P_prior = init_P_prior;
    for (int index = 1; index < input_list.size(); index++){
            std::cout << "index: " << index << std::endl;
            // calculate_prior_state;
            Eigen::VectorXd  prior_state;
            prior_state = motion_equation(A, state_estimated[index - 1], input_list[index], Eigen::VectorXd::Zero(A.rows()), Eigen::MatrixXd::Zero(A.rows(), A.cols())); 
            std::cout << "prior_state: " << prior_state << std::endl;

            // calculate_prior_var;
            Eigen::MatrixXd  prior_var;
            prior_var = A * P_prior * A.transpose() + motion_noise_var;
            std::cout << "prior_var: " << prior_var << std::endl;

            // calculate_kf_gain;
            Eigen::MatrixXd  Kk;
            Kk = P_prior * C.transpose() * (C * P_prior * C.transpose() + observation_noise_var).inverse();
            // std::cout <<   "calcul: " <<P_prior * C.transpose()  << "   " <<  C * P_prior * C.transpose() << "   " << (C * P_prior * C.transpose() + observation_noise_var).inverse() << "   " << P_prior * C.transpose() * (C * P_prior * C.transpose() + observation_noise_var).inverse() << std::endl;
            std::cout << "Kk: " << Kk <<std::endl;

            // calculate_posterior_state;
            Eigen::VectorXd  posterior_state;
            Eigen::VectorXd observation ;
            observation = observation_list[index];
            posterior_state =  prior_state + Kk * (observation - C * prior_state);
            std::cout << "posterior_state: " << posterior_state << std::endl;

            // calculate_posterior_var;
            Eigen::MatrixXd  posterior_var;
            posterior_var =  (Eigen::MatrixXd::Identity(prior_var.rows(), prior_var.cols()) - Kk * C) * prior_var;
            std::cout << "posterior_var: " << posterior_var << std::endl << std::endl;

            // save result;
            state_estimated[index] = posterior_state;
            P_prior = posterior_var;
    }
}