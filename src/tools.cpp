#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    rmse.fill(0.0);

    if(estimations.at(0).size()!=rmse.size()) {
        cout << "Size Mismatch - Error ! \n";
        return rmse;
    }

    for(unsigned int i=0; i<estimations.size(); ++i)
    {
        VectorXd res = estimations[i] - ground_truth[i];
        res = res.array()*res.array();
        rmse += res;
    }

    rmse = rmse/estimations.size();
    rmse = rmse.array().sqrt();
    return rmse;
}