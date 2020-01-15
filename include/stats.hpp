#ifndef STATS_HPP
#define STATS_HPP

namespace Hypertun_SR
{

struct stats {
    // flag for accuracy calculation
    bool acc_calc;

    // number of iterations
    int it;
    
    // elapsed algorithm time
    float alg_time;

    // algorithm frquency
    float alg_freq;

    // accuracy
    float acc2;
    float acc3;
    float acc4;
    float acc5;

};

}  // namespace Hypertun_SR


#endif // STATS_HPP