/** @file lateral_Control_WEBOTS.hpp
 *  @brief The header file for lateral controller implementation when using IMACS framework with WEBOTS simulator
 */
#ifndef LANEDETECTION_LATERAL_CONTROL_WEBOTS_H_
#define LANEDETECTION_LATERAL_CONTROL_WEBOTS_H_

#include <Eigen/Eigen>
#include "lkas_model.hpp"
#include "config_webots.hpp"
// 6 pipelines, 1 core per pipe
//Case 3
#define MAX_SCENARIOS 1 //!< maximum number of scenarios defined
#define PIPELINES_NUM 6 // Extra pipelines : Case 3 being considered
#define LENGTH_PHI_AUG 5+PIPELINES_NUM //!< length of the phi_aug matrix of the controller.
/// @brief Class for lateral controller in Webots.
class lateralControllerWEBOTS : lkasModel {
private:
    /// member variables
    std::vector<long double> m_input = std::vector<long double> (40000); /// the control input. Stored as a list (inefficient).[TO DO: convert from vector<long double> to long double for delay <= h]
    long double m_desired_steering_angle; /// computed steering angle
    /// Controller matrices. Note that the Matrix dimensions need to be changed depending on the type of controller and controller implementation choice.
    std::vector< Eigen::Matrix<long double, LENGTH_PHI_AUG, LENGTH_PHI_AUG> > m_phi_aug
                                        = std::vector< Eigen::Matrix<long double, LENGTH_PHI_AUG, LENGTH_PHI_AUG> > (MAX_SCENARIOS); /// controller phi_aug matrices
    std::vector< Eigen::Matrix<long double, 1, LENGTH_PHI_AUG> > m_K2c
                                        = std::vector< Eigen::Matrix<long double, 1, LENGTH_PHI_AUG> > (MAX_SCENARIOS);  /// controller gain matrices
    std::vector< Eigen::Matrix<long double, LENGTH_PHI_AUG, 1> > m_Gamma_aug
                                        = std::vector< Eigen::Matrix<long double, LENGTH_PHI_AUG, 1> > (MAX_SCENARIOS);  /// controller Gamma_aug matrices
public:

    vector<float> period_ms; 	/// sampling period in milliseconds
    vector<float> tau_ms; 	/// sensor-to-actuator delay in milliseconds

    /**  @brief C++ implementation of the controller that computes the steering angle for the LKAS represented by the m_phi_aug and m_Gamma_aug matrices using the gain matrix m_K2c
         @param[in] the_yL           	Lateral Deviation at the look-ahead distance. Since the system is SISO, only one state is needed as input
         @param[in] the_it_counter	The iteration counter that keeps track of the k-th instance
	 @param[in] scenario		The scenario to simulate
	 @note the function updates the class member m_desired_steering_angle
      */
    void compute_steering_angle(long double the_yL, int the_it_counter,int scenario);

    /**  @brief Function to access the steering angle from outside the class
	 @return the m_desired_steering_angle.
      */
    long double get_steering_angle();

    /**  @brief C++ implementation of a simple estimator
         @param[in] the_it_counter	The iteration counter that keeps track of the k-th instance
	 @param[in] scenario		The scenario to simulate
	 @note the function updates the m_input and (derived) states of the system m_zi
      */
    void estimate_next_state(int the_it_counter,int scenario);
    /// constructor
    lateralControllerWEBOTS(int vel) : lkasModel(1.628L, 2.995L, (vel*5/18), 5.5L, 1.2975L) {
            m_desired_steering_angle = 0.0L;
            fill(m_input.begin(), m_input.end(), 0.0L); // m_input is the input of the last sampling period.

            period_ms={16.6666f}; //!< sampling period h in ms for each scenario s_i
            tau_ms={100.f}; //!< sensor-to-actuator delay in ms for each scenario s_i
	    if (period_ms.size() != tau_ms.size())
		throw range_error("In lateralControllerWEBOTS, size of vectors period_ms and tau_ms should be the same");
            /////////////////////////////////////////////////////////////// v0 ////////////////////////////////////////////////////////////////////////////
            //Case1: Q = 5, R = 500
            m_phi_aug[0] <<   

   0.866925991619477,  -0.230065621413774,                   0,                   0,                   0,   0.867462133216390, 0, 0, 0, 0, 0,
  -0.008033346910390,   0.902528329082198,                   0,                   0,                   0,   0.504841881503503, 0, 0, 0, 0, 0, 
  -0.015138927064923,  -0.086972005257029,   1.000000000000000,   0.231481481481481,   0.026791838134431,  -0.031517592624964, 0, 0, 0, 0, 0,
   0.000069779865124,  -0.015837463842039,                   0,   1.000000000000000,   0.231481481481481,  -0.004290660378108, 0, 0, 0, 0, 0,
                   0,                   0,                   0,                   0,   1.000000000000000,                   0, 0, 0, 0, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 1, 0, 0, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 1, 0, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 1, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 0, 1, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 0, 0, 1,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 0, 0, 0;




	    m_K2c[0] <<
   //0.179331678082375,  -2.232889373667210,   1.574251418143982,   3.730449252764377,  -0.000000000000132,  -0.993862609283507,  -0.942362335002195,  -0.880817668022051, -0.806813566696677,  -0.717433652139471,  -0.609160837208317; //Q=500,R=50, Too aggressive
   0.034794093456874,  -0.693412169158249,   0.478859196079693,   1.465161660828294,  -0.000000000000052,  -0.331625234402305,  -0.320556911224497,  -0.307667189872094, -0.292485229721852,  -0.274440498223270,  -0.252842876893185;




	   
	    m_Gamma_aug[0] <<
			  0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        1;
	
    }
    // destructor
    ~lateralControllerWEBOTS(){
    }
};

#endif
