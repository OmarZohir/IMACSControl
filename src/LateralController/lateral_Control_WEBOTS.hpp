/** @file lateral_Control_WEBOTS.hpp
 *  @brief The header file for lateral controller implementation when using IMACS framework with WEBOTS simulator
 */
#ifndef LANEDETECTION_LATERAL_CONTROL_WEBOTS_H_
#define LANEDETECTION_LATERAL_CONTROL_WEBOTS_H_

#include <Eigen/Eigen>
#include "lkas_model.hpp"
#include "config_webots.hpp"
// 11 pipelines, 1 core per pipe
//Case 4
#define MAX_SCENARIOS 1 //!< maximum number of scenarios defined
#define PIPELINES_NUM 11 // Extra pipelines : Case 3 being considered
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

            period_ms={8.333f}; //!< sampling period h in ms for each scenario s_i
            tau_ms={91.6666f}; //!< sensor-to-actuator delay in ms for each scenario s_i
	    if (period_ms.size() != tau_ms.size())
		throw range_error("In lateralControllerWEBOTS, size of vectors period_ms and tau_ms should be the same");
            /////////////////////////////////////////////////////////////// v0 ////////////////////////////////////////////////////////////////////////////
            //Case1: Q = 5, R = 500
            m_phi_aug[0] <<   
   0.930807917585907,  -0.122339687199470,                   0,                   0,                   0,   0.465744993509085, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  -0.004271812286175,   0.949739815372259,                   0,                   0,                   0,   0.259948251911874, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  -0.007940537014226,  -0.044623099709004,                   1,   0.115740740740741,   0.006697959533608,  -0.008046508054319, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
   0.000018171358072,  -0.008121719954082,                   0,                   1,   0.115740740740741,  -0.001093948346287, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0,                   0,                   0,                   0,   1.000000000000000,                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,
                   0,                   0,                   0,                   0,                   0,                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;


	    m_K2c[0] <<

   0.071205291612542,  -1.001028021447707,   0.689940834414254,   1.783776984494047,                   0,  -0.230359574535768,
  -0.225811405070194,  -0.220867086621302,  -0.215478871812408,  -0.209594202044813,  -0.203155248887038,  -0.196098412618028,
  -0.188353773951939,  -0.179844494604839,  -0.170486161962627,  -0.160186072671274;






	   
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
