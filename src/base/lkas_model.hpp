#ifndef LKAS_MODEL_H_
#define LKAS_MODEL_H_

class lkasModel {
public:
    // member variables
    long double m_d;   //!< 2*d = distance between left and right wheels in simulator's car model
    long double m_l;   //!< l = distance between front and rear wheels in simulator's car model
    long double m_vx;  //!< vehicle longitudinal speed is 2.2m/s to simulate ~80km/h in reality
    long double m_LL;  //!< look-ahead distance is 0.25s*vx
    long double m_l_r; //!< distance from CG to rear axle (m)
    long double m_z1, m_z2, m_z3, m_z4, m_z5; //!< (initial) system states: z1 is vy; z2 is yaw rate; z3 is yL; z4 is epsilon_L; z5 is curvature at lookahead distance KL (which is K_ref of CoG)

    // constructor
    lkasModel(long double m_d,long double m_l,long double m_vx,long double m_LL,long double m_l_r) : m_d{m_d}, m_l{m_l}, m_vx{m_vx}, m_LL{m_LL}, m_l_r{m_l_r}, m_z1{0.0L}, m_z2{0.0L}, m_z3{0.0L}, m_z4{0.0L}, m_z5{0.0L}
    {        
    }
    // destructor
    ~lkasModel(){
    }
};

#endif
