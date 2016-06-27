/* 
 * File:   motion_planner.h
 * Author: deanmiller
 *
 * Created on December 4, 2015, 7:03 PM
 */

#include "main.h"
#include "boost/circular_buffer.hpp"
#include <cmath>
#ifdef ENV_TEST
#include <boost/tr1/tuple.hpp>
#else
#include <boost/math/tools/tuple.hpp>
#endif

#ifndef MOTION_PLANNER_H
#define	MOTION_PLANNER_H

#define MIN_BUF_LEN 10

typedef struct step_t{
    float point[NUM_AXIS] = {0.0, 0.0, 0.0};	//xyz point at this node
    float feedrate = 0.0;
    float max_delta;	//maximum speed change that can occur at the node
    float vm = 0.0;     //maximum speed at node
    float vs = 0.0;	//start speed at this node
    float ve = 0.0;	//end speed at this node
    float L = 0.0;	//length of line
    std::vector<float> ad_profile;	//acceleration profile;
    step_t *previous;   //pointer to previous node
    step_t *next;       //pointer to next node
} step_t;

typedef struct settings_t{
    unsigned char I2CBus;
    unsigned char I2CAddress;
    
    float Linkage_1;
    float Linkage_2;
    float L1_2;
    float L2_2;

    float SCARA_offset_x;
    float SCARA_offset_y;
    float axis_scaling[NUM_AXIS];
    float max_plunge;
    /*
     * 360 degrees / 1.8 degrees per step = 200 steps per revolution
     * 200 steps / 1.5875 mm per rev (lead screw) = 125.984252 steps per mm
     */
    float steps_per_mm;
    float steps_per_radian_x;
    float steps_per_radian_y;

    float Sm;
    float Jm;
    float Am;
    float Vm;
    float Fmax;
    float M;
    float T;  
} settings_t;

typedef struct interpolate_t{
    float tm, t_current, dm, dist, speed;
    float cosines[NUM_AXIS];
    step_t step;
} interpolate_t;

//optimization functions
template <class T>
struct vd1_functor
{
   vd1_functor(T const& Sm, T const& t1, T const& vs, T const& dm) : _Sm(Sm), _t1(t1), _vs(vs), _dm(dm) { }
#ifdef ENV_TEST
   std::tr1::tuple<T, T> operator()(T const& z)
#else
   boost::math::tuple<T, T> operator()(T const& z)
#endif
   {
      T fn = 4*_Sm*std::pow(_t1, 4) + 3*_Sm*std::pow(_t1, 3)* z + .5*_Sm*std::pow(_t1, 2)*std::pow(z, 2) + _vs*(4*_t1 + z) - _dm;
      T d = .5*3*_Sm*std::pow(_t1, 2) * std::pow(z, 2) + _vs + 3*_Sm*std::pow(_t1, 3);
#ifdef ENV_TEST
      return std::tr1::make_tuple(fn, d);
#else
      return boost::math::make_tuple(fn, d);
#endif
   }
private:
   T _Sm;
   T _t1;
   T _vs;
   T _dm;
};

template <class T>
struct vd2_functor
{
   vd2_functor(T const& Sm, T const& vs, T const& dm) : _Sm(Sm), _vs(vs), _dm(dm) { }
#ifdef ENV_TEST
   std::tr1::tuple<T, T> operator()(T const& z)
#else
   boost::math::tuple<T, T> operator()(T const& z)
#endif
   {
      T fn = 4*_Sm* std::pow(z, 4) + 4*_vs*z - _dm;
      T d = 16 * _Sm * std::pow(z, 3) + 4 * _vs;
#ifdef ENV_TEST
      return std::tr1::make_tuple(fn, d);
#else
      return boost::math::make_tuple(fn, d);
#endif
   }
private:
   T _Sm;
   T _vs;
   T _dm;
};

class motion_planner {
public:
    motion_planner(settings_t _set);
    bool plan_buffer(std::vector<float> &buf, float feedrate);
    bool full();
    bool empty();
    bool data_ready();
    uint32_t interpolate(uint32_t max_items, uint16_t *buf);
    void recalculate(void);
    int size();
    void pop();
    virtual ~motion_planner();
    
    int min_buf_len; //minimum number of steps that must be in the buffer before we start interpolating
private:
    settings_t set;
    interpolate_t interp; //the current interpolation position
    long int last_pos[NUM_AXIS]; //previous position of motors in interpolation cycle
    step_t head;
    
    boost::circular_buffer<step_t> cb; //buffer to store steps in
    
    float force(float a[NUM_AXIS], float b[NUM_AXIS], float c[NUM_AXIS], float V); //return the impulse of an angle (XYZ) at a speed (V)
    float _pro_vd(float vs, float dm); //return the max speed reachable with a specified starting speed (vs) and distance (dm)
    bool _pro_vv(float vs, float ve, std::vector<float> &ad); //return the A/D profile (t1, t2, t3) in vector ad
    float _pro_ad(float vs, float ve, float dm, float vm, std::vector<float> &ad);
    float _v0(float t, std::vector<float> &ad);
    int calculate_delta(float cartesian[NUM_AXIS], long int delta[NUM_AXIS]);
    float _v_bar(float t, std::vector<float> &ad, float vs, float vm);
    void zero(void);
};

#endif	/* MOTION_PLANNER_H */

