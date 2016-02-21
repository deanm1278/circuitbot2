/* 
 * File:   motion_planner.cpp
 * Author: deanmiller
 * 
 * Created on December 4, 2015, 7:03 PM
 */

#include "motion_planner.h"
#include <iostream>
#include <cmath>
#include <numeric>
#include <iterator>
#include <algorithm>
#include <boost/math/tools/roots.hpp>

#include <fstream> //REMOVE: only for logging

motion_planner::motion_planner(settings_t _set) : cb(100){
    set = _set;
}

bool comp(float i, float j){
    return std::abs(i) < std::abs(j);
}

float motion_planner::force(float x[2], float y[2], float z[2], float V){
    float theta_P, theta_L;
    theta_P = std::atan2(y[1] - x[1], y[0] - x[0]); //direction of momentum vector
    theta_L = std::atan2(z[1] - y[1], z[0] - y[0]); //direction of resultant vector
    return std::sqrt(std::pow(set.M*V*std::cos(theta_L) - set.M*V*std::cos(theta_P), 2) + std::pow(set.M*V*std::sin(theta_L) - set.M*V*std::sin(theta_P), 2))/set.T;
}

float motion_planner::_pro_vd(float vs, float dm){
    float t[] = {0., 0., 0.};
    float d0;
    if(std::pow(set.Jm, 2) >= set.Sm*set.Am){
        t[1] = 0.;
        d0 = (4* std::pow(set.Am, 2)/set.Sm) + 4*vs*std::sqrt(set.Am/set.Sm);
        //optimization parameters
        float min = 0.0;
        float max = 100.0;
        float guess = 0.5;
        int digits = std::numeric_limits<float>::digits;
        if(dm > d0){
            //this can probably be solved with a quadratic equation
            t[0] = std::sqrt(set.Am/set.Sm);
            t[2] = boost::math::tools::newton_raphson_iterate(vd1_functor<float>(set.Sm, t[0], vs, dm), guess, min, max, digits);
        }
        else{
            t[2] = 0.0;
            t[0] = boost::math::tools::newton_raphson_iterate(vd2_functor<float>(set.Sm, vs, dm), guess, min, max, digits);
        }
        //return the end speed
        return vs + 2*set.Sm*std::pow(t[0], 3) + set.Sm*t[0]*t[1]*t[2] + set.Sm*t[0]*std::pow(t[1], 2) + 3*set.Sm*std::pow(t[0], 2)*t[1] + set.Sm*std::pow(t[0], 2)*t[2];
    }
    else{
        //todo implement this
        return 0;
    }
}

bool motion_planner::_pro_vv(float vs, float ve, std::vector<float> &ad){
    float v_star, t1, t2, t3;
    v_star = std::abs(ve - vs);
    
    ad.clear();
    if(pow(set.Jm, 2) >= set.Sm * set.Am){
       if(v_star > 2*set.Sm*std::pow(set.Am/set.Sm, 3.0/2.0)){
           t1 = std::sqrt(set.Am/set.Sm);
           t2 = 0.;
           t3 = (v_star - 2 * set.Sm * std::pow(t1, 3))/set.Am;
       }
       else{
           t1 = std::pow(v_star/(2 * set.Sm), 1./3);
           t2 = 0.;
           t3 = 0.;
       }
    }
    else{
       if(v_star > (std::pow(set.Am, 2)/set.Jm + (set.Am* set.Jm)/set.Sm)){
           t1 = set.Jm/set.Sm;
           t2 = (set.Am - (std::pow(set.Jm, 2)/set.Sm)) / set.Jm;
           t3 = (v_star - (2*set.Sm*std::pow(t1, 3) + 3*set.Sm*std::pow(t1,2) + set.Sm*t1*std::pow(t2, 2)))/set.Am;
       }
       else if((2*std::pow(set.Jm, 3))/std::pow(set.Sm, 2) < v_star && v_star <= (std::pow(set.Am, 2)/set.Jm) + (set.Am * set.Jm)/set.Sm){
           t1 = set.Jm/set.Sm;
           t2 = (std::sqrt(std::pow(t1, 2) + (4*v_star)/set.Jm) - 3*t1)/2;
           t3 = 0.;
       }
       else{
           t1 = pow(v_star/(2*set.Sm), 1./3);
           t2 = 0.;
           t3 = 0.;
       }
    }
    ad.push_back(t1);
    ad.push_back(t2);
    ad.push_back(t3);
}

float motion_planner::_pro_ad(float vs, float ve, float dm, std::vector<float> &ad){
    std::vector<float> t, ti, td;
    float dmin, di, dd, t4, v0, v1, e, v2;
    
    ad.clear();
    _pro_vv(vs, ve, t);
    dmin = ((vs + ve)/2) * (4*t.at(0) + 2*t.at(1) + t.at(2));
    if(dmin > dm){
        return false;
    }
    _pro_vv(vs, set.Vm, ti);
    _pro_vv(ve, set.Vm, td);
    di = ((vs + set.Vm)/2) * (4*ti.at(0) + 2*ti.at(1) + ti.at(2));
    dd = ((set.Vm + ve)/2) * (4*td.at(0) + 2*td.at(1) + td.at(2));
    if(di + dd <= dm){
        //Vm is reachable
        t4 = (dm - di - dd)/set.Vm;
        ad.push_back(ti.at(0));
        ad.push_back(ti.at(1));
        ad.push_back(ti.at(2));
        ad.push_back(t4);
        ad.push_back(td.at(0));
        ad.push_back(td.at(1));
        ad.push_back(ti.at(2));
        return set.Vm;
    }
    else{
        t4 = 0.0;
        v0 = set.Vm;
        v1 = ve;
        e = .001;
        while(1){
            v2 = (v0 + v1)/2;
            _pro_vv(vs, v2, ti);
            _pro_vv(ve, v2, td);
            di = ((vs + v2)/2) * (4*ti.at(0) + 2*ti.at(1) + ti.at(2));
            dd = ((v2 + ve)/2) * (4*td.at(0) + 2*td.at(1) + td.at(2));
            if(abs(di + dd - dm) < e){
                ad.push_back(ti.at(0));
                ad.push_back(ti.at(1));
                ad.push_back(ti.at(2));
                ad.push_back(t4);
                ad.push_back(td.at(0));
                ad.push_back(td.at(1));
                ad.push_back(ti.at(2));
                return v2;
            }
            else if(di + dd <= dm){
                v0 = std::max(v0, v1);
                v1 = v2;
            }
            else{
                v1 = std::min(v0, v1);
                v0 = v2;
            }
        }
    }
}

float motion_planner::_v0(float t, std::vector<float> &ad){
    if (0 <= t && t < ad.at(0))
        return (1./6) * set.Sm * pow(t, 3);
    else if (ad.at(0) <= t && t < ad.at(0) + ad.at(1))
        return (1./2) * pow(t - ad.at(0), 2) * set.Sm * ad.at(0) + (1./2) * (t - ad.at(0)) * set.Sm * pow(ad.at(0), 2) + (1./6) * set.Sm * pow(ad.at(0), 3);
    else if (ad.at(0) + ad.at(1) <= t && t < 2*ad.at(0) + ad.at(1))
        return (set.Sm * pow(ad.at(0), 3))/3 + (set.Sm * pow(ad.at(0), 2) * ad.at(1))/2 - set.Sm * pow(ad.at(0), 2) * t + (set.Sm * ad.at(0) * pow(ad.at(1), 2))/2 - set.Sm * ad.at(0) * ad.at(1) * t + set.Sm * ad.at(0) * pow(t, 2) + (set.Sm * pow(ad.at(1),3))/6 - (set.Sm*pow(ad.at(1), 2) * t)/2 + (set.Sm*ad.at(1)*pow(t,2))/2 - (set.Sm*pow(t, 3))/6;
    else if (2 * ad.at(0) + ad.at(1) <= t && t < 2 * ad.at(0) + ad.at(1) + ad.at(2))
        return -1 * (set.Sm*pow(ad.at(0), 3)) - (3*(set.Sm*pow(ad.at(0), 2) * ad.at(1))/2) - ((set.Sm * ad.at(0) * pow(ad.at(1), 2))/2) + set.Sm * ad.at(0) * ad.at(1) * t + set.Sm * pow(ad.at(0), 2) * t;
    else if (2 * ad.at(0) + ad.at(1) + ad.at(2) <= t && t < 3 * ad.at(0) + ad.at(1) + ad.at(2))
        return set.Sm * (ad.at(0) * ad.at(1) + pow(ad.at(0), 2)) * (t - 2 * ad.at(0) - ad.at(1) - ad.at(2)) + set.Sm*pow(2*ad.at(0) + ad.at(1) + ad.at(2) - t, 3) / 6 + set.Sm*pow(ad.at(0), 3) + (3 * set.Sm * pow(ad.at(0), 2) * ad.at(1))/2 + (set.Sm*ad.at(0)*pow(ad.at(1), 2))/2 + set.Sm*ad.at(0)*ad.at(1)*ad.at(2) + set.Sm*pow(ad.at(0), 2)*ad.at(2);
    else if (3 * ad.at(0) + ad.at(1) + ad.at(2) <= t && t < 3*ad.at(0) + 2 * ad.at(1) + ad.at(2))
        return 2*set.Sm*ad.at(0)*ad.at(1)*t + (7*set.Sm*pow(ad.at(0),2)*t)/2 + set.Sm*ad.at(0)*ad.at(2)*t - (set.Sm*ad.at(0)*pow(t, 2))/2 - 4*set.Sm*pow(ad.at(0), 2)*ad.at(1) - set.Sm*ad.at(0)*pow(ad.at(1), 3) - set.Sm*ad.at(0)*ad.at(1)*ad[3] - (25*set.Sm*pow(ad.at(0),3))/6 - (5*set.Sm*pow(ad.at(0),2)*ad.at(2))/2 - (set.Sm*ad.at(0)*pow(ad.at(2),2))/2;
    else if (3 * ad.at(0) + 2 * ad.at(1) + ad.at(2) <= t && t < 4 * ad.at(0) + 2*ad.at(1) + ad.at(2))
        return (-26 * set.Sm * pow(ad.at(0), 3))/3 + 4*set.Sm*ad.at(0)*ad.at(2)*t - 7*set.Sm*ad.at(0)*ad.at(1)*ad.at(2) + 8*set.Sm*ad.at(0)*ad.at(1)*t + 2*set.Sm*ad.at(1)*ad.at(2)*t - 7*set.Sm*ad.at(0)*pow(ad.at(1), 2) + 8*set.Sm*pow(ad.at(0),2)*t - 2*set.Sm*ad.at(0)*pow(t,2) - 13*set.Sm*pow(ad.at(0),2)*ad.at(1) - 7*set.Sm*pow(ad.at(0),2)*ad.at(2) - 2*set.Sm*ad.at(0)*pow(ad.at(2),2) - set.Sm*ad.at(1)*pow(t,2) - (set.Sm*ad.at(2)*pow(t,2))/2 + 2*set.Sm*pow(ad.at(1),2)*t + (set.Sm*pow(ad.at(2),2)*t)/2 - 2*set.Sm*pow(ad.at(1),2)*ad.at(2) - set.Sm*ad.at(1)*pow(ad.at(2),2) + (set.Sm*pow(t,3))/6 - (4*set.Sm*pow(ad.at(1),3))/3 - (set.Sm*pow(ad.at(2),3))/6;
    else if (4 * ad.at(0) + 2*ad.at(1) + ad.at(2) <= t)
        return 2*set.Sm*pow(ad.at(0),3) + set.Sm*ad.at(0)*ad.at(1)*ad.at(2) + set.Sm*ad.at(0)*pow(ad.at(1),2) + 3*set.Sm*pow(ad.at(0),2)*ad.at(1) + set.Sm*pow(ad.at(0),2)*ad.at(2);

    //should never get here
    else return 0;
}

int motion_planner::calculate_delta(float cartesian[NUM_AXIS], uint32_t delta[NUM_AXIS]){
        //DM: modified slightly for application. Original from https://github.com/vitaminrad/Marlin-for-Scara-Arm
	// Inverse kinematics.
	float SCARA_pos[2];
	float SCARA_C2, SCARA_S2, SCARA_K1, SCARA_K2, SCARA_theta, SCARA_psi;

	SCARA_pos[X_AXIS] = cartesian[X_AXIS] * set.axis_scaling[X_AXIS] - set.SCARA_offset_x;  //Translate SCARA to standard X Y
	SCARA_pos[Y_AXIS] = cartesian[Y_AXIS] * set.axis_scaling[Y_AXIS] - set.SCARA_offset_y;  // With scaling factor.

	if (set.Linkage_1 == set.Linkage_2){
		SCARA_C2 = ( ( pow(SCARA_pos[X_AXIS], 2) + pow(SCARA_pos[Y_AXIS], 2) ) / (2 * (float)set.L1_2) ) - 1;
        }
	else{
                SCARA_C2 =   ( pow(SCARA_pos[X_AXIS], 2) + pow(SCARA_pos[Y_AXIS], 2) - (float)set.L1_2 - (float)set.L2_2 ) / 45000;
        }

	SCARA_S2 = sqrt( 1 - pow(SCARA_C2, 2) );

	SCARA_K1 = set.Linkage_1 + set.Linkage_2 * SCARA_C2;
	SCARA_K2 = set.Linkage_2 * SCARA_S2;

	SCARA_theta = ( atan2(SCARA_pos[X_AXIS],SCARA_pos[Y_AXIS])-atan2(SCARA_K1, SCARA_K2) ) * -1;
	SCARA_psi   =   atan2(SCARA_S2,SCARA_C2);

	delta[X_AXIS] = SCARA_theta * RAD_TO_TICK; // Theta is support arm angle (shoulder)
	delta[Y_AXIS] = SCARA_psi * RAD_TO_TICK;   // - Psi sub arm angle (elbow)
	delta[Z_AXIS] = cartesian[Z_AXIS] * TO_FIXED;

    return 0;
}

float motion_planner::_v_bar(float t, std::vector<float> &ad, float vs, float vm){
    float tm1, tm2, tmax;
    tm1 = 4*ad.at(0) + 2*ad.at(1) + ad.at(2);
    tm2 = 4*ad.at(4) + 2*ad.at(5) + ad.at(6);
    tmax = tm1 + ad.at(3) + tm2;


    if(t >= 0 && t <= tm1){
        std::vector<float> ad2 = {ad.at(0), ad.at(1), ad.at(2)};
        return vs + _v0(t, ad2);
    }
    else if(t >= tm1 && t <= tm1 + ad.at(3)) return vm;
    else if(t >= tm1 + ad.at(3) && t <= tmax){
        std::vector<float> ad2 = {ad.at(4), ad.at(5), ad.at(6)};
        return vm - _v0(t - tm1 - ad.at(3), ad2);
    }
    //should never get here
    return 0;
}
                

void motion_planner::recalculate(void){
    //recalculate the entire buffer of data
    boost::circular_buffer<step_t>::iterator iter = cb.begin();
    while(std::distance(iter, cb.end()) > 2){
        step_t step = *iter;
        step_t next = *(iter + 1);
        step_t next_2 = *(iter + 2);
        float x[2] = {step.point[X_AXIS], step.point[Y_AXIS]};
        float y[2] = {next.point[X_AXIS], next.point[Y_AXIS]};
        float z[2] = {next_2.point[X_AXIS], next_2.point[Y_AXIS]};
        
        float dfdv = this->force(x, y, z, 1.0);

        //determine limit speed at next node
        next.speed = std::min(set.Fmax/dfdv * 1000, set.Vm); //convert to mm/s
        
        if(std::distance(iter, cb.end()) > 1){
            //check if speed is reachable given the length of the line
            float dm = std::sqrt(std::pow(next.point[X_AXIS] - step.point[X_AXIS], 2) + std::pow(next.point[Y_AXIS] - step.point[Y_AXIS], 2));
            float max_reachable = this->_pro_vd(step.speed, dm);
            next.speed = std::min(next.speed, max_reachable);
        }
        
        *(iter + 1) = next;
        iter++;
    }
    
    //todo make sure this makes sense
    //iterate over the list in reverse order to make sure we can stop in time for all of the segments
    boost::circular_buffer<step_t>::reverse_iterator riter = cb.rbegin();
    while(std::distance(riter, cb.rend()) > 1){
        step_t step = *riter;
        step_t next = *(riter + 1);
        //check if speed is reachable given the length of the line
        float dm = std::sqrt(std::pow(next.point[X_AXIS] - step.point[X_AXIS], 2) + std::pow(next.point[Y_AXIS] - step.point[Y_AXIS], 2));
        float max_reachable = this->_pro_vd(step.speed, dm);
        //set speed to max reachable speed
        next.speed = std::min(next.speed, max_reachable);
        *(riter + 1) = next;
        riter++;
    }
    
    //now calculate the acc/dec profile at each node
    iter = cb.begin();
    while(std::distance(iter, cb.end()) > 1){
        step_t step = *iter;
        step_t next = *(iter + 1);
        float L = std::sqrt(std::pow(next.point[X_AXIS] - step.point[X_AXIS], 2) + std::pow(next.point[Y_AXIS] - step.point[Y_AXIS], 2));

        if(this->_pro_vd(step.speed, L) < set.Vm){
            _pro_vv(step.speed, next.speed, step.ad_profile);
        }
        else{
            step.vm = _pro_ad(step.speed, next.speed, L, step.ad_profile);
        }
        
        *iter = step;
        iter++;
    }
}

bool motion_planner::interpolate(int max_items){
    std::ofstream logfile;
    logfile.open ("outfile.csv", std::ios::app);
    
    //interpolate as much as we can based on max items
    int num = 0;
    while(num < max_items){
        //pop off the next step if we are at the end of the current one
        if(interp.t_current > interp.tm || (interp.t_current == 0 && interp.tm == 0)){
            if(cb.size() > 1){
                interp.step = cb.front();
                cb.pop_front();
                step_t next = cb.front();
                
                //reset time and distance
                interp.t_current = 0;
                interp.dist = 0;
                
                //set the total time to complete the profile based on whether we have 3 or 7 periods
                if(interp.step.ad_profile.size() == 3) interp.tm = 4*interp.step.ad_profile.at(0) + 2*interp.step.ad_profile.at(1) + interp.step.ad_profile.at(2);
                else interp.tm = 4*interp.step.ad_profile.at(0) + 2*interp.step.ad_profile.at(1)+ interp.step.ad_profile.at(2) + interp.step.ad_profile.at(3) + 4*interp.step.ad_profile.at(4) + 2*interp.step.ad_profile.at(5) * interp.step.ad_profile.at(6);
                
                //total distance of line
                interp.dm = sqrt(pow(next.point[X_AXIS] - interp.step.point[X_AXIS], 2) + pow(next.point[Y_AXIS] - interp.step.point[Y_AXIS],2));
                
                //direction
                interp.theta = atan2(next.point[1] - interp.step.point[1], next.point[0] - interp.step.point[0]);

                interp.vs = interp.step.speed;
                interp.ve = next.speed;
            }
            else {
                logfile.close();
                return 0;
            }
        }
        else{
            //get speed
            if(interp.step.ad_profile.size() == 7) interp.speed =  _v_bar(interp.t_current, interp.step.ad_profile, interp.vs, interp.step.vm);
            else{
                if(interp.vs <= interp.ve) interp.speed = interp.vs + _v0(interp.t_current, interp.step.ad_profile);
                else interp.speed = interp.vs - _v0(interp.t_current, interp.step.ad_profile);
            }

            interp.dist = interp.dist + interp.speed * set.T;
            
            float destination[NUM_AXIS];
            destination[X_AXIS] = interp.step.point[X_AXIS] + cos(interp.theta) * interp.dist;
            destination[Y_AXIS] = interp.step.point[Y_AXIS] + sin(interp.theta) * interp.dist;

            //calculate the position we need to get to
            uint32_t delta[NUM_AXIS];
            calculate_delta(destination, delta);
            
            logfile << destination[X_AXIS] << "," << destination[Y_AXIS] << "," << interp.speed << "\n";
            interp.t_current = interp.t_current + set.T;
            
            num++;
        }
    }
    
    logfile.close();
    return 0;
};

bool motion_planner::plan_buffer(std::vector<float> &buf, float feedrate){
    if( cb.full() ){
        return 0;
    } //wait for space to open up in the buffer
    
    step_t step;
    std::copy(buf.begin(), buf.end(), step.point);
    step.feedrate = feedrate;
    
    cb.push_back(step);
    this->recalculate();
    return 1;
}

bool motion_planner::data_ready(){
    return cb.size() >= MIN_BUF_LEN;
}

bool motion_planner::full(){
    return cb.full();
}

bool motion_planner::empty(){
    return cb.size() == 0 || (size() == 1 && interp.t_current > interp.tm);
}

int motion_planner::size(){
    return cb.size();
}

void motion_planner::pop(void){
    cb.pop_front();
}

motion_planner::~motion_planner() {
}

