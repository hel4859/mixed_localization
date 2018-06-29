/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter/particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).

    //Set the number of particles.
    num_particles = 1000;

    //Initialize all particles to first position
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);

    default_random_engine gen;
    for(int i=0; i<num_particles; ++i){
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);
        p.weight = 1.0;
        particles.push_back(p);
        std::cout<<"particle "<<i<<"\tx: "<<p.x<<"\ty: "<<p.y<<std::endl;
    }
    weights.resize(num_particles, 1.0f);
    best_particle.id = -1;
    best_particle.weight = 1000;
    is_initialized = true;
}

void ParticleFilter::reset(double x, double y, double theta)
{
    //Initialize all particles to first position
    normal_distribution<double> dist_x(x, 0.3);
    normal_distribution<double> dist_y(y, 0.3);
    normal_distribution<double> dist_theta(theta, 0.0001);

    weights.resize(num_particles, 1.0f);

    default_random_engine gen;
    for(int i=0; i<num_particles; ++i){
        Particle p;
        p.id = i;
        p.x = dist_x(gen);
        p.y = dist_y(gen);
        p.theta = dist_theta(gen);



        double dx = p.x - x;
        double dy = p.y - y;

        // calculate normalization term
        double gauss_norm= (1.0/(2.0 * M_PI * 0.3 * 0.3));

        // calculate exponent
        double exponent = (dx*dx)/(2.0 * 0.3 * 0.3) + (dy*dy)/(2.0 * 0.3 * 0.3);

        // calculate weight using normalization terms and exponent
        double weight = gauss_norm * exp(-exponent);


        p.weight = weight;
        weights[i] = weight;
        particles[i] = p;
//        std::cout<<"particle "<<i<<"\tx: "<<p.x<<"\ty: "<<p.y<<std::endl;
    }
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

    default_random_engine gen;
    normal_distribution<double> dist_x(0, 0.0003);
    normal_distribution<double> dist_y(0, 0.0003);

    for(int i=0; i<num_particles; ++i){
        //predict
        double d_inc = velocity * delta_t;
        double yaw_inc = yaw_rate * delta_t;
        double noise_d = 0.08 * sqrtf(delta_t);
        double noise_yaw = 0.01 * sqrtf(delta_t);

        normal_distribution<double> dist_encoder_err(0, noise_d);
        normal_distribution<double> dist_yaw_err(0, noise_yaw);


        if(velocity>0.01){
            //add noise
            d_inc += dist_encoder_err(gen);
            yaw_inc += dist_yaw_err(gen);
        }

        if (fabs( yaw_rate ) < 0.0001) {
            particles[i].x += d_inc * cos( particles[i].theta );
            particles[i].y += d_inc* sin( particles[i].theta );
        }
        else{
            particles[i].x += ( d_inc / yaw_inc ) * ( sin( particles[i].theta + ( yaw_inc ) ) - sin( particles[i].theta ) );
            particles[i].y += ( d_inc / yaw_inc ) * ( cos( particles[i].theta ) - cos( particles[i].theta + ( yaw_inc ) ) );
            particles[i].theta += yaw_inc;
        }

        if(velocity>0.01){
            //add noise
            particles[i].x += dist_x(gen);
            particles[i].y += dist_y(gen);
        }

    }
}

void ParticleFilter::deadReckoning(double delta_t, double velocity, double yaw_rate){

    default_random_engine gen;
    normal_distribution<double> dist_x(0, 0.01);
    normal_distribution<double> dist_y(0, 0.01);

    for(int i=0; i<num_particles; ++i){
        //predict
        double d_inc = velocity * delta_t;
        double yaw_inc = yaw_rate * delta_t;
        double noise_d = 0.01 * sqrtf(delta_t);
        double noise_yaw = 0.001 * sqrtf(delta_t);

        normal_distribution<double> dist_encoder_err(0, noise_d);
        normal_distribution<double> dist_yaw_err(0, noise_yaw);


//        if(velocity>0.01){
//            //add noise
//            d_inc += dist_encoder_err(gen);
//            yaw_inc += dist_yaw_err(gen);
//        }

        if (fabs( yaw_rate ) < 0.0001) {
            particles[i].x += d_inc * cos( particles[i].theta );
            particles[i].y += d_inc* sin( particles[i].theta );
        }
        else{
            particles[i].x += ( d_inc / yaw_inc ) * ( sin( particles[i].theta + ( yaw_inc ) ) - sin( particles[i].theta ) );
            particles[i].y += ( d_inc / yaw_inc ) * ( cos( particles[i].theta ) - cos( particles[i].theta + ( yaw_inc ) ) );
            particles[i].theta += yaw_inc;
        }
    }
}

void ParticleFilter::processDelay(double delta_t, double velocity, double yaw_rate)
{
    for(int i=0; i<num_particles; ++i){
        //predict
        double d_inc = velocity * delta_t;
        double yaw_inc = yaw_rate * delta_t;

        if (fabs( yaw_rate ) < 0.0001) {
            particles[i].x += d_inc * cos( particles[i].theta );
            particles[i].y += d_inc* sin( particles[i].theta );
        }
        else{
            particles[i].x += ( d_inc / yaw_inc ) * ( sin( particles[i].theta + ( yaw_inc ) ) - sin( particles[i].theta ) );
            particles[i].y += ( d_inc / yaw_inc ) * ( cos( particles[i].theta ) - cos( particles[i].theta + ( yaw_inc ) ) );
            particles[i].theta += yaw_inc;
        }
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

    for(auto& ob_lm: observations){
        double dist_min = std::numeric_limits<double>::max();
        int map_id = -1;

        for(auto map_lm: predicted){
            double dist = dist_landmark(ob_lm,map_lm);
            if(dist < dist_min){
                dist_min = dist;
                map_id = map_lm.id;
            }
        }
        ob_lm.id = map_id;
    }
}

std::vector<LandmarkObs> ParticleFilter::getPredictLandMarks(const Particle& p, const double &sensor_range, const Map &map_landmarks)
{
    std::vector<LandmarkObs> predicted_landmarks;
    for(auto map_lm : map_landmarks.landmark_list){
        double d = dist(p.x,p.y,map_lm.x_f,map_lm.y_f);
        if(d<sensor_range){
            predicted_landmarks.push_back(LandmarkObs{ map_lm.id_i,map_lm.x_f,map_lm.y_f });
        }
    }
    return predicted_landmarks;
}

std::vector<LandmarkObs> ParticleFilter::transformToMap(const Particle &p, const std::vector<LandmarkObs>& observations)
{
    double dx = p.x;
    double dy = p.y;
    double cos_theta = cos(p.theta);
    double sin_theta = sin(p.theta);

    std::vector<LandmarkObs> transformed_landmarks;
    for(auto ob_lm : observations){
        double map_x = cos_theta * ob_lm.x - sin_theta * ob_lm.y + dx;
        double map_y = sin_theta * ob_lm.x + cos_theta * ob_lm.y + dy;
        transformed_landmarks.push_back(LandmarkObs{ob_lm.id, map_x, map_y});
    }

    return transformed_landmarks;
}

LandmarkObs ParticleFilter::getAssociateLandMark(const std::vector<LandmarkObs>& map_landmarks, const int& id)
{
    LandmarkObs associated_landmark;
    associated_landmark.id = -1;
    for(auto map_lm : map_landmarks)
        if(map_lm.id == id) associated_landmark = map_lm;
    return associated_landmark;
}

double ParticleFilter::computeWeight(const LandmarkObs& observation, const double std_landmark[], const LandmarkObs& map_landmark){
    double sig_x = std_landmark[0];
    double sig_y = std_landmark[1];
    double dx = observation.x - map_landmark.x;
    double dy = observation.y - map_landmark.y;

    // calculate normalization term
    double gauss_norm= (1.0/(2.0 * M_PI * sig_x * sig_y));

    // calculate exponent
    double exponent = (dx*dx)/(2.0*sig_x*sig_x) + (dy*dy)/(2.0*sig_y*sig_y);

    // calculate weight using normalization terms and exponent
    double weight = gauss_norm * exp(-exponent);

//    std::cout<<"dx: "<<dx<<"\tdy: "<<dy<<"\tsig_x: "<<sig_x<<"\tsig_y: "<<sig_y<<std::endl;
//    std::cout<<"gauss_norm: "<<gauss_norm<<"\texponent: "<<exponent<<"\tweight: "<<weight<<std::endl;

    return weight;
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

    for(int i=0; i<num_particles; ++i){
        std::vector<LandmarkObs> predicted_landmarks = getPredictLandMarks(particles[i],sensor_range,map_landmarks);

        std::vector<LandmarkObs> transformed_landmarks = transformToMap(particles[i], observations);

        dataAssociation(predicted_landmarks, transformed_landmarks);

        double W = 1.0;
        for(auto obs_lm : transformed_landmarks){

            auto pre_lm = getAssociateLandMark(predicted_landmarks, obs_lm.id);

            double w = computeWeight(obs_lm, std_landmark, pre_lm);
            W *= w;
        }
        particles[i].weight = W;
        weights[i] = particles[i].weight;
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

    std::discrete_distribution<int> d(weights.begin(), weights.end());
    std::vector<Particle> new_particles;
    default_random_engine gen;

    for(unsigned i = 0; i < num_particles; i++)
    {
        auto ind = d(gen);
//        std::cout<<"rand index: "<<ind<<"\tweight: "<<weights[ind]<<std::endl;
        new_particles.push_back(particles[ind]);
    }
    particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates

	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
