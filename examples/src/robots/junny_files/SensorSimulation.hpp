//
// Created by junny on 20. 2. 8..
//

#ifndef RAISIMOGRE_SENSORSIMULATION_HPP
#define RAISIMOGRE_SENSORSIMULATION_HPP


#include <raisim/OgreVis.hpp>
#include "eigenmvn.h"
#include "sensor_model.hpp"

namespace SensorSimulation
{








//    template<size_t n>
//    raisim::Vec<n> MultiVariateGaussianDistribution(Eigen::Matrix<double, n,1> &mean, Eigen::Matrix<double, n,n> &cov);
//


//
//
//    template <typename T>
//    T SensorModel_<T>::update(double dt)
//    {
//        for(std::size_t i = 0; i < current_error_.size(); ++i) current_error_[i] = SensorModelInternalUpdate(current_drift_[i], drift[i], drift_frequency[i], offset[i], gaussian_noise[i], dt);
//        return current_error_;
//    }
//
//    template <>
//    double SensorModel_<double>::update(double dt)
//    {
//        current_error_ = SensorModelInternalUpdate(current_drift_, drift, drift_frequency, offset, gaussian_noise, dt);
//        return current_error_;
//    }
//










    class ImuSimulation {





        private:

        sensor_properties<double, 3> gyro_properties;
        sensor_properties<double, 3> acc_properties;

        public:



            ImuSimulation();
            ~ImuSimulation();
            raisim::Vec<6> imu_data;
            raisim::Mat<3,3> imu_Rotation;
            raisim::Vec<3> imu_AngularVelocity;
            raisim::Vec<3> imu_Velocity;
            raisim::Vec<3> imu_Velocity_bef;
            raisim::Vec<3> imu_Position;

    //        raisim::Vec<3> imu_gyro_mean;
    //        raisim::Mat<3,3> imu_gyro_cov;
    //        raisim::Vec<3> imu_acc_mean;
    //        raisim::Mat<3,3> imu_acc_cov;
    //
    //        raisim::Vec<3> imu_gyro_bias_mean;
    //        raisim::Mat<3,3> imu_gyro_bias_cov;
    //        raisim::Vec<3> imu_acc_bias_mean;
    //        raisim::Mat<3,3> imu_acc_bias_cov;




            void imu_simulation(raisim::Mat<3, 3> &imu_Rotation, raisim::Vec<3> &imu_AngularVelocity, raisim::Vec<3> &imu_Velocity,raisim::Vec<3> &imu_Velocity_bef,
                                          double &dt, raisim::World &world);
            void imu_data_print();


    };








}





#endif //RAISIMOGRE_SENSORSIMULATION_HPP
