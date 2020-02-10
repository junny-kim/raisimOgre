//
// Created by junny on 20. 2. 8..
//

#include "SensorSimulation.hpp"
#include <raisim/OgreVis.hpp>


SensorSimulation::ImuSimulation::ImuSimulation() {


    this->gyro_properties.sensor_mean_init << 1.0,1.0,1.0;
    this->acc_properties.sensor_mean_init << 1.0,1.0,1.0;
    this->gyro_properties.sensor_bias_mean_init << 1.0,1.0,1.0;
    this->acc_properties.sensor_bias_mean_init << 1.0,1.0,1.0;

    this->gyro_properties.sensor_cov_init << 1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0;

    this->acc_properties.sensor_cov_init << 1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0;

    this->gyro_properties.sensor_bias_cov_init << 1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0;

    this->acc_properties.sensor_bias_cov_init << 1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0;



}
SensorSimulation::ImuSimulation::~ImuSimulation() {

}
void SensorSimulation::ImuSimulation::imu_simulation(raisim::Mat<3, 3> &imu_Rotation, raisim::Vec<3> &imu_AngularVelocity, raisim::Vec<3> &imu_Velocity,raisim::Vec<3> &imu_Velocity_bef,
                                 double &dt, raisim::World &world) {
    raisim::Vec<3> imu_gyro;
    raisim::Vec<3> imu_gyro_temp;
    raisim::Vec<3> imu_acc;
    raisim::Vec<3> imu_acc_temp;

    imu_gyro_temp = imu_AngularVelocity;
    raisim::vecsub(imu_Velocity,imu_Velocity_bef,imu_acc_temp);
    imu_acc_temp/=dt;
    imu_acc_temp+=world.getGravity();

    raisim::matTransposevecmul(imu_Rotation,imu_gyro_temp,imu_gyro);
    raisim::matTransposevecmul(imu_Rotation,imu_acc_temp,imu_acc);
    imu_Velocity_bef = imu_Velocity;

    for(int i=0; i<3;i++ )
    {
        this->imu_data[i] = imu_gyro[i];
        this->imu_data[i+3] = imu_acc[i];
    }

}

void SensorSimulation::ImuSimulation::imu_data_print() {


//    std::cout<<"imu_gyro_x : "<<this->imu_data[0]<<std::endl;
//    std::cout<<"imu_gyro_y : "<<this->imu_data[1]<<std::endl;
//    std::cout<<"imu_gyro_z : "<<this->imu_data[2]<<std::endl;
//    std::cout<<"imu_acc_x : "<<this->imu_data[3]<<std::endl;
//    std::cout<<"imu_acc_y : "<<this->imu_data[4]<<std::endl;
//    std::cout<<"imu_acc_z : "<<this->imu_data[5]<<std::endl<<std::endl;

    double mu=0.0;
    double sigma=0.05;
    std::cout<<SensorSimulation::SensorModelGaussianKernel(mu, sigma)<<std::endl;


}



//template<size_t n>
//raisim::Vec<n> SensorSimulation::MultiVariateGaussianDistribution(Eigen::Matrix<double, n,1> &mean, Eigen::Matrix<double, n,n> &cov) {
//
//
//
//
//
//
////    current_error_.X() = SensorModelInternalUpdate(current_drift_.X(), drift.X(), drift_frequency.X(), offset.X(), gaussian_noise.X(), dt);
////    current_error_.Y() = SensorModelInternalUpdate(current_drift_.Y(), drift.Y(), drift_frequency.Y(), offset.Y(), gaussian_noise.Y(), dt);
////    current_error_.Z() = SensorModelInternalUpdate(current_drift_.Z(), drift.Z(), drift_frequency.Z(), offset.Z(), gaussian_noise.Z(), dt);
//
//
//
//
//
//
//
//
//    Eigen::EigenMultivariateNormal<double> solver(mean,cov);
//    return raisim::Vec<n>(solver.samples(1));
//
//
//}


