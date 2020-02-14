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






    this->gyro_properties.sensor_offset_init=Eigen::Vector3d::Zero();
    this->gyro_properties.sensor_drift_init<<0.00001,0.00001,0.00001;
    this->gyro_properties.sensor_drift_frequency_init<<1.0/3600.0,1.0/3600.0,1.0/3600.0;
    this->gyro_properties.sensor_gaussian_noise_init<<0.001,0.001,0.001;
    this->gyro_properties.sensor_scale_error_init=Eigen::Matrix3d::Identity();

    this->acc_properties.sensor_offset_init=Eigen::Vector3d::Zero();
    this->acc_properties.sensor_drift_init<<0.1,0.1,0.1;
    this->acc_properties.sensor_drift_frequency_init<<1.0/3600.0,1.0/3600.0,1.0/3600.0;
    this->acc_properties.sensor_gaussian_noise_init<<0.01,0.01,0.01;
    this->acc_properties.sensor_scale_error_init=Eigen::Matrix3d::Identity();






    accel.Load(acc_properties);
    gyro.Load(gyro_properties);









}
SensorSimulation::ImuSimulation::~ImuSimulation() {

}


void SensorSimulation::ImuSimulation::imu_simulation(raisim::Mat<3, 3> &imu_Rotation, raisim::Vec<3> &imu_AngularVelocity, raisim::Vec<3> &imu_Velocity,raisim::Vec<3> &imu_Velocity_bef,
                                 double &dt, raisim::World &world) {
    raisim::Vec<3> imu_gyro;
    raisim::Vec<3> imu_gyro_temp;
    raisim::Vec<3> imu_acc;
    raisim::Vec<3> imu_acc_temp;
    Eigen::Vector3d imu_acc_eigen;
    Eigen::Vector3d imu_gyro_eigen;
    imu_gyro_temp = imu_AngularVelocity;
    raisim::vecsub(imu_Velocity,imu_Velocity_bef,imu_acc_temp);
    imu_acc_temp/=dt;
    imu_acc_temp+=world.getGravity();

    raisim::matTransposevecmul(imu_Rotation,imu_gyro_temp,imu_gyro);
    raisim::matTransposevecmul(imu_Rotation,imu_acc_temp,imu_acc);
    imu_Velocity_bef = imu_Velocity;

    imu_gyro_eigen = imu_gyro.e();
    imu_acc_eigen = imu_acc.e();

    imu_gyro_eigen = gyro(imu_gyro_eigen,dt);
    imu_acc_eigen = accel(imu_acc_eigen,dt);



    for(int i=0; i<3;i++ )
    {
        this->imu_data[i] = imu_gyro_eigen[i];
        this->imu_data[i+3] = imu_acc_eigen[i];
    }

}

void SensorSimulation::ImuSimulation::imu_data_print() {


    std::cout<<"imu_gyro_x : "<<this->imu_data[0]<<std::endl;
    std::cout<<"imu_gyro_y : "<<this->imu_data[1]<<std::endl;
    std::cout<<"imu_gyro_z : "<<this->imu_data[2]<<std::endl;
    std::cout<<"imu_acc_x : "<<this->imu_data[3]<<std::endl;
    std::cout<<"imu_acc_y : "<<this->imu_data[4]<<std::endl;
    std::cout<<"imu_acc_z : "<<this->imu_data[5]<<std::endl<<std::endl;



}






