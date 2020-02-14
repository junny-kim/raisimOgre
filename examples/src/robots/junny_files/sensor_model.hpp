//
// Created by junny on 20. 2. 10..
//

#ifndef RAISIMOGRE_SENSOR_MODEL_HPP
#define RAISIMOGRE_SENSOR_MODEL_HPP


#include <numeric>
#include <stdio.h>
#include <raisim/OgreVis.hpp>


    template<typename T, int n>
    struct sensor_properties
    {
        Eigen::Matrix<T, n, 1> sensor_mean_init;
        Eigen::Matrix<T, n, 1> sensor_bias_mean_init;
        Eigen::Matrix<T, n, n> sensor_cov_init;
        Eigen::Matrix<T, n, n> sensor_bias_cov_init;


        Eigen::Matrix<T, n, 1> sensor_offset_init;
        Eigen::Matrix<T, n, 1> sensor_drift_init;
        Eigen::Matrix<T, n, 1> sensor_drift_frequency_init;
        Eigen::Matrix<T, n, 1> sensor_gaussian_noise_init;
        Eigen::Matrix<T, n, n> sensor_scale_error_init;
    };




    template<typename T, int n>
    class SensorModel_ {
    public:
        SensorModel_();

        virtual ~SensorModel_();

        virtual void Load(sensor_properties<T,n> sensor_initial);
//        virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = std::string());


        virtual Eigen::Matrix<T, n, 1> operator()(const Eigen::Matrix<T, n, 1> &value) const { return scale_error * value  + current_error_; }

        virtual Eigen::Matrix<T, n, 1> operator()(const Eigen::Matrix<T, n, 1> &value, double dt) { return scale_error * value  + update(dt); }

        virtual Eigen::Matrix<T, n, 1> update(double dt);

        virtual void reset();

        virtual void reset(Eigen::Matrix<T, n, 1> &value);

        virtual const Eigen::Matrix<T, n, 1> &getCurrentError() const { return current_error_; }

        virtual Eigen::Matrix<T, n, 1> getCurrentBias() const { return current_drift_ + offset; }

        virtual const Eigen::Matrix<T, n, 1> &getCurrentDrift() const { return current_drift_; }

        virtual const Eigen::Matrix<T, n, n> &getScaleError() const { return scale_error; }

        virtual void setCurrentDrift(const Eigen::Matrix<T, n, 1> &new_drift) { current_drift_ = new_drift; }

    private:
//        virtual bool LoadImpl(sdf::ElementPtr _element, T& _value);

    public:
        Eigen::Matrix<T, n, 1> offset;
        Eigen::Matrix<T, n, 1> drift;
        Eigen::Matrix<T, n, 1> drift_frequency;
        Eigen::Matrix<T, n, n> scale_error;
        Eigen::Matrix<T, n, 1> gaussian_noise;

    private:
        Eigen::Matrix<T, n, 1> current_drift_;
        Eigen::Matrix<T, n, 1> current_error_;
    };

    template<typename T, int n>

    SensorModel_<T,n>::SensorModel_() {


//        for (int i=0;i<scale_error.size(); i++)
//        {
//            offset[i]=(T)0.0;
//            drift[i]=(T)0.0;
//            gaussian_noise[i]=(T)0.0;
//            drift_frequency[i] = (T)(1.0/3600.0);
//            scale_error[i] = (T)(1.0);
//        }
        offset = Eigen::VectorXd::Zero(n);
        drift = Eigen::VectorXd::Zero(n);
        drift_frequency = Eigen::VectorXd::Zero(n);
        gaussian_noise = Eigen::VectorXd::Zero(n);
//        scale_error = Eigen::VectorXd::Zero(n);
//
//

        for (int i=0;i<drift_frequency.size(); i++)
        {
            drift_frequency[i] = (T)(1.0/3600.0);
        }

//        drift_frequency = Eigen::VectorXd::Constant(1.0 / 3600.0); // time constant 1h
        scale_error = Eigen::MatrixXd::Identity(n,n);
        reset();
    }

    template<typename T, int n>
    SensorModel_<T,n>::~SensorModel_() {
    }

    template<typename T, int n>
    void SensorModel_<T,n>::Load(sensor_properties<T,n> sensor_initial) {

        offset=sensor_initial.sensor_offset_init;
        drift=sensor_initial.sensor_drift_init;
        drift_frequency=sensor_initial.sensor_drift_frequency_init;
        gaussian_noise=sensor_initial.sensor_gaussian_noise_init;
        scale_error=sensor_initial.sensor_scale_error_init;
        reset();
    }

//    template<typename T, int n>
//    bool SensorModel_<T,n>::LoadImpl(sdf::ElementPtr _element, T &_value) {
//        if (!_element->GetValue()) return false;
//        return _element->GetValue()->Get(_value);
//    }

    namespace {
        template<typename T>
        static inline T SensorModelGaussianKernel(T mu, T sigma) {
            // using Box-Muller transform to generate two independent standard normally distributed normal variables
            // see wikipedia
            T U = (T) rand() / (T) RAND_MAX; // normalized uniform random variable
            T V = (T) rand() / (T) RAND_MAX; // normalized uniform random variable
            T X = sqrt(-2.0 * log(U)) * cos(2.0 * M_PI * V);
            X = sigma * X + mu;
            return X;
        }

        template<typename T>
        static inline T
        SensorModelInternalUpdate(T &current_drift, T drift, T drift_frequency, T offset, T gaussian_noise, double dt) {
            // current_drift = current_drift - dt * (current_drift * drift_frequency + SensorModelGaussianKernel(T(), sqrt(2*drift_frequency)*drift));
            current_drift = exp(-dt * drift_frequency) * current_drift + dt * SensorModelGaussianKernel((T)0.0, sqrt(2 * drift_frequency) * drift);
            return offset + current_drift + SensorModelGaussianKernel((T)0.0, gaussian_noise);
        }
    }

template<typename T, int n>
Eigen::Matrix<T, n, 1> SensorModel_<T,n>::update(double dt) {
        for (std::size_t i = 0; i < current_error_.size(); ++i)
            current_error_[i] = SensorModelInternalUpdate(current_drift_[i], drift[i], drift_frequency[i], offset[i],
                                                          gaussian_noise[i], dt);
        return current_error_;
    }



template<typename T, int n>
    void SensorModel_<T,n>::reset() {
        for (std::size_t i = 0; i < current_drift_.size(); ++i)
        {
            current_drift_[i] = SensorModelGaussianKernel((T)0.0, drift[i]);
            current_error_[i] = (T)0.0;
        }

    }



template<typename T, int n>
    void SensorModel_<T,n>::reset(Eigen::Matrix<T, n, 1> &value) {

    for (std::size_t i = 0; i < current_drift_.size(); ++i) {
        current_drift_[i] = value[i];
        current_error_[i] = (T) 0.0;
    }

}
    typedef SensorModel_<double, 1> SensorModel;
    typedef SensorModel_<double, 3> SensorModel3;





#endif //RAISIMOGRE_SENSOR_MODEL_HPP