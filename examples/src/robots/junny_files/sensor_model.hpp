//
// Created by junny on 20. 2. 10..
//

#ifndef RAISIMOGRE_SENSOR_MODEL_HPP
#define RAISIMOGRE_SENSOR_MODEL_HPP


#include <numeric>
#include <stdio.h>
#include <raisim/OgreVis.hpp>
#include <sdf/sdf.hh>



    template<typename T, size_t n>
    struct sensor_properties
    {
        const Eigen::Matrix<T, n, 1> sensor_mean_init;
        const Eigen::Matrix<T, n, 1> sensor_bias_mean_init;
        const Eigen::Matrix<T, n, n> sensor_cov_init;
        const Eigen::Matrix<T, n, n> sensor_bias_cov_init;


        const Eigen::Matrix<T, n, 1> sensor_offset_init;
        const Eigen::Matrix<T, n, 1> sensor_drift_init;
        const Eigen::Matrix<T, n, 1> sensor_drift_frequency_init;
        const Eigen::Matrix<T, n, 1> sensor_gaussian_noise_init;
        const Eigen::Matrix<T, n, 1> sensor_scale_error_init;





    };




    template<typename T, size_t n>
    class SensorModel_ {
    public:
        SensorModel_();

        virtual ~SensorModel_();

        virtual void Load(sensor_properties<T,n> sensor_initial);
//        virtual void Load(sdf::ElementPtr _sdf, const std::string& prefix = std::string());


        virtual T operator()(const T &value) const { return value * scale_error + current_error_; }

        virtual T operator()(const T &value, double dt) { return value * scale_error + update(dt); }

        virtual Eigen::Matrix<T, n, 1> update(double dt);

        virtual void reset();

        virtual void reset(Eigen::Matrix<T, n, 1> &value);

        virtual const T &getCurrentError() const { return current_error_; }

        virtual T getCurrentBias() const { return current_drift_ + offset; }

        virtual const T &getCurrentDrift() const { return current_drift_; }

        virtual const T &getScaleError() const { return scale_error; }

        virtual void setCurrentDrift(const T &new_drift) { current_drift_ = new_drift; }

    private:
//        virtual bool LoadImpl(sdf::ElementPtr _element, T& _value);

    public:
        const Eigen::Matrix<T, n, 1> offset;
        const Eigen::Matrix<T, n, 1> drift;
        const Eigen::Matrix<T, n, 1> drift_frequency;
        const Eigen::Matrix<T, n, 1> gaussian_noise;
        const Eigen::Matrix<T, n, 1> scale_error;

    private:
        const Eigen::Matrix<T, n, 1> current_drift_;
        const Eigen::Matrix<T, n, 1> current_error_;
    };

    template<typename T, size_t n>
    SensorModel_<T,n>::SensorModel_() {
        offset = Eigen::VectorXd::Zero(n);
        drift = Eigen::VectorXd::Zero(n);
        drift_frequency = Eigen::VectorXd::Zero(n);
        gaussian_noise = Eigen::VectorXd::Zero(n);
        scale_error = Eigen::VectorXd::Zero(n);


        drift_frequency = Eigen::VectorXd::Constant(1.0 / 3600.0); // time constant 1h
        scale_error = Eigen::VectorXd::Constant(1.0);
        reset();
    }

    template<typename T, size_t n>
    SensorModel_<T,n>::~SensorModel_() {
    }

    template<typename T, size_t n>
    void SensorModel_<T,n>::Load(sensor_properties<T,n> sensor_initial) {


            memcpy(offset,sensor_initial.sensor_offset_init);
            memcpy(drift,sensor_initial.sensor_drift_init);
            memcpy(drift_frequency,sensor_initial.sensor_drift_frequency_init);
            memcpy(gaussian_noise,sensor_initial.sensor_gaussian_noise_init);
            memcpy(scale_error,sensor_initial.sensor_scale_error_init);





        reset();
    }

//    template<typename T, size_t n>
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
            T X = sqrt(-2.0 * ::log(U)) * cos(2.0 * M_PI * V);
            X = sigma * X + mu;
            return X;
        }

        template<typename T>
        static inline T
        SensorModelInternalUpdate(T &current_drift, T drift, T drift_frequency, T offset, T gaussian_noise, double dt) {
            // current_drift = current_drift - dt * (current_drift * drift_frequency + SensorModelGaussianKernel(T(), sqrt(2*drift_frequency)*drift));
            current_drift = exp(-dt * drift_frequency) * current_drift +
                            dt * SensorModelGaussianKernel(T(), sqrt(2 * drift_frequency) * drift);
            return offset + current_drift + SensorModelGaussianKernel(T(), gaussian_noise);
        }
    }

template<typename T, size_t n>
Eigen::Matrix<T, n, 1> SensorModel_<T,n>::update(double dt) {
        for (std::size_t i = 0; i < current_error_.size(); ++i)
            current_error_[i] = SensorModelInternalUpdate(current_drift_[i], drift[i], drift_frequency[i], offset[i],
                                                          gaussian_noise[i], dt);
        return current_error_;
    }



template<typename T, size_t n>
    void SensorModel_<T,n>::reset() {
        for (std::size_t i = 0; i < current_drift_.size(); ++i)
        {
            current_drift_[i] = SensorModelGaussianKernel(T::value_type(), drift[i]);
            current_error_[i] = (T)0.0;
        }

    }



template<typename T, size_t n>
    void SensorModel_<T,n>::reset(Eigen::Matrix<T, n, 1> &value) {

        for (std::size_t i = 0; i < current_drift_.size(); ++i)
    {
        current_drift_[i] = value[i];
        current_error_[i] = (T)0.0;
    }



    typedef SensorModel_<double,1> SensorModel;
    typedef SensorModel_<double,3> SensorModel3;












#endif //RAISIMOGRE_SENSOR_MODEL_HPP
