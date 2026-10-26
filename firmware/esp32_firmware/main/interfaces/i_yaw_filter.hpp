#ifndef I_YAW_FILTER_HPP
#define I_YAW_FILTER_HPP

namespace holodex {

class IYawFilter {
public:
    virtual ~IYawFilter() = default;

    // Reinicia o estado do filtro com uma orientação inicial (radianos)
    virtual void reset(float initial_theta = 0.0f) = 0;

    // Etapa de predição temporal baseada no giroscópio Z (rad/s)
    virtual void predict(float gyro_z, float dt_s) = 0;

    // Etapa de correção usando a velocidade angular da odometria das rodas (rad/s)
    virtual void updateWheelOmega(float wheel_omega, float gyro_z) = 0;

    // Retorna a orientação estimada de Yaw normalizada [-pi, pi] (radianos)
    virtual float getTheta() const = 0;

    // Retorna a estimativa atual do bias do giroscópio (rad/s)
    virtual float getBias() const = 0;

    // Indica se o filtro está inicializado
    virtual bool isReady() const = 0;
};

} // namespace holodex

#endif // I_YAW_FILTER_HPP
