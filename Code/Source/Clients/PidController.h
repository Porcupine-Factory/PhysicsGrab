#pragma once

#include <AzCore/Math/Vector2.h>
#include <AzCore/Math/Vector3.h>

namespace ObjectInteraction
{
    template<class T>
    class PidController
    {
    public:
        PidController(float p, float i, float d, float integralLimit = 100.0f, float derivFilterAlpha = 0.7f)
            : m_p(p)
            , m_i(i)
            , m_d(d)
            , m_integralLimit(integralLimit)
            , m_derivFilterAlpha(derivFilterAlpha)
        {
            Reset();
        }

        T Output(const T& error, float deltaTime)
        {
            // Proportional
            T proportional = error * m_p;

            // Integral (cumulative, with anti-windup clamp)
            m_integral += error * deltaTime * m_i;
            m_integral = Clamp(m_integral, -m_integralLimit, m_integralLimit);

            // Derivative (with low-pass filter for stability)
            T raw_deriv = (error - m_prevError) / deltaTime;
            T derivative = m_prevDerivative.Lerp(raw_deriv * m_d, m_derivFilterAlpha);
            m_prevDerivative = derivative;

            m_prevError = error;

            // Output as force (P/D/I scaled appropriately)
            return proportional + m_integral + derivative;
        }

        void Reset()
        {
            m_integral = Zero();
            m_prevError = Zero();
            m_prevDerivative = Zero();
        }

    private:
        float m_p, m_i, m_d;
        float m_integralLimit;
        float m_derivFilterAlpha;

        T m_integral;
        T m_prevError;
        T m_prevDerivative;

        // Helper: Zero value for T (specialized below)
        static T Zero()
        {
            return T(0.0f);
        }

        // Clamp helper (vector/scalar overload via operators)
        static T Clamp(const T& val, float min, float max)
        {
            return val.GetClamped(T(min), T(max)); // Assumes T has GetClamped or component-wise clamp
        }
    };

    // Specialize Zero/Clamp for Vector2
    template<>
    inline AZ::Vector2 PidController<AZ::Vector2>::Zero()
    {
        return AZ::Vector2::CreateZero();
    }

    template<>
    inline AZ::Vector2 PidController<AZ::Vector2>::Clamp(const AZ::Vector2& val, float min, float max)
    {
        return AZ::Vector2(AZ::GetClamp(val.GetX(), min, max), AZ::GetClamp(val.GetY(), min, max));
    }

    // Specialize Zero/Clamp for Vector3
    template<>
    inline AZ::Vector3 PidController<AZ::Vector3>::Zero()
    {
        return AZ::Vector3::CreateZero();
    }

    template<>
    inline AZ::Vector3 PidController<AZ::Vector3>::Clamp(const AZ::Vector3& val, float min, float max)
    {
        return AZ::Vector3(AZ::GetClamp(val.GetX(), min, max), AZ::GetClamp(val.GetY(), min, max), AZ::GetClamp(val.GetZ(), min, max));
    }

    // For float (scalar)
    template<>
    inline float PidController<float>::Zero()
    {
        return 0.0f;
    }

    template<>
    inline float PidController<float>::Clamp(const float& val, float min, float max)
    {
        return AZ::GetClamp(val, min, max);
    }
}; // namespace ObjectInteraction