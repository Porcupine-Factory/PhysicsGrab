#pragma once

#include <AzCore/Math/Vector2.h>
#include <AzCore/Math/Vector3.h>

namespace ObjectInteraction
{
    template<class T>
    class PidController
    {
    public:
        enum DerivativeMode
        {
            ErrorRate,
            Velocity
        };

        // Default constructor delegates to the parameterized constructor with default values
        PidController()
            : PidController(0.0f, 0.0f, 0.0f, 100.0f, 0.8f, ErrorRate)
        {
        }

        PidController(
            float p, float i, float d, float integralLimit = 100.0f, float derivFilterAlpha = 0.7f, DerivativeMode mode = ErrorRate)
            : m_p(p)
            , m_i(i)
            , m_d(d)
            , m_integralLimit(integralLimit)
            , m_derivFilterAlpha(derivFilterAlpha)
            , m_mode(mode)
        {
            Reset();
        }

        DerivativeMode GetMode() const
        {
            return m_mode;
        }

        T Output(const T& error, float deltaTime, const T& currentValue)
        {
            if (!m_initialized)
            {
                m_initialized = true;
                m_prevError = error;
                m_prevValue = currentValue;
                m_integral = Zero();
                m_prevDerivative = Zero();
                return error * m_p + m_integral; // Skip derivative on first call
            }

            // Proportional
            T proportional = error * m_p;

            // Integral (cumulative, with anti-windup clamp)
            m_integral += error * deltaTime * m_i;
            m_integral = Clamp(m_integral, -m_integralLimit, m_integralLimit);

            // Derivative
            T raw_deriv;
            if (m_mode == Velocity)
            {
                T value_rate = (currentValue - m_prevValue) / deltaTime;
                raw_deriv = -value_rate;
            }
            else
            {
                raw_deriv = (error - m_prevError) / deltaTime;
            }

            // Add low-pass filter via lerp for stability
            T derivative = m_prevDerivative.Lerp(raw_deriv * m_d, m_derivFilterAlpha);
            m_prevDerivative = derivative;

            m_prevError = error;
            m_prevValue = currentValue;

            // Output as force (P/I/D scaled appropriately)
            return proportional + m_integral + derivative;
        }

        void Reset()
        {
            m_integral = Zero();
            m_prevError = Zero();
            m_prevDerivative = Zero();
            m_prevValue = Zero();
            m_initialized = false;
        }

    private:
        float m_p, m_i, m_d;
        float m_integralLimit;
        float m_derivFilterAlpha;
        DerivativeMode m_mode;

        T m_integral;
        T m_prevError;
        T m_prevDerivative;
        T m_prevValue;
        bool m_initialized;

        // Helper: Zero value for T (specialized below)
        static T Zero()
        {
            return T(0.0f);
        }

        // Clamp helper (vector/scalar overload via operators)
        static T Clamp(const T& val, float min, float max)
        {
            return val.GetClamped(T(min), T(max));
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