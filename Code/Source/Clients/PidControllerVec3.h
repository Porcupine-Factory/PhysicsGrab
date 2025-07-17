#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>

namespace ObjectInteraction
{
    class PidControllerVec3
    {
    public:
        PidControllerVec3(float p, float i, float d, int errorNum)
            : m_p(p), m_i(i), m_d(d)
        {
            SetErrorNum(errorNum);
            Reset();
        }

        AZ::Vector3 Output(const AZ::Vector3& error, float deltaTime)
        {
            m_errors[m_errorIndex] = error;
            m_timeSteps[m_errorIndex] = deltaTime;

            AZ::Vector3 integral = AZ::Vector3::CreateZero();
            for (size_t i = 0; i < m_errors.size(); ++i)
            {
                integral += m_errors[i] * m_timeSteps[i];
            }

            AZ::Vector3 derivative = AZ::Vector3::CreateZero();
            if (m_lastIndex >= 0)
            {
                derivative = (m_errors[m_errorIndex] - m_errors[m_lastIndex]) / deltaTime;
            }

            m_lastIndex = m_errorIndex;
            m_errorIndex = (m_errorIndex + 1) % m_errors.size();

            return error * m_p + integral * m_i + derivative * m_d;
        }

        void SetErrorNum(int num)
        {
            m_errors.resize(num, AZ::Vector3::CreateZero());
            m_timeSteps.resize(num, 0.0f);
        }

        void Reset()
        {
            m_errorIndex = 0;
            m_lastIndex = -1;
            m_timeSteps.assign(m_timeSteps.size(), 0.0f);
        }

    private:
        float m_p, m_i, m_d;
        AZStd::vector<AZ::Vector3> m_errors;
        AZStd::vector<float> m_timeSteps;
        int m_errorIndex = 0;
        int m_lastIndex = -1;
    };
}