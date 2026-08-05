#pragma once
#include <frc/Timer.h>
#include "RobotIO.h"

namespace motor
{
    enum eState
    {
        STATE_START = 0,
        STATE_HOME = 1,
        STATE_IDLE = 2,
        STATE_MANUAL_FORWARD = 3,
        STATE_MANUAL_REVERSE = 4,
        STATE_AUTO_FORWARD = 5,
        STATE_ERROR = 99
    };

    enum eCommand
    {
        COMMAND_NONE,
        COMMAND_HOME,
        COMMAND_MANUAL_FORWARD,
        COMMAND_MANUAL_REVERSE,
        COMMAND_AUTO_FORWARD,
        COMMAND_STOP,
    };
    // Timeout Constants
    static constexpr double dHomingTimeout;
    static constexpr double dManualForwardTimeout;
    static constexpr double dManualReverseTimeout;
    static constexpr double dAutoForwardTimeout;

    // Motor Speed Constanst
    static constexpr double dHomingSpeed;
    static constexpr double dManualForwardSpeed;
    static constexpr double dManualreverseSpeed;
    static constexpr double dAutoForwardSpeed;

    // Setpoint Constants
    static constexpr double dHomingSetpoint;
}

class Motor
{
    public:

        // Constructor/Destructor
     Motor();
        ~Motor()
        {   }

        // Accessor Methods
        inline void Home()
        {m_eCommand = motor::COMMAND_HOME;}

        inline void ManualForward()
        {m_eCommand = motor::COMMAND_MANUAL_FORWARD;}

        inline void ManualReverse()
        {m_eCommand = motor::COMMAND_MANUAL_REVERSE;}

        inline void AutoForward()
        {m_eCommand = motor::COMMAND_AUTO_FORWARD;}


        inline bool IsIdle()
        {return(m_eState == motor::eState::STATE_IDLE);}

        inline bool IsHoming()
        {return(m_eState == motor::eState::STATE_HOME);}

        inline bool IsManualForward()
        {return(m_eState == motor::eState::STATE_MANUAL_FORWARD);}

        inline bool IsManualReverse()
        {return(m_eState == motor::eState::STATE_MANUAL_REVERSE);}

        inline bool IsAutoForward()
        {return(m_eState == motor::eState::STATE_AUTO_FORWARD);}

        void Initialize( RobotIO *pRobotIO);
        void Execute();
        void UpdateInputStatus();

    private:

    motor::eCommand m_eCommand;
    motor::eState m_eState;

    RobotIO *m_pRobotIO;

    frc::Timer *m_pTimeoutTimer;

    configs::MotorOutputConfigs m_MotorConfigs;
};