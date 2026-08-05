#include "Motor.h"
#include "RobotIO.h"

Motor::Motor()
{
    m_eState = motor::eState::STATE_START;
    m_eCommand = motor::COMMAND_NONE;
}

void Initailize()
{
    
}

void Motor::UpdateInputStatus()
{

}

void Motor::Execute()
{

    // Check if m_pRobotIO has been assigned
    if (m_pRobotIO != nullptr)
    {
    
        // ***************
        // * Start State *
        // ***************

        if (m_eState == motor::eState::STATE_START)
        {
            m_eState = motor::eState::STATE_IDLE;
        }
        
        // **************
        // * Idle State *
        // **************

        if (m_eState == motor::eState::STATE_IDLE)
        {
            
            // *--------------*
            // * Home Command *
            // *--------------*
            if (m_eCommand = motor::COMMAND_HOME)
            {
                if (m_pRobotIO->GetMotorLimit() )
                {
                    m_eCommand = motor::COMMAND_NONE;
                }
                return;

                // Apply Motor Configs
                m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                // Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set Motor Speed
                m_pRobotIO->m_Motor.Set(motor::dHomingSpeed);

                // Transition to Homing State
                m_eState = motor::eState::STATE_HOME;

            }

            // *------------------------*
            // * Manual Forward Command *
            // *------------------------*
            else if (m_eCommand = motor::COMMAND_MANUAL_FORWARD)
            {
                // Apply Motor Configs
                m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                // Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();
            
                // Set Motor Speed
                m_pRobotIO->m_Motor.Set(motor::dManualForwardSpeed);

                // Transition to Manual Forward State
                m_eState = motor::eState::STATE_MANUAL_FORWARD;
            }
            
            // *------------------------*
            // * Manual Reverse Command *
            // *------------------------*
            else if (m_eCommand = motor::COMMAND_MANUAL_REVERSE)
            {
               // Apply Motor Configs
               m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
               m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);
               
               // Reset ans Start Timer
               m_pTimeoutTimer->Reset();
               m_pTimeoutTimer->Start();
            }
            
            // *----------------------*
            // * Auto Forward Command *
            // *----------------------*
            else if (m_eCommand = motor::COMMAND_AUTO_FORWARD)
            {
                
            }
            
        }
        
        // ****************
        // * Homing State *
        // ****************

        // ************************
        // * Manual Forward State *
        // ************************

        // ************************
        // * Manual Reverse State *
        // ************************

        // **********************
        // * Auto Forward State *
        // **********************


        // Handle Error or Unknown State
        else
        {
            // Error code here if needed
            printf("Error in Motor.cpp - Error or Unknown State");
        }
        
    }
    else
    {
        // Handle RobotIO nullptr error
        printf("Error in Motor.cpp - Robot IO Pointer Null\n");
    }

}

