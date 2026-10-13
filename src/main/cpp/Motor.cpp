#include "Motor.h"
#include "RobotIO.h"

Motor::Motor()
{
    m_eState = motor::eState::STATE_START;
    m_eCommand = motor::eCommand::COMMAND_NONE;

    m_pRobotIO = nullptr;
}

void Motor::Initialize(RobotIO *p_RobotIO)
{
   m_pRobotIO = p_RobotIO;
    m_pTimeoutTimer = new::frc::Timer();
    m_pTimeoutTimer->Reset();
    m_pRobotIO->m_Motor.GetConfigurator().Refresh(m_MotorConfigs);
    
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
            m_eState = motor::eState::STATE_MOTOR_IDLE;
        }
        
        // **************
        // * Idle State *
        // **************
        else if (m_eState == motor::eState::STATE_MOTOR_IDLE)
        {
            // *--------------*
            // * Home Command *
            // *--------------*
            if (m_eCommand = motor::COMMAND_MOTOR_HOME)
            {
                if  (m_pRobotIO->GetMotorLimit() )
                {
                    m_eCommand = motor::COMMAND_NONE;
                    return;
                }

                // Apply Motor Configs
                m_MotorConfigs.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                // Reset and Start Timer
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Set Motor Speed
                m_pRobotIO->m_Motor.Set(motor::dMotorHomingSpeed);

                // Transition to Homing State
                m_eState = motor::eState::STATE_MOTOR_HOME;

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
                if (m_pRobotIO->GetMotorLimit())
                {
                    m_eCommand = motor::COMMAND_NONE;
                    return;
                }
                

               // Apply Motor Configs
                m_MotorConfigs.NeutralMode.Coast;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);
               
               // Reset ans Start Timer
               m_pTimeoutTimer->Reset();
               m_pTimeoutTimer->Start();

               // Set Motor Speed
               m_pRobotIO->m_Motor.Set(motor::dManualreverseSpeed);

               // Transition To Manual Reverse State
               m_eState = motor::eState::STATE_MANUAL_REVERSE;
            }
            
            // *----------------------*
            // * Auto Forward Command *
            // *----------------------*
            else if (m_eCommand = motor::COMMAND_AUTO_FORWARD)
            {
                if (m_pRobotIO->m_Motor.GetPosition().GetValueAsDouble() >= motor::dAutoForwardSetpoint)
                {
                    m_eCommand = motor::COMMAND_NONE;
                    return;
                }
                
                // Apply Motor configs
                m_MotorConfigs.NeutralMode.Coast;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                // Reset and Start
                m_pTimeoutTimer->Reset();
                m_pTimeoutTimer->Start();

                // Transition to AutoForward State
                m_eState = motor::eState::STATE_AUTO_FORWARD;
            }
            
        }
        
        // ****************
        // * Homing State *
        // ****************
        else if (m_eState = motor::eState::STATE_MOTOR_HOME)
        {
          bool bIsTimedOut = false;
          
          if ((double)m_pTimeoutTimer->Get() >= motor::dMotorHomingTimeout)
          {
            bIsTimedOut = true;
          }
          
          if (m_pRobotIO->GetMotorLimit() || bIsTimedOut)
          {
                // Stop Motors
                m_pRobotIO->m_Motor.StopMotor();

                //Enable Break Mode
                m_MotorConfigs.NeutralMode.Coast;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                //Reset Encoders
                m_pRobotIO->m_Motor.SetPosition(units::angle::turn_t{0});

                m_eCommand = motor::COMMAND_NONE;
                m_eState = motor::eState::STATE_MOTOR_IDLE;
          }
          
        }
        
        // ************************
        // * Manual Forward State *
        // ************************
        else if (m_eState = motor::eState::STATE_MANUAL_FORWARD)
        {
           bool bIsTimedOut = false;
           
           if ((double)m_pTimeoutTimer->Get() >= motor::dManualForwardTimeout)
           {
                bIsTimedOut = true;
           }
           if (m_eCommand == motor::COMMAND_NONE || bIsTimedOut)
           {
                // Stop Motors
                m_pRobotIO->m_Motor.StopMotor();

                // Enable Break Mode
                m_MotorConfigs.NeutralMode.Brake;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                m_eCommand = motor::COMMAND_NONE;
                m_eState = motor::eState::STATE_MOTOR_IDLE;
           }
           
        }
        
        // ************************
        // * Manual Reverse State *
        // ************************
        else if (m_eState = motor::eState::STATE_MANUAL_REVERSE)
        {
            bool bIsTimedOut = false;
            if ((double)m_pTimeoutTimer->Get() >= motor::dManualReverseTimeout)
            {
                bIsTimedOut = true;
            }

            if (m_eCommand == motor::COMMAND_STOP || m_pRobotIO->GetMotorLimit() || bIsTimedOut)
            {
                // Stop Motors
                m_pRobotIO->m_Motor.StopMotor();

                // Enable Break Mode
                m_MotorConfigs.NeutralMode.Coast;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                m_eCommand = motor::COMMAND_NONE;
                m_eState = motor::eState::STATE_MOTOR_IDLE;
            }
        }
        
        // **********************
        // * Auto Forward State *
        // **********************
        else if (m_eState = motor::eState::STATE_AUTO_FORWARD)
        {
          bool bIsTimedOut = false;  

           if ((double)m_pTimeoutTimer->Get() >= motor::dAutoForwardTimeout)
            {
                bIsTimedOut = true;
            }

            if (m_pRobotIO->m_Motor.GetPosition().GetValueAsDouble() >= motor::dAutoForwardSetpoint || bIsTimedOut)
            {
                // Stop Motors
                m_pRobotIO->m_Motor.StopMotor();

                // Enable Break Mode
                m_MotorConfigs.NeutralMode.Coast;
                m_pRobotIO->m_Motor.GetConfigurator().Apply(m_MotorConfigs);

                m_eCommand = motor::COMMAND_NONE;
                return;
            }
        }

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