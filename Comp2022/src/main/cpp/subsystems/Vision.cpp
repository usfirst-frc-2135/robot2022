// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "RobotContainer.h"
#include "frc2135/RobotConfig.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <spdlog/spdlog.h>

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES
#include "subsystems/Vision.h"

#include <frc/smartdashboard/SmartDashboard.h>

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=INCLUDES

Vision::Vision()
{
    SetName("Vision");
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    SetSubsystem("Vision");

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    m_table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    SetLEDMode(LED_ON);
    SetCameraDisplay(PIP_SECONDARY);

    frc::SmartDashboard::SetDefaultBoolean("VI_SM_OVERRIDE_ENABLED", false);

    frc2135::RobotConfig *config = frc2135::RobotConfig::GetInstance();
    config->GetValueAsDouble("VI_Distance1", m_distance1, 48);
    config->GetValueAsDouble("VI_Distance2", m_distance2, 60);
    config->GetValueAsDouble("VI_VertOffset1", m_vertOffset1, -0.58);
    config->GetValueAsDouble("VI_VertOffset2", m_vertOffset2, -4.8);
    frc::SmartDashboard::PutNumber("VI_Distance1", m_distance1);
    frc::SmartDashboard::PutNumber("VI_Distance2", m_distance2);
    frc::SmartDashboard::PutNumber("VI_VertOffset1", m_vertOffset1);
    frc::SmartDashboard::PutNumber("VI_VertOffset2", m_vertOffset2);

    Initialize();
}

double Vision::CalculateDist()
{
    m_slope = (m_distance2 - m_distance1) / (m_vertOffset2 - m_vertOffset1);
    m_distOffset = m_distance1 - m_slope * m_vertOffset1;
    m_distLight = (m_slope * m_targetVertAngle) + m_distOffset;

    return m_distLight;
}

void Vision::Periodic()
{
    // Put code here to be run every loop

    bool smOverrideEnabled = frc::SmartDashboard::GetBoolean("VI_SM_OVERRIDE_ENABLED", false);
    if (smOverrideEnabled)
    {
        // During daytime hours we can use smartdashboard to bipass the limelight.
        // This will allow us to calibrate the shooter distance without relying
        // on lighting conditions or limelight tuning.
        m_targetHorizAngle = frc::SmartDashboard::GetNumber("VI_SM_OVERRIDE_H_ANGLE", 0.0);
        m_targetVertAngle = frc::SmartDashboard::GetNumber("VI_SM_OVERRIDE_V_ANGLE", 0.0);
        m_targetArea = frc::SmartDashboard::GetNumber("VI_SM_OVERRIDE_TARGET_AREA", 0.0);
        m_targetSkew = frc::SmartDashboard::GetNumber("VI_SM_OVERRIDE_TARGET_SKEW", 0.0);
        m_targetValid = frc::SmartDashboard::GetBoolean("VI_SM_OVERRIDE_TARGET_VALID", true);
    }
    else
    {
        m_targetHorizAngle = m_table->GetNumber("tx", 0.0);
        m_targetVertAngle = m_yfilter.Calculate(m_table->GetNumber("ty", 0.0));
        m_targetArea = m_table->GetNumber("ta", 0.0);
        m_targetSkew = m_table->GetNumber("ts", 0.0);
        //m_targetValid = (bool)m_table->GetNumber("tv", 0.0);
        m_targetValid = m_vfilter.Calculate((bool)m_table->GetNumber("tv", 0.0));
    }

    CalculateDist();

    frc::SmartDashboard::PutNumber("VI_HORIZ_ANGLE", m_targetHorizAngle);
    frc::SmartDashboard::PutNumber("VI_VERT_ANGLE", m_targetVertAngle);
    frc::SmartDashboard::PutNumber("VI_TARGET_AREA", m_targetArea);
    frc::SmartDashboard::PutNumber("VI_TARGET_SKEW", m_targetSkew);
    frc::SmartDashboard::PutNumber("VI_TARGET_VALID", m_targetValid);
    frc::SmartDashboard::PutNumber("VI_Slope", m_slope);
    frc::SmartDashboard::PutNumber("VI_DistanceLimeLight", m_distLight);
}

void Vision::SyncStateFromDashboard(void)
{
    m_distance1 = frc::SmartDashboard::GetNumber("VI_Distance1", m_distance1);
    m_distance2 = frc::SmartDashboard::GetNumber("VI_Distance2", m_distance2);
    m_vertOffset1 = frc::SmartDashboard::GetNumber("VI_VertOffset1", m_vertOffset1);
    m_vertOffset2 = frc::SmartDashboard::GetNumber("VI_VertOffset2", m_vertOffset2);
}

void Vision::SimulationPeriodic()
{
    // This method will be called once per scheduler run when in simulation
}

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

// Put methods for controlling this subsystem
// here. Call these from Commands.

void Vision::Initialize(void)
{
    spdlog::info("VI Init");
    m_table->PutNumber("ledMode", LED_OFF);
    SyncStateFromDashboard();
}

double Vision::GetHorizOffsetDeg()
{
    return m_targetHorizAngle;
}

double Vision::GetVertOffsetDeg()
{
    return m_targetVertAngle;
}

double Vision::GetTargetArea()
{
    return m_targetArea;
}

double Vision::GetTargetSkew()
{
    return m_targetSkew;
}

bool Vision::GetTargetValid()
{
    return m_targetValid;
}

void Vision::SetLEDMode(ledMode_e mode)
{
    m_table->PutNumber("ledMode", mode);
    spdlog::info("VI SetLedMode : {}", mode);
}

Vision::ledMode_e Vision::GetLEDMode(void)
{
    ledMode_e mode = LED_CUR_MODE;
    mode = (ledMode_e)m_table->GetNumber("ledMode", mode);

    spdlog::info("VI GetLedMode : {}", mode);
    return mode;
}

void Vision::SetCameraDisplay(camStream_e stream)
{
    m_table->PutNumber("stream", stream);
    spdlog::info("VI SetCameraDisplay : {}", stream);
}
