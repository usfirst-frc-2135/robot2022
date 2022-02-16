#include "commands/SimulateLimelight.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableInstance.h>
#include <units/math.h>

constexpr auto LIMELIGHT2_HORIZONTAL_HALF_FOV = 29.8_deg;
constexpr auto LIMELIGHT2_VERTICAL_HALF_FOV = 24.85_deg;

SimulateLimelight::SimulateLimelight(
    std::function<frc::Pose2d()> robotPoseProvider,
    frc::Translation2d goalTranslation,
    units::meter_t goalHeight,
    frc::Translation2d cameraTranslationOffset,
    frc::Rotation2d cameraYawOffset,
    units::meter_t cameraHeight,
    units::radian_t cameraPitch) :
    m_robotPoseProvider(robotPoseProvider),
    m_goalTranslation(goalTranslation),
    m_goalHeight(goalHeight),
    m_cameraTranslationOffset(cameraTranslationOffset),
    m_cameraYawOffset(cameraYawOffset),
    m_cameraHeight(cameraHeight),
    m_cameraPitch(cameraPitch)
{
    frc::SmartDashboard::PutData("Limelight (Sim)", m_mech.get());
}

void SimulateLimelight::Execute()
{
    frc::Pose2d robotPose = m_robotPoseProvider();
    // The camera may not be centered on the robot, so we need to account for offets
    frc::Pose2d cameraPose{ robotPose.Translation() + m_cameraTranslationOffset.RotateBy(robotPose.Rotation()),
                            robotPose.Rotation() + m_cameraYawOffset };

    // Get the goal translation and height as if the camera were at [0, 0, 0]
    frc::Translation2d cameraRelativeGoalTranslation =
        (m_goalTranslation - cameraPose.Translation()).RotateBy(-cameraPose.Rotation());
    units::meter_t cameraRelativeGoalHeight = m_goalHeight - m_cameraHeight;

    units::degree_t tx = -units::math::atan2(cameraRelativeGoalTranslation.Y(), cameraRelativeGoalTranslation.X());
    units::degree_t ty =
        units::math::atan2(cameraRelativeGoalHeight, cameraRelativeGoalTranslation.Norm()) - m_cameraPitch;

    auto table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    table->PutNumber("tx", tx.value());
    table->PutNumber("ty", ty.value());
    table->PutBoolean(
        "tv",
        (tx >= -LIMELIGHT2_HORIZONTAL_HALF_FOV && tx <= LIMELIGHT2_HORIZONTAL_HALF_FOV)
            && (ty >= -LIMELIGHT2_VERTICAL_HALF_FOV && ty <= LIMELIGHT2_VERTICAL_HALF_FOV));
    m_mechRoot->SetPosition(tx / LIMELIGHT2_HORIZONTAL_HALF_FOV + 0.95, ty / LIMELIGHT2_VERTICAL_HALF_FOV + 1);
}

bool SimulateLimelight::RunsWhenDisabled() const
{
    return true;
}