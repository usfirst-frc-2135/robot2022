#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/util/Color8Bit.h>
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <functional>
#include <memory>
#include <units/angle.h>
#include <units/length.h>

class SimulateLimelight : public frc2::CommandHelper<frc2::CommandBase, SimulateLimelight>
{
public:
    SimulateLimelight(
        std::function<frc::Pose2d()> robotPoseProvider,
        frc::Translation2d goalTranslation,
        units::meter_t goalHeight,
        frc::Translation2d cameraTranslationOffset,
        frc::Rotation2d cameraYawOffset,
        units::meter_t cameraHeight,
        units::radian_t cameraPitch);
    void Execute() override;
    bool RunsWhenDisabled() const override;

private:
    std::function<frc::Pose2d()> m_robotPoseProvider;
    frc::Translation2d m_goalTranslation;
    units::meter_t m_goalHeight;
    frc::Translation2d m_cameraTranslationOffset;
    frc::Rotation2d m_cameraYawOffset;
    units::meter_t m_cameraHeight;
    units::radian_t m_cameraPitch;

    std::unique_ptr<frc::Mechanism2d> m_mech = std::make_unique<frc::Mechanism2d>(2, 2, frc::Color8Bit{ 0, 0, 0 });
    std::unique_ptr<frc::MechanismRoot2d> m_mechRoot =
        std::unique_ptr<frc::MechanismRoot2d>(m_mech->GetRoot("Limelight", 0, 0));
    std::unique_ptr<frc::MechanismLigament2d> m_mechLigament = std::unique_ptr<frc::MechanismLigament2d>(
        m_mechRoot->Append<frc::MechanismLigament2d>("Target", 0.1, 0_deg, 24, frc::Color8Bit{ 0, 255, 0 }));
};