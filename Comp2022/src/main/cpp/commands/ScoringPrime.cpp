// RobotBuilder Version: 3.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// C++ from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

#include "commands/ScoringPrime.h"

#include "RobotContainer.h"

#include <spdlog/fmt/ostr.h>
#include <spdlog/spdlog.h>

ScoringPrime::ScoringPrime(Shooter *shooter)
{
    // Use AddRequirements() here to declare subsystem dependencies
    // eg. AddRequirements(m_Subsystem);
    SetName("ScoringPrime");

    spdlog::info("ScoringPrime");

    // Add your commands here, e.g.
    // AddCommands(FooCommand(), BarCommand());
    AddCommands(

        ShooterAim(true),
        ShooterRun(Shooter::SHOOTERSPEED_PRIME, shooter));
}

bool ScoringPrime::RunsWhenDisabled() const
{
    return false;
}