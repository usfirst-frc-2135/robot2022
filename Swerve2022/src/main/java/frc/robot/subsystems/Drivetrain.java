// RobotBuilder Version: 4.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS

/**
 *
 */
public class Drivetrain extends SubsystemBase
{
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private WPI_TalonFX driveLF1;
    private WPI_TalonFX steerLF2;
    private WPI_TalonFX driveRF3;
    private WPI_TalonFX steerRF4;
    private WPI_TalonFX driveLR5;
    private WPI_TalonFX steerLR6;
    private WPI_TalonFX driveRR7;
    private WPI_TalonFX steerRR8;
    private Pigeon2 pigeonIMU;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
    *
    */
    public Drivetrain()
    {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveLF1 = new WPI_TalonFX(1);

        steerLF2 = new WPI_TalonFX(2);

        driveRF3 = new WPI_TalonFX(3);

        steerRF4 = new WPI_TalonFX(4);

        driveLR5 = new WPI_TalonFX(5);

        steerLR6 = new WPI_TalonFX(6);

        driveRR7 = new WPI_TalonFX(7);

        steerRR8 = new WPI_TalonFX(8);

        pigeonIMU = new Pigeon2(0);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    @Override public void periodic()
    {
        // This method will be called once per scheduler run
    }

    @Override public void simulationPeriodic()
    {
        // This method will be called once per scheduler run when in simulation
    }

    // Put methods for controlling this subsystem
    // here. Call these from Commands.
}
