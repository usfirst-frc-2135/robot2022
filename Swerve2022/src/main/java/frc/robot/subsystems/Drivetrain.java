
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;

/**
 *
 */
public class Drivetrain extends SubsystemBase
{
    private WPI_TalonFX driveLF1;
    private WPI_TalonFX steerLF2;
    private WPI_TalonFX driveRF3;
    private WPI_TalonFX steerRF4;
    private WPI_TalonFX driveLR5;
    private WPI_TalonFX steerLR6;
    private WPI_TalonFX driveRR7;
    private WPI_TalonFX steerRR8;
    private Pigeon2 pigeonIMU;

    /**
    *
    */
    public Drivetrain()
    {
        driveLF1 = new WPI_TalonFX(1);

        steerLF2 = new WPI_TalonFX(2);

        driveRF3 = new WPI_TalonFX(3);

        steerRF4 = new WPI_TalonFX(4);

        driveLR5 = new WPI_TalonFX(5);

        steerLR6 = new WPI_TalonFX(6);

        driveRR7 = new WPI_TalonFX(7);

        steerRR8 = new WPI_TalonFX(8);

        pigeonIMU = new Pigeon2(0);
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
