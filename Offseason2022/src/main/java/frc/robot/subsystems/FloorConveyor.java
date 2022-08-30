
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FCConsts;
import frc.robot.Constants.FCConsts.FCMode;
import frc.robot.frc2135.PhoenixUtil;
import frc.robot.frc2135.RobotConfig;

/**
 *
 */
public class FloorConveyor extends SubsystemBase
{
  // Constants
  private static final int                CANTIMEOUT            = 30;  // CAN timeout in msec
  private static final int                PIDINDEX              = 0;   // PID in use (0-primary, 1-aux)
  private static final int                SLOTINDEX             = 0;   // Use first PID slot

  // Devices and simulation objects
  private WPI_TalonFX                     m_motorFC8            = new WPI_TalonFX(FCConsts.kCANID);

  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true, 45.0, 45.0, 0.001);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 0.001);

  // Declare module variables
  private boolean                         m_validFC8;  // Health indicator for floor conveyor talon
  private int                             m_resetCountFC8;  // reset counter for motor
  private double                          m_acquireSpeed;
  private double                          m_expelSpeed;
  private double                          m_expelSpeedFast;

  /**
   *
   */
  public FloorConveyor( )
  {
    // Set the names for this subsystem for later use
    setName("FloorConveyor");
    setSubsystem("FloorConveyor");

    // Validate Talon FX controllers, initialize and display firmware versions
    m_validFC8 = PhoenixUtil.getInstance( ).talonFXInitialize(m_motorFC8, "FC8");
    SmartDashboard.putBoolean("HL_validFC8", m_validFC8);

    // Initialize Variables

    m_acquireSpeed = FCConsts.kFCAcquireSpeed;
    m_expelSpeed = FCConsts.kFCExpelSpeed;
    m_expelSpeedFast = FCConsts.kFCExpelSpeedFast;

    if (m_validFC8)
    {
      m_motorFC8.setInverted(false);
      m_motorFC8.setNeutralMode(NeutralMode.Coast);
      m_motorFC8.set(ControlMode.PercentOutput, 0.0);

      m_motorFC8.configSupplyCurrentLimit(m_supplyCurrentLimits);
      m_motorFC8.configStatorCurrentLimit(m_statorCurrentLimits);

      m_motorFC8.setStatusFramePeriod(StatusFrame.Status_1_General, 255, CANTIMEOUT);
      m_motorFC8.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255, CANTIMEOUT);
    }

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    if (m_motorFC8.hasResetOccurred( ))
    {
      m_resetCountFC8 += 1;
      SmartDashboard.putNumber("HL_resetCountFC8", m_resetCountFC8);
    }
  }

  @Override
  public void simulationPeriodic( )
  {
    // This method will be called once per scheduler run when in simulation
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public void initialize( )
  {
    DataLogManager.log("FC Init");
    setFloorConveyorSpeed(FCConsts.FCMode.FCONVEYOR_STOP);
  }

  // Dump all Talon faults
  void faultDump( )
  {
    PhoenixUtil.getInstance( ).talonFXFaultDump(m_motorFC8, "FC 8");
  }

  public void setFloorConveyorSpeed(FCMode mode)
  {
    final String strName;
    double outputFC = 0.0; // Default: off

    switch (mode)
    {
      default :
      case FCONVEYOR_STOP :
        strName = "STOP";
        outputFC = 0.0;
        break;
      case FCONVEYOR_ACQUIRE :
        strName = "ACQUIRE";
        outputFC = m_acquireSpeed;
        break;
      case FCONVEYOR_EXPEL :
        strName = "EXPEL";
        outputFC = m_expelSpeed;
        break;
      case FCONVEYOR_EXPEL_FAST :
        strName = "EXPEL_FAST";
        outputFC = m_expelSpeedFast;
        break;
    }

    DataLogManager.log(getSubsystem( ) + ": FC Set Speed - " + strName);

    if (m_validFC8)
      m_motorFC8.set(ControlMode.PercentOutput, outputFC);
  }

}
