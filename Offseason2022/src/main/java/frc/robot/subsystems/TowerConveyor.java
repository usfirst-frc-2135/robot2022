// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TCConsts;
import frc.robot.Constants.TCConsts.TCMode;
import frc.robot.frc2135.PhoenixUtil;

/**
 *
 */
public class TowerConveyor extends SubsystemBase
{
  // Constants
  private static final int                CANTIMEOUT            = 30;  // CAN timeout in msec
  private static final int                PIDINDEX              = 0;   // PID in use (0-primary, 1-aux)
  private static final int                SLOTINDEX             = 0;   // Use first PID slot

  // Devices and simulation objects
  private WPI_TalonFX                     m_motorVC9            = new WPI_TalonFX(TCConsts.kCANID);
  private DigitalInput                    m_cargoDetect         = new DigitalInput(TCConsts.kCargoDIO);

  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true, 45.0, 45.0, 0.001);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true, 80.0, 80.0, 0.001);

  // Declare module variables
  private boolean                         m_validVC9;
  private int                             m_resetCountVC9;
  private double                          m_acquireSpeed;
  private double                          m_acquireSpeedSlow;
  private double                          m_expelSpeed;
  private double                          m_expelSpeedFast;

  /**
   *
   */
  public TowerConveyor( )
  {
    setName("TowerConveyor");
    setSubsystem("TowerConveyor");
    addChild("CargoDetect", m_cargoDetect);

    m_validVC9 = PhoenixUtil.getInstance( ).talonFXInitialize(m_motorVC9, "VC9");
    SmartDashboard.putBoolean("HL_validVC9", m_validVC9);

    m_acquireSpeed = TCConsts.kTCAcquireSpeed;
    m_acquireSpeedSlow = TCConsts.kAcquireSpeedSlow;
    m_expelSpeed = TCConsts.kTCExpelSpeed;
    m_expelSpeedFast = TCConsts.kTCExpelSpeedFast;

    if (m_validVC9)
    {
      m_motorVC9.setInverted(false);
      m_motorVC9.setNeutralMode(NeutralMode.Coast);
      m_motorVC9.set(ControlMode.PercentOutput, 0.0);

      m_motorVC9.configSupplyCurrentLimit(m_supplyCurrentLimits);
      m_motorVC9.configStatorCurrentLimit(m_statorCurrentLimits);

      m_motorVC9.setStatusFramePeriod(StatusFrame.Status_1_General, 255, CANTIMEOUT);
      m_motorVC9.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255, CANTIMEOUT);
    }

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    if (m_motorVC9.hasResetOccurred( ))
    {
      m_resetCountVC9 += 1;
      SmartDashboard.putNumber("HL_resetCountVC9", m_resetCountVC9);
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
    DataLogManager.log("VC Init");
    setTowerConveyorSpeed(TCMode.TCONVEYOR_STOP);
  }

  public void faultDump( )
  {
    PhoenixUtil.getInstance( ).talonFXFaultDump(m_motorVC9, "VC 9");
  }

  // Set mode of conveyor
  public void setTowerConveyorSpeed(TCMode mode)
  {
    final String strName;
    double outputVC = 0.0; // Default: off

    switch (mode)
    {
      default :
      case TCONVEYOR_STOP :
        strName = "STOP";
        outputVC = 0.0;
        break;
      case TCONVEYOR_ACQUIRE :
        strName = "ACQUIRE";
        outputVC = m_acquireSpeed;
        break;
      case TCONVEYOR_ACQUIRE_SLOW :
        strName = "ACQUIRE_SLOW";
        outputVC = m_acquireSpeedSlow;
        break;
      case TCONVEYOR_EXPEL :
        strName = "EXPEL";
        outputVC = m_expelSpeed;
        break;
      case TCONVEYOR_EXPEL_FAST :
        strName = "EXPEL_FAST";
        outputVC = m_expelSpeedFast;
        break;
    }

    DataLogManager.log(getSubsystem( ) + ": VC Set Speed - " + strName);

    if (m_validVC9)
      m_motorVC9.set(ControlMode.PercentOutput, outputVC);
  }

  public boolean isCargoDetected( )
  {
    boolean cargoDetected = m_cargoDetect.get( );
    SmartDashboard.putBoolean("VC_cargoDetected", cargoDetected);
    return cargoDetected;
  }
}
