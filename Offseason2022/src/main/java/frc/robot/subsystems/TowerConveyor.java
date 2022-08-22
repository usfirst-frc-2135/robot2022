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
import frc.robot.Constants.TowerConveyor.Mode;
import frc.robot.frc2135.PhoenixUtil;
import frc.robot.frc2135.RobotConfig;

/**
 *
 */
public class TowerConveyor extends SubsystemBase
{
  private boolean                         m_talonValidVC9;
  private WPI_TalonFX                     m_motorVC9            = new WPI_TalonFX(9);
  private double                          m_acquireSpeed;
  private double                          m_acquireSpeedSlow;
  private double                          m_expelSpeed;
  private double                          m_expelSpeedFast;
  private SupplyCurrentLimitConfiguration m_supplyCurrentLimits = new SupplyCurrentLimitConfiguration(true, 45.0, 45.0,
      0.001);
  private StatorCurrentLimitConfiguration m_statorCurrentLimits = new StatorCurrentLimitConfiguration(true, 80.0, 80.0,
      0.001);
  private final int                       kCANTimeout           = 30;
  private int                             resetCountVC9;
  private DigitalInput                    m_cargoDetected       = new DigitalInput(2);

  /**
   *
   */
  public TowerConveyor( )
  {
    // cargoDetect = new DigitalInput(0);
    // addChild("CargoDetect", cargoDetect);

    setName("Tower Conveyor");
    setSubsystem("Tower Conveyor");

    m_talonValidVC9 = PhoenixUtil.getInstance( ).talonFXInitialize(m_motorVC9, "VC9");
    SmartDashboard.putBoolean("HL_VC9Valid", m_talonValidVC9);

    RobotConfig config = RobotConfig.getInstance( );
    m_acquireSpeed = config.getValueAsDouble("VC_AcquireSpeed", 1.0);
    m_acquireSpeedSlow = config.getValueAsDouble("VC_AcquireSpeedSlow", 0.2);
    m_expelSpeed = config.getValueAsDouble("VC_expelSpeed", -0.2);
    m_expelSpeedFast = config.getValueAsDouble("VC_expelSpeedFast", -1.0);

    if (m_talonValidVC9)
    {
      m_motorVC9.setInverted(false);
      m_motorVC9.setNeutralMode(NeutralMode.Coast);
      m_motorVC9.set(ControlMode.PercentOutput, 0.0);

      m_motorVC9.configSupplyCurrentLimit(m_supplyCurrentLimits);
      m_motorVC9.configStatorCurrentLimit(m_statorCurrentLimits);

      m_motorVC9.setStatusFramePeriod(StatusFrame.Status_1_General, 255, kCANTimeout);
      m_motorVC9.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 255, kCANTimeout);
    }

    initialize( );
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
    if (m_motorVC9.hasResetOccurred( ))
    {
      resetCountVC9 += 1;
      SmartDashboard.putNumber("HL_Resets_VC9", resetCountVC9);
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
    setVerticalConveyorSpeed(Mode.VCONVEYOR_STOP);
  }

  public void faultDump( )
  {
    PhoenixUtil.getInstance( ).talonFXFaultDump(m_motorVC9, "VC 9");
  }

  // Set mode of conveyor
  public void setVerticalConveyorSpeed(Mode mode)
  {
    final String strName;
    double outputVC = 0.0; // Default: off

    switch (mode)
    {
      default :
      case VCONVEYOR_STOP :
        strName = "STOP";
        outputVC = 0.0;
        break;
      case VCONVEYOR_ACQUIRE :
        strName = "ACQUIRE";
        outputVC = m_acquireSpeed;
        break;
      case VCONVEYOR_ACQUIRE_SLOW :
        strName = "ACQUIRE_SLOW";
        outputVC = m_acquireSpeedSlow;
        break;
      case VCONVEYOR_EXPEL :
        strName = "EXPEL";
        outputVC = m_expelSpeed;
        break;
      case VCONVEYOR_EXPEL_FAST :
        strName = "EXPEL_FAST";
        outputVC = m_expelSpeedFast;
        break;
    }

    DataLogManager.log(getSubsystem( ) + ": VC Set Speed - " + strName);

    if (m_talonValidVC9)
      m_motorVC9.set(ControlMode.PercentOutput, outputVC);
  }

  boolean isCargoDetected( )
  {
    boolean cargoDetected = m_cargoDetected.get( );
    SmartDashboard.putBoolean("VC_cargoDetected", cargoDetected);
    return cargoDetected;
  }
}
