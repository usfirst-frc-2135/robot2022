package frc.robot.frc2135;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

public class PhoenixUtil
{

  private static PhoenixUtil instance = null;

  PhoenixUtil( )
  {}

  public static PhoenixUtil getInstance( )
  {
    if (instance == null)
      instance = new PhoenixUtil( );

    return instance;
  }

  public boolean talonFXInitialize(WPI_TalonFX talon, String motorName)
  {
    return true;
  }

  public void talonFXFaultDump(WPI_TalonFX talon, String motorName)
  {

  }

  
  public boolean pigeonIMUInitialize(PigeonIMU pigeon)
  {
    return true;
  }

  public void pigeonIMUFaultDump(PigeonIMU pigeon)
  {

  }

  public void checkError(ErrorCode errorCode, String message)
  {

  }
}
