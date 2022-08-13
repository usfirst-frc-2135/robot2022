
// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LED.LEDColor;

/**
 *
 */

public class LED extends SubsystemBase
{
  // variables
  public LEDColor           previousColor = LEDColor.LEDCOLOR_OFF;
  private CANdle            candle        = new CANdle(0);
  SendableChooser<LEDColor> ledChooser    = new SendableChooser<LEDColor>( );

  /**
   *
   */
  public LED( )
  {
    setName("LED");
    setSubsystem("LED");

    setColor(LEDColor.LEDCOLOR_BLUE);
    candle.configBrightnessScalar(0.7);

    // Add options for colors in SmartDashboard
    ledChooser.setDefaultOption("LED_Off", LEDColor.LEDCOLOR_OFF);
    ledChooser.addOption("LED_White", LEDColor.LEDCOLOR_WHITE);
    ledChooser.addOption("LED_Red", LEDColor.LEDCOLOR_RED);
    ledChooser.addOption("LED_Orange", LEDColor.LEDCOLOR_ORANGE);
    ledChooser.addOption("LED_Yellow", LEDColor.LEDCOLOR_YELLOW);
    ledChooser.addOption("LED_Green", LEDColor.LEDCOLOR_GREEN);
    ledChooser.addOption("LED_Blue", LEDColor.LEDCOLOR_BLUE);
    ledChooser.addOption("LED_Purple", LEDColor.LEDCOLOR_PURPLE);

    SmartDashboard.putData("LED_Color", ledChooser);
    SmartDashboard.putBoolean("LED_colorMode", false);
  }

  @Override
  public void periodic( )
  {
    // This method will be called once per scheduler run
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
    DataLogManager.log(getSubsystem( ) + ": subsystem initialized!");
    setColor(LEDColor.LEDCOLOR_OFF);
  }

  public void setColor(LEDColor color)
  {
    final String strName;

    if (previousColor != color)
    {
      if (color == (LEDColor.LEDCOLOR_DASH))
        color = ledChooser.getSelected( );

      switch (color)
      {
        default :
        case LEDCOLOR_OFF :
          strName = "OFF";
          candle.setLEDs(0, 0, 0); // black
          break;
        case LEDCOLOR_WHITE :
          strName = "WHITE";
          candle.setLEDs(255, 255, 255); // white
          break;
        case LEDCOLOR_RED :
          strName = "RED";
          candle.setLEDs(255, 0, 0); // red
          break;
        case LEDCOLOR_ORANGE :
          strName = "ORANGE";
          candle.setLEDs(255, 80, 0); // orange
          break;
        case LEDCOLOR_YELLOW :
          strName = "YELLOW";
          candle.setLEDs(255, 255, 0); // yellow
          break;
        case LEDCOLOR_GREEN :
          strName = "GREEN";
          candle.setLEDs(0, 255, 0); // green
          break;
        case LEDCOLOR_BLUE :
          strName = "BLUE";
          candle.setLEDs(0, 0, 255); // blue
          break;
        case LEDCOLOR_PURPLE :
          strName = "PURPLE";
          candle.setLEDs(255, 0, 255); // purple
          break;
      }

      DataLogManager.log(getSubsystem( ) + ": color is now " + strName);
      previousColor = color;
    }
  }

  public void setShooterColor(LEDColor color)
  {
    Boolean colorMode = SmartDashboard.getBoolean("LED_colorMode", false);

    if (colorMode)
      setColor(color);
  }

  public void setLLColor(LEDColor color)
  {
    Boolean colorMode = SmartDashboard.getBoolean("LED_colorMode", false);

    if (!colorMode)
      setColor(color);
  }

}
