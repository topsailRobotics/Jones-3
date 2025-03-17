//goon rizz gyatt sussy baka ohio mobile <3
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BlinkinConstants;
public class BlinkinSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Initilization
private final Spark Blinkin = new Spark(0);

public BlinkinSubsystem() {

  
  }
  /** 
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }



  public void idleBlinkin() {
    Blinkin.set(BlinkinConstants.Gold);
  }

  public void L2Blinkin() {
    Blinkin.set(BlinkinConstants.Black);
  }
 
}