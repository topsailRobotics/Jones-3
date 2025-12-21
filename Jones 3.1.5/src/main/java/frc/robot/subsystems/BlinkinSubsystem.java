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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  public void idleBlinkin() {
    Blinkin.set(BlinkinConstants.Blue);
  }

  public void L2Blinkin() {
    Blinkin.set(BlinkinConstants.Blue_Violet);
  }

  public void L3Blinkin() {
    Blinkin.set(BlinkinConstants.Violet);
  }

  public void willyBlinkin() {
    Blinkin.set(BlinkinConstants.Hot_Pink);
  }
 
}