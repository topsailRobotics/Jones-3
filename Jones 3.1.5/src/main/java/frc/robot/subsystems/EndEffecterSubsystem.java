package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EndEffecterConstants;
public class EndEffecterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Initilization
private final SparkMax m_LeftEndEffecterMotor = new SparkMax(EndEffecterConstants.kEELeftMotorCANID, MotorType.kBrushless);
private final SparkMax m_RightEndEffecterMotor = new SparkMax(EndEffecterConstants.kEERightMotorCANID, MotorType.kBrushless);
//Removed "private" part so code would work
  public EndEffecterSubsystem() {
  
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



  public void runEE() {
    m_LeftEndEffecterMotor.setVoltage(3.3);
    m_RightEndEffecterMotor.setVoltage(-3.3);
  }

  public void runTrough() {
    m_LeftEndEffecterMotor.setVoltage(3.3);
  }

  public void reverseEE() {
    m_LeftEndEffecterMotor.setVoltage(-3.3);
    m_RightEndEffecterMotor.setVoltage(3.3);
  }

  public void autoEE() {
    m_LeftEndEffecterMotor.setVoltage(2);
    m_RightEndEffecterMotor.setVoltage(-2);
  }
  public void stopEE() {
    m_LeftEndEffecterMotor.setVoltage(0);
    m_RightEndEffecterMotor.setVoltage(0);
  }
}