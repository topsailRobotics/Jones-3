package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

// Initilization
private final SparkMax m_LeftElevatorMotor = new SparkMax(ElevatorConstants.kLeftMotorCANID, MotorType.kBrushless);
private final SparkMax m_rightElevatorMotor = new SparkMax(ElevatorConstants.kRightMotorCANID, MotorType.kBrushless);


  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
   // m_rightElevatorMotor.setInverted(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

// our current method

public void ElevatorUp() {
  m_LeftElevatorMotor.setVoltage(2);
  m_rightElevatorMotor.setVoltage(-2);
}

public void ElevatorDown() {
  m_LeftElevatorMotor.setVoltage(-2);
  m_rightElevatorMotor.setVoltage(2);
}

public void ElevatorOff() {
  m_LeftElevatorMotor.setVoltage(0);
  m_rightElevatorMotor.setVoltage(0);
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
}