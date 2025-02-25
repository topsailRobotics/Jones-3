// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ExampleSubsystem extends SubsystemBase {

  //temp name 
  //private final SparkMax m_LeftElevatorMotor = new SparkMax(ElevatorConstants.kLeftMotorCANID, MotorType.kBrushless);

  /** Creates a new ExampleSubsystem. */
  public void ExampleSubsystem() {
    // SparkMax: white motor, SparkFlex: black cylinder motot
 //   m_LeftElevatorMotor.setVoltage(3);
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
// sketchy quick fix
  public void setVoltage(int i) {
    // TODO Auto-generated method stub
//    throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
  }
}
