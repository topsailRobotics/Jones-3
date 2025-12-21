package frc.robot.subsystems;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KickerConstants;
public class KickerSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  // Initilization
private final SparkMax m_KickerArm = new SparkMax(KickerConstants.kKickerArmCANID, MotorType.kBrushless);
private final SparkMax m_KickerWheel = new SparkMax(KickerConstants.kKickerWheelCANID, MotorType.kBrushless);
private final SparkClosedLoopController m_pidController = m_KickerArm.getClosedLoopController();
private SparkAnalogSensor m_ArmEncoder;
  public KickerSubsystem() {

  
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
    m_ArmEncoder = m_KickerArm.getAnalog();
    SmartDashboard.putNumber("Kicker Encoder Position", m_ArmEncoder.getPosition());
    SmartDashboard.putNumber("Kicker Set Point Value",KickerConstants.kKickerSetPointVal);
  }

  

  public void runKicker(double setposition) {
    m_KickerWheel.setVoltage(4);
    m_pidController.setReference(setposition, com.revrobotics.spark.SparkBase.ControlType.kPosition);

  }

  public void stopKicker(double setposition) {
    m_KickerWheel.setVoltage(0);
    //m_pidController.setReference(0.1, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    m_pidController.setReference(setposition, com.revrobotics.spark.SparkBase.ControlType.kPosition);

  }
 
 /*  public void KickerSetPoint(double setposition) {
    Set the setpoint of the PID controller in raw position mode
 }*/

}