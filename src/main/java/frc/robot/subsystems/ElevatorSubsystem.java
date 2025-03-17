package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
//import com.revrobotics.spark.config.SparkMaxConfig;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
public class ElevatorSubsystem extends SubsystemBase {

// Initilization
private final SparkMax m_LeftElevatorMotor = new SparkMax(ElevatorConstants.kELeftMotorCANID, MotorType.kBrushless);
private final SparkMax m_RightElevatorMotor = new SparkMax(ElevatorConstants.kERightMotorCANID, MotorType.kBrushless);
private AbsoluteEncoder m_ElevatorEncoder;
// initialize the PID controller
private final SparkClosedLoopController m_RightpidController = m_RightElevatorMotor.getClosedLoopController();
//m_LeftElevatorMotor.apply;
//int smartMotionSlot = 0;
//m_RightpidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
//m_RightpidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
//m_RightpidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
//m_RightpidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);
//private static final double LIMIT_BOTTOM = 0.646978;
//private static final double LIMIT_TOP = 2.38377;
  /** Creates a new ExampleSubsystem. */
  public ElevatorSubsystem() {
   // m_rightElevatorMotor.setInverted(true);
   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_ElevatorEncoder = m_RightElevatorMotor.getAbsoluteEncoder();
    SmartDashboard.putNumber("Elevator Encoder Position", m_ElevatorEncoder.getPosition());
    SmartDashboard.putNumber("Elevator Encoder Velocity", m_ElevatorEncoder.getVelocity());
    SmartDashboard.putNumber("Elevator Set Point Value", ElevatorConstants.ElevatorSetPointVal);
    
  }
/* 
Tests

public Command ElevatorUpTest() {
  return run(
        () -> {
          m_LeftElevatorMotor.setVoltage(2);
          m_rightElevatorMotor.setVoltage(-2);
        });
  
}

public Command ElevatorDownTest() {
  return run(
        () -> {
          
          m_LeftElevatorMotor.setVoltage(-2);
  m_rightElevatorMotor.setVoltage(2);
        });
  
}
*/
//What we actually use currently
public void ElevatorUp() {
  m_LeftElevatorMotor.setVoltage(2);
  m_RightElevatorMotor.setVoltage(-2);
}

public void ElevatorDown() {
  m_LeftElevatorMotor.setVoltage(-2);
  m_RightElevatorMotor.setVoltage(2);
}

public void ElevatorSetPoint(double setposition) {
  // Set the setpoint of the PID controller in raw position mode
m_RightpidController.setReference(setposition, com.revrobotics.spark.SparkBase.ControlType.kPosition);
}

public void ElevatorOff() {
  m_LeftElevatorMotor.setVoltage(0);
  m_RightElevatorMotor.setVoltage(0);
}          

  
}