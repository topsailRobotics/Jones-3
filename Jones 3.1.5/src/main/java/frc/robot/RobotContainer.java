// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.KickerOff;
import frc.robot.commands.KickerOn;
import frc.robot.commands.Left;
import frc.robot.commands.LeftAlign;
import frc.robot.commands.Right;
import frc.robot.commands.RightAlign;
import frc.robot.commands.Align;
import frc.robot.commands.ElevatorL1;
import frc.robot.commands.ElevatorL2;
import frc.robot.commands.ElevatorL3;
import frc.robot.commands.ElevatorTrough;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.MathUtil;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.BlinkinSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
//import frc.robot.commands.endEffecterCommand;
import frc.robot.subsystems.EndEffecterSubsystem;
import frc.robot.subsystems.KickerSubsystem;
// import frc.robot.subsystems.unused.AutoAlignSubsystem;
// import frc.robot.subsystems.unused.ClimberSubsystem;
import frc.robot.subsystems.unused.LimelightSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.commands.ElevatorUpDown;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ElevatorSubsystem m_elevator = new ElevatorSubsystem();
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final EndEffecterSubsystem m_endeffecter = new EndEffecterSubsystem();
  private final KickerSubsystem m_kicker = new KickerSubsystem();
  private final BlinkinSubsystem m_blinkin = new BlinkinSubsystem();
  //private final ClimberSubsystem m_climber = new ClimberSubsystem();
  //private final AutoAlignSubsystem m_aAlign = new AutoAlignSubsystem();
  private final LimelightSubsystem m_lime = new LimelightSubsystem();


  private final String m_defaultAuto = "Middle Trough";
  private final String m_TestAuto = "10FootTest";

  // A chooser for autonomous commands
  SendableChooser<String> m_chooser = new SendableChooser<>();

  


  //NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
  //NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
  //NamedCommands.registerCommand("Rise-L3", new RiseL3(m_elevator));
  private final XboxController m_driverController1 =
      new XboxController(OperatorConstants.kDriverControllerPort0);

  private final XboxController m_driverController2 =
      new XboxController(OperatorConstants.kDriverControllerPort1);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

// Add commands to the autonomous command chooser
m_chooser.setDefaultOption(m_defaultAuto, m_defaultAuto);
m_chooser.addOption(m_TestAuto, m_TestAuto);
m_chooser.addOption(m_TestAuto, m_TestAuto);
m_chooser.addOption(m_TestAuto, m_TestAuto);

// Put the chooser on the dashboard
SmartDashboard.putData(m_chooser);

    NamedCommands.registerCommand("ElevatorL1", new ElevatorL1(m_elevator));

    NamedCommands.registerCommand("ElevatorL1.5", new ElevatorTrough(m_elevator));

    NamedCommands.registerCommand("ElevatorL2", new ElevatorL2(m_elevator));

    NamedCommands.registerCommand("ElevatorL3", new ElevatorL3(m_elevator));

    NamedCommands.registerCommand("KickerOff", new KickerOff(m_kicker));

    NamedCommands.registerCommand("KickerOn", new KickerOn(m_kicker));

    NamedCommands.registerCommand("runEE", 
    new InstantCommand(m_endeffecter::autoEE, m_endeffecter)
    .andThen(Commands.waitSeconds(1))
    .andThen(new InstantCommand(m_endeffecter::stopEE, m_endeffecter)));
    
    NamedCommands.registerCommand("EEintake", 
    new InstantCommand(m_endeffecter::runEE, m_endeffecter)
    .andThen(Commands.waitSeconds(.25))
    .andThen(new InstantCommand(m_endeffecter::stopEE, m_endeffecter)));

    NamedCommands.registerCommand("LeftAa",new LeftAlign(m_robotDrive));
    NamedCommands.registerCommand("RightAa",new RightAlign(m_robotDrive));
    NamedCommands.registerCommand("AutoAlign",new Align(m_robotDrive));
    NamedCommands.registerCommand("Left",new Left(m_robotDrive));
    NamedCommands.registerCommand("Right",new Right(m_robotDrive));


    // Configure the trigger bindings
    configureBindings();
     m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController1.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController1.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController1.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

            m_elevator.setDefaultCommand(
                new RunCommand(
                    () -> m_elevator.ElevatorOff(),
                    m_elevator));
           
 /*            m_climber.setDefaultCommand(
                new RunCommand(
                    () -> m_climber.stopClimber(),
                    m_climber));
   */                 
            m_endeffecter.setDefaultCommand(
                new RunCommand(
                    () -> m_endeffecter.stopEE(),
                    m_endeffecter));

            m_kicker.setDefaultCommand(
                new RunCommand(
                    () -> m_kicker.stopKicker(.1),
                    m_kicker));

            m_blinkin.setDefaultCommand(
                new RunCommand(
                    () -> m_blinkin.idleBlinkin(),
                    m_blinkin));

//THIS IS WHAT NEEDS TO BE FIXED 


  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  
  private void configureBindings() {

/* climber code thats unused
    //new JoystickButton(m_driverController, XboxController.Button.kA.value)
    //    .whileTrue(new RunCommand(
    //        () -> m_climber.runClimber(),
    //        m_climber));

    //new JoystickButton(m_driverController, XboxController.Button.kX.value)
    //    .whileTrue(new RunCommand(
    //        () -> m_climber.reverseClimber(),
    //        m_climber));
 */   
//normal button code for standard actions

new JoystickButton(m_driverController1, XboxController.Button.kA.value)
        .onTrue(new RunCommand(
                () -> m_elevator.ElevatorSetPoint(0),
                m_elevator));

new JoystickButton(m_driverController1, XboxController.Button.kX.value)
        .onTrue(new RunCommand(
                () -> m_elevator.ElevatorSetPoint(-13),
                m_elevator));

new JoystickButton(m_driverController1, XboxController.Button.kX.value)
        .onTrue(new RunCommand(
                () -> m_blinkin.L2Blinkin(),
                m_blinkin));

new JoystickButton(m_driverController1, XboxController.Button.kY.value)
        .onTrue(new RunCommand(
                () -> m_elevator.ElevatorSetPoint(-28),
                m_elevator));

new JoystickButton(m_driverController1, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(
                () -> m_endeffecter.reverseEE(),
                m_endeffecter));

new JoystickButton(m_driverController1, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(
                () -> m_endeffecter.runEE(),
                m_endeffecter));


 
    //
    //
    // used to switch between field relativity and non field relativity, might not be needed after april tags implemented
    new JoystickButton(m_driverController1, XboxController.Button.kB.value)
    .toggleOnTrue(new RunCommand(
        () -> m_robotDrive.drive(
            -MathUtil.applyDeadband(m_driverController1.getLeftY(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController1.getLeftX(), OIConstants.kDriveDeadband),
            -MathUtil.applyDeadband(m_driverController1.getRightX(), OIConstants.kDriveDeadband),
            false),
        m_robotDrive));
    //
    //
    // Custom trigger buttons TITANium Tigers team helped us with, used for Willy and gyro reset
    Trigger leftTrigger = new Trigger(() -> (m_driverController1.getLeftTriggerAxis() > 0.1));

    leftTrigger.whileTrue(new RunCommand(
            () -> m_kicker.runKicker(1),
            m_kicker));

    Trigger rightTrigger = new Trigger(() -> (m_driverController1.getRightTriggerAxis() > 0.5));

    rightTrigger.whileTrue(new RunCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
    //
    //
    // These are the binds that call the auto-align commands (gpt produced)
//     new JoystickButton(m_driverController2, XboxController.Button.kLeftBumper.value)
//             .whileTrue(new AutoAlignLeftCommand(
//                     m_robotDrive,
//                     m_aAlign));

//     new JoystickButton(m_driverController2, XboxController.Button.kRightBumper.value)
//             .whileTrue(new AutoAlignRightCommand(
//                     m_robotDrive,
//                     m_aAlign));
new JoystickButton(m_driverController2, XboxController.Button.kLeftBumper.value)
                .toggleOnTrue(new RunCommand(() -> m_robotDrive.AutoMove(m_robotDrive,-18.25),
                                m_lime));
                                
new JoystickButton(m_driverController2, XboxController.Button.kRightBumper.value)
                .toggleOnTrue(new RunCommand(() -> m_robotDrive.AutoMove(m_robotDrive,18.25),
                                m_lime));


//   controller 2 code  
    // new JoystickButton(m_driverController2, XboxController.Button.kA.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_kicker.runKicker(1),
    //         m_kicker));
/* 
new JoystickButton(m_driverController2, XboxController.Button.kB.value)
        .onTrue(new RunCommand(
            () -> m_robotDrive.setFieldRelativity(),
            m_robotDrive));
             
new JoystickButton(m_driverController2, XboxController.Button.kX.value)
        .onTrue(new RunCommand(
               () -> m_kicker.runKicker(1),
                m_kicker));
            
*/       
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    return new PathPlannerAuto(m_chooser.getSelected());
  }
}
 