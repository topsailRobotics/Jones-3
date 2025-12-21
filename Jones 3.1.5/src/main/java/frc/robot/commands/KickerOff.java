package frc.robot.commands;

import frc.robot.subsystems.KickerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
// Example  "1"  Command
/** An example command that uses an example subsystem. */
public class KickerOff extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final KickerSubsystem m_kicker;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public KickerOff(KickerSubsystem subsystem) {
    this.m_kicker = subsystem;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Kicker off command initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_kicker.stopKicker(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Kicker off command interruted");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}