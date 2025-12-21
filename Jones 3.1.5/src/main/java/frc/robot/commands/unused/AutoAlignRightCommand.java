package frc.robot.commands.unused;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.unused.AutoAlignSubsystem;

public class AutoAlignRightCommand extends AutoAlignCommand {
    public AutoAlignRightCommand(DriveSubsystem driveSubsystem, AutoAlignSubsystem autoAlignSubsystem) {
        super(driveSubsystem, autoAlignSubsystem, 6.5);  // Offset of 6.5 inches for right
    }
    public void end(boolean interrupted) {
        System.out.println("auto align interupted");
    }
}
