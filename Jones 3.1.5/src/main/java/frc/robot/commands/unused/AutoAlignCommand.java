package frc.robot.commands.unused;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.unused.AutoAlignSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAlignCommand extends Command {
    
    private final DriveSubsystem driveSubsystem;
    private final AutoAlignSubsystem autoAlignSubsystem;

    // Proportional control constants
    private static final double kTurnP = 0.02;  // Proportional gain for turning
    private static final double kDistanceP = 0.05;  // Proportional gain for distance adjustment
    private final double offset;  // The offset in inches to apply (positive for right, negative for left)

    public AutoAlignCommand(DriveSubsystem driveSubsystem, AutoAlignSubsystem autoAlignSubsystem, double offset) {
        this.driveSubsystem = driveSubsystem;
        this.autoAlignSubsystem = autoAlignSubsystem;
        this.offset = offset;  // Accepting offset as a parameter
        
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Fetch Limelight data
        double[] limelightData = autoAlignSubsystem.getLimelightData();

        // Check if a valid target is detected
        if (autoAlignSubsystem.isValidTarget(limelightData)) {
            double tx = limelightData[0];  // Horizontal offset (rotation correction)
            double ty = limelightData[1];  // Vertical offset (distance correction)

            // Apply proportional control to adjust robot position
            double turnSpeed = tx * kTurnP;  // Adjust turn speed based on horizontal offset
            double distanceSpeed = ty * kDistanceP;  // Adjust forward/backward speed based on vertical offset

            // Apply offset (adjust the horizontal alignment)
            double offsetAdjustment = offset * 1.0;  // Scale the offset distance appropriately (1.0 means 1-to-1 scaling)

            // Adjust turnSpeed with the offset for left or right offset
            turnSpeed += offsetAdjustment;

            // Apply turn and distance adjustment (using field-relative control)
            driveSubsystem.drive(0, distanceSpeed, turnSpeed, true);  // Moving forward/backward and rotating to align
        } else {
            // If no valid target, stop the robot or perform fallback actions (e.g., error handling)
            driveSubsystem.drive(0, 0, 0, true);  // Stop the robot
        }
    }

    @Override
    public boolean isFinished() {
        // Command finishes when robot is aligned with the target
        double[] limelightData = autoAlignSubsystem.getLimelightData();
        return autoAlignSubsystem.isAligned(limelightData);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, true);  // Stop robot if command is interrupted or finished
        System.out.println("auto align interupted");
    }
}
