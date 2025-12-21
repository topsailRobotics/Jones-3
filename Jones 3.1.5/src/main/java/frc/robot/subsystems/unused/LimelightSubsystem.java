package frc.robot.subsystems.unused;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class LimelightSubsystem extends SubsystemBase {

    @Override
    public void periodic() {
        SmartDashboard.putNumber("limelight horizontal offset", LimelightHelpers.getTX("limelight"));
        SmartDashboard.putNumber("limelight screen percent", LimelightHelpers.getTA("limelight"));

        
    }

        public void AutoAlign(DriveSubsystem DriveSubsystem) {

            if (LimelightHelpers.getTV("limelight") == true) {
                DriveSubsystem.drive(0, 0, 1, true);
            }
            if (LimelightHelpers.getTV("limelight") == false) {
                DriveSubsystem.drive(0, 0, 0, true);
            }
        
        }
        

    }


