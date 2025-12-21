// Langgang was here :)  8========
package frc.robot.subsystems.unused;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AutoAlignSubsystem {

    public double[] getLimelightData() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-two");
        NetworkTableEntry tx = table.getEntry("tx");  // Horizontal offset
        NetworkTableEntry ty = table.getEntry("ty");  // Vertical offset
        NetworkTableEntry ta = table.getEntry("ta");  // Target area
        NetworkTableEntry tid = table.getEntry("tid"); // Tag ID (optional for tracking purposes)

        // Read values from the Limelight
        double x = tx.getDouble(0.0);  // Horizontal offset
        double y = ty.getDouble(0.0);  // Vertical offset
        double area = ta.getDouble(0.0);  // Area of the target
        double tagId = tid.getDouble(-1.0); // AprilTag ID (could be used for logging or debugging)

        // Store the data and return it
        double[] limelightData = new double[4];
        limelightData[0] = x;  // Horizontal offset
        limelightData[1] = y;  // Vertical offset
        limelightData[2] = area;  // Area of the target
        limelightData[3] = tagId;  // Tag ID for logging or tracking purposes
        return limelightData;
    }

    public boolean isValidTarget(double[] limelightData) {
        return limelightData[2] > 0;  // Target is valid if the area (limelightData[2]) is greater than 0
    }

    public boolean isAligned(double[] limelightData) {
        // The robot is aligned if the horizontal offset (tx) is within a small tolerance (e.g., 2 degrees)
        return Math.abs(limelightData[0]) < 2.0;  // Adjust this threshold based on your robot's needs
    }
}
