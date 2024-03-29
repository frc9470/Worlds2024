package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;

import static frc.robot.Constants.LimelightConstants.*;

public class VisionSubsystem extends SubsystemBase {

    public VisionSubsystem() {

    }

    public double getDistance() {
        double angle = Math.toRadians(LimelightHelpers.getTY(""));
        return TARGET_HEIGHT_DIFFERENCE / Math.tan(angle+ Math.toRadians(LIMELIGHT_ANGLE));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance: ", getDistance());
    }


}
