package frc.robot.subsystems.fieldtracking;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;

public class FieldTrackingIOLimeLight implements FieldTrackingIO {

    @Override
    public void updateInputs(FieldTrackingIOInputs inputs) {
        inputs.tid = NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("tid")
                .getInteger(-1);
                if(NetworkTableInstance.getDefault().getTable("limelight").containsKey("botpose_orb_wpiblue")){
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        inputs.pose = mt2.pose;
        inputs.timestampSeconds = mt2.timestampSeconds;
        inputs.tagCount = mt2.tagCount;
                }
        inputs.targetpose_robotspace = NetworkTableInstance.getDefault()
                .getTable("limelight")
                .getEntry("targetpose_robotspace")
                .getDoubleArray(new double[6]);

        inputs.on = NetworkTableInstance.getDefault().getTable("limelight").containsKey("botpose_orb_wpiblue");
    }

    @Override
    public void setRobotOrientation(double degrees) {
        LimelightHelpers.SetRobotOrientation("limelight", degrees, 0, 0, 0, 0, 0);
    }

    @Override
    public void setCameraIMUMode(IMUMode imuMode) {
        LimelightHelpers.SetIMUMode("limelight", imuMode.limeLightConstant);
    }

    @Override
    public void setIMUAssistAlpha(double alpha) {
        LimelightHelpers.SetIMUAssistAlpha("limelight", alpha);
    }

    @Override
    public void setCameraThrottle(int throttle) {
        LimelightHelpers.SetThrottle("limelight", throttle);
    }
}
