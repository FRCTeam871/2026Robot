package frc.robot.subsystems.fieldtracking;

import java.security.Key;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;

public class FieldTrackingIOLimeLight implements FieldTrackingIO {
    final String[] limelightKeys = new String[] {
            "limelight-back",
            "limelight-left"
    };

    @Override
    public void updateInputs(FieldTrackingIOInputs inputs) {
        if (inputs.limelights == null || inputs.limelights.length != limelightKeys.length) {
            inputs.limelights = new LimelightIOAutoLogged[limelightKeys.length];
        }
        for (int i = 0; i < inputs.limelights.length; i++) {
            if (inputs.limelights[i] == null) {
                inputs.limelights[i] = new LimelightIOAutoLogged();
            }
            String key = limelightKeys[i];
            inputs.limelights[i].tid = NetworkTableInstance.getDefault()
                    .getTable(key)
                    .getEntry("tid")
                    .getInteger(-1);
            if (NetworkTableInstance.getDefault().getTable(key).containsKey("botpose_orb_wpiblue")) {
                LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(key);
                inputs.limelights[i].pose = mt2.pose;
                inputs.limelights[i].timestampSeconds = mt2.timestampSeconds;
                inputs.limelights[i].tagCount = mt2.tagCount;
            }
            inputs.limelights[i].targetpose_robotspace = NetworkTableInstance.getDefault()
                    .getTable(key)
                    .getEntry("targetpose_robotspace")
                    .getDoubleArray(new double[6]);

            inputs.limelights[i].on = NetworkTableInstance.getDefault().getTable(key)
                    .containsKey("botpose_orb_wpiblue");

        }
    }

    @Override
    public void setRobotOrientation(double degrees) {
        for (String key : limelightKeys) {
            LimelightHelpers.SetRobotOrientation(key, degrees, 0, 0, 0, 0, 0);
        }
    }

    @Override
    public void setCameraIMUMode(IMUMode imuMode) {
        for (String key : limelightKeys) {
            LimelightHelpers.SetIMUMode(key, imuMode.limeLightConstant);
        }
    }

    @Override
    public void setIMUAssistAlpha(double alpha) {
        for (String key : limelightKeys) {
            LimelightHelpers.SetIMUAssistAlpha(key, alpha);
        }
    }

    @Override
    public void setCameraThrottle(int throttle) {
        for (String key : limelightKeys) {
            LimelightHelpers.SetThrottle(key, throttle);
        }
    }

    @Override
    public void clip(double durationSeconds) {
        for (String key : limelightKeys) {
            LimelightHelpers.triggerRewindCapture(key, durationSeconds);
        }
    }
}
