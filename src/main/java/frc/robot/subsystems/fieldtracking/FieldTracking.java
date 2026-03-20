package frc.robot.subsystems.fieldtracking;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.fieldtracking.FieldTrackingIO.FieldTrackingIOInputsAutoLogged;
import frc.robot.subsystems.fieldtracking.FieldTrackingIO.IMUMode;
import frc.robot.subsystems.fieldtracking.FieldTrackingIO.LimelightIO;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

public class FieldTracking extends SubsystemBase {
    private final ProfiledPIDController sidePidController;
    private final ProfiledPIDController forwardPidController;
    private final ProfiledPIDController yawPidController;
    private final SwerveDrive swerveDrive;
    private final FieldTrackingIO io;
    private final FieldTrackingIOInputsAutoLogged inputs = new FieldTrackingIOInputsAutoLogged();
    private final AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public FieldTracking(final SwerveDrive swerveDrive, final FieldTrackingIO io) {
        this.sidePidController = new ProfiledPIDController(2.5, 0.001, .1, new Constraints(1000, 1000));
        sidePidController.setTolerance(.06);
        this.forwardPidController = new ProfiledPIDController(2.5, 0.001, .1, new Constraints(1000, 1000));
        forwardPidController.setTolerance(.1);
        this.yawPidController = new ProfiledPIDController(0.08, 0.001, 0, new Constraints(1000, 1000));
        yawPidController.setTolerance(5);
        yawPidController.enableContinuousInput(0, 360);
        this.swerveDrive = swerveDrive;
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("FieldTracking", inputs);

        io.setRobotOrientation(swerveDrive.getEstimatedPose().getRotation().getDegrees());
        for (int i = 0; i < inputs.limelights.length; i++) {
            // if our angular velocity is greater than 720 degrees per
            // second, ignore vision updates
            if (Math.abs(swerveDrive.getYawRate()) > 720) {
                Logger.recordOutput("FieldTracking/TargetPoses" + i, Constants.EMPTY_POSE_ARRAY);
                // Logger.recordOutput("FieldTracking/TargetIDs", Constants.EMPTY_INT_ARRAY);
            } else if (inputs.limelights[i].tagCount == 0) {
                Logger.recordOutput("FieldTracking/TargetPoses" + i, Constants.EMPTY_POSE_ARRAY);
                // Logger.recordOutput("FieldTracking/TargetIDs", Constants.EMPTY_INT_ARRAY);

            } else {
                final Optional<Pose3d> pose = fieldLayout.getTagPose((int) inputs.limelights[i].tid);
                if (pose.isPresent() && !Double.isNaN(inputs.limelights[i].pose.getX())
                        && !Double.isNaN(inputs.limelights[i].pose.getY())) {
                    swerveDrive.addVisionMeasurement(
                            inputs.limelights[i].pose, inputs.limelights[i].timestampSeconds,
                            VecBuilder.fill(.7, .7, 9999999));
                    Logger.recordOutput("FieldTracking/Pose" + i, inputs.limelights[i].pose);

                    Logger.recordOutput("FieldTracking/TargetPoses" + i, new Pose3d[] { pose.get() });
                    // Logger.recordOutput("FieldTracking/TargetIDs", new int[] {(int) inputs.limelights[0].tid});
                } else {
                    Logger.recordOutput("FieldTracking/TargetPoses" + i, Constants.EMPTY_POSE_ARRAY);
                    // Logger.recordOutput("FieldTracking/TargetIDs", Constants.EMPTY_INT_ARRAY);
                }
            }
        }
    }

    public boolean isAprilTagDetected() {
        for (LimelightIO limelight : inputs.limelights) {
            if (limelight.tid != -1) {
                return true;
            }
        }
        return false;
    }

    // public long getAprilTag() {
    //     return inputs.limelights[0].tid;
    // }

    // public Pose2d getLimeLightPose() {
    //     return inputs.limelights[0].pose;
    // }
    // public Command followAprilTag() {
    // return run(() -> {
    // if (!isAprilTagDetected()) {
    // ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    // swerveDrive.updateSpeed(speeds); // this funciton sets everything to 0
    // return;
    // }

    // public Command followAprilTag() {
    //     return run(() -> {
    //         if (!isAprilTagDetected()) {
    //             final ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
    //             swerveDrive.updateSpeed(speeds);
    //             return;
    //         }

    //         // step one read output from limelight
    //         // in meters
    //         final double tx = -inputs.limelights[0].targetpose_robotspace[0];
    //         final double ty = inputs.limelights[0].targetpose_robotspace[1];
    //         final double tz = inputs.limelights[0].targetpose_robotspace[2];
    //         Logger.recordOutput("FieldTracking/tx", tx);
    //         Logger.recordOutput("FieldTracking/ty", ty);
    //         Logger.recordOutput("FieldTracking/tz", tz);

    //         // in degrees
    //         final double pitch = inputs.limelights[0].targetpose_robotspace[3];
    //         final double yaw = inputs.limelights[0].targetpose_robotspace[4];
    //         final double roll = inputs.limelights[0].targetpose_robotspace[5];
    //         // Logger.recordOutput("FieldTracking/pitch", pitch);
    //         Logger.recordOutput("FieldTracking/yaw", yaw);
    //         // Logger.recordOutput("FieldTracking/roll", roll);

    //         // step two feed values into pids
    //         final double xout = sidePidController.calculate(-tx, 0);
    //         final double zout = forwardPidController.calculate(-tz, 1);
    //         final double yawout = yawPidController.calculate(-yaw, 0);

    //         // step three take pid values and put it into swervedrive
    //         final ChassisSpeeds speeds = new ChassisSpeeds(zout, xout, yawout);

    //         swerveDrive.updateSpeed(speeds); // this will update the speeed
    //     })
    //             .until(this::isAtPosition);
    // }

    public Command maintainPose(final Pose2d poseToMaintain) {
        return run(() -> {
            Logger.recordOutput("FieldTracking/MaintainPose", poseToMaintain);

            final Pose2d relTgtPose = poseToMaintain.relativeTo(swerveDrive.getEstimatedPose());

            final double yout = sidePidController.calculate(0, relTgtPose.getY());
            final double xout = forwardPidController.calculate(0, relTgtPose.getX());
            final double yawout = yawPidController.calculate(
                    0, relTgtPose.getRotation().getDegrees());
            // Logger.recordOutput("FieldTracking/tyPID", yout);
            // Logger.recordOutput("FieldTracking/txPID", xout);

            final ChassisSpeeds speeds = new ChassisSpeeds(xout, yout, yawout);
            swerveDrive.updateSpeed(speeds); // this will update the speeed
        })
                .finallyDo(() -> Logger.recordOutput("FieldTracking/MaintainPose", new Pose2d()));
    }

    public boolean limeLightsOn() {
        for (LimelightIO limelight : inputs.limelights) {
            if (!limelight.on) {
                return false;
            }
        }
        return true;
    }

    public boolean isAtPosition() {
        return sidePidController.atGoal() && forwardPidController.atGoal() && yawPidController.atGoal();
    }

    public void setCameraIMUMode(final IMUMode imuMode) {
        io.setCameraIMUMode(imuMode);
    }

    public void setThrottle(final int throttle) {
        io.setCameraThrottle(throttle);
    }

    public void setIMUAssistAlpha(final double alpha) {
        io.setIMUAssistAlpha(alpha);
    }

    public void clip(double durationSeconds) {
        io.clip(durationSeconds);
    }
}