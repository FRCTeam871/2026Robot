package frc.robot.subsystems.fieldtracking;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface FieldTrackingIO {
    FieldTrackingIO EMPTY = new FieldTrackingIO() {};

    @AutoLog
    class FieldTrackingIOInputs {
        int tagCount;
        Pose2d pose = new Pose2d();
        double timestampSeconds;
        long tid;
        double[] targetpose_robotspace;
        boolean on;
    }

    enum IMUMode {
        /** Only external (navx) angle used */
        ExternalOnly(0),
        /** External angle used, internal angle reset to match */
        ExternalReset(1),
        /** Only internal angle used */
        InternalOnly(2),
        /** Only internal angle used, also factors in MT1 angle from apriltags */
        InternalMT1Assist(3),
        /** Internal angle used, also factors in external angle while not moving */
        InternalExternalAssist(4);

        public int limeLightConstant;

        IMUMode(int limeLightConstant) {
            this.limeLightConstant = limeLightConstant;
        }
    }

    default void updateInputs(FieldTrackingIOInputs inputs) {}

    default void setRobotOrientation(double degrees) {}

    default void setCameraIMUMode(IMUMode imuMode) {}

    default void setCameraThrottle(int throttle) {}

    default void setIMUAssistAlpha(double alpha) {}
}
