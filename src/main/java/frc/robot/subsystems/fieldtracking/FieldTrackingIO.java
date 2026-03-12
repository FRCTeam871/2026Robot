package frc.robot.subsystems.fieldtracking;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface FieldTrackingIO {
    FieldTrackingIO EMPTY = new FieldTrackingIO() {
    };

    class FieldTrackingIOInputs {
        LimelightIOAutoLogged[] limelights = new LimelightIOAutoLogged[0];
    }

    public class FieldTrackingIOInputsAutoLogged extends FieldTrackingIOInputs implements LoggableInputs {

        @Override
        public void toLog(LogTable table) {
            table.put("limelightsCount", limelights.length);
            for (int i = 0; i < limelights.length; i++) {
                table.put("limelight" + i, limelights[i]);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            int limelightCount = table.get("limelightsCount", 0);
            limelights = new LimelightIOAutoLogged[limelightCount];
            for (int i = 0; i < limelightCount; i++) {
                limelights[i] = table.get("limelight" + i, new LimelightIOAutoLogged());

            }
        }
    }

    @AutoLog
    class LimelightIO {
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

    default void updateInputs(FieldTrackingIOInputs inputs) {
    }

    default void setRobotOrientation(double degrees) {
    }

    default void setCameraIMUMode(IMUMode imuMode) {
    }

    default void setCameraThrottle(int throttle) {
    }

    default void setIMUAssistAlpha(double alpha) {
    }

    default void clip(double durationSeconds){
    }
}
