package frc.robot.subsystems.swervedrive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveDriveIO {
    SwerveDriveIO EMPTY = new SwerveDriveIO() {};

    @AutoLog
    class SwerveDriveIOInputs {
        Rotation2d gyroRotation = new Rotation2d(0);
        boolean isCalibrating;
        double gyroRate;
    }

    default void setCurrentAngle(double angle) {}

    default void updateInputs(SwerveDriveIOInputs inputs) {}
}
