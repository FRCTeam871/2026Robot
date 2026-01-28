package frc.robot.subsystems.swerveModule;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    SwerveModuleIO EMPTY = new SwerveModuleIO() {};

    @AutoLog
    class SwerveModuleIOInputs {
        double steeringAngleDegrees;
        double driveVelocity;
        double drivePosition;
    }

    default void updateInputs(final SwerveModuleIOInputs inputs) {}

    default void setDriveSpeed(final double speed) {}

    default void setSteerSpeed(final double speed) {}
}
