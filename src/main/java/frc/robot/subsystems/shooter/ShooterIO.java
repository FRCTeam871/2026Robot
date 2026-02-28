package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public interface ShooterIO {
    ShooterIO EMPTY = new ShooterIO() {
    };

    @AutoLog
    class ShooterIOInputs {
        AngularVelocity velocity = Units.RotationsPerSecond.of(0);
        Voltage motorVoltage = Units.Volts.of(0);
        Angle position = Units.Rotations.of(0);
        boolean isAtRPMSetpoint = false;
    }

    default void updateInputs(ShooterIOInputs inputs) {
    }

    default void setMotorSetpoint(AngularVelocity rpmSetpoint) {
    };

    default void runMotorSpeed(double speed) {
    };

    default void setVoltage(Voltage v) {
    };
}
