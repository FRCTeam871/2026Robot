package frc.robot.subsystem.turret;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

public interface TurretIO {
    TurretIO EMPTY = new TurretIO() {};

    @AutoLog
    public class turretIOInputs {
    }
    default void runTurretMotor(double speed) {}
}
