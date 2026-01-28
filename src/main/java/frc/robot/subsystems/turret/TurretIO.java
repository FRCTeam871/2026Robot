package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.geometry.Pose3d;

public interface TurretIO {
    TurretIO EMPTY = new TurretIO() {};

    @AutoLog
    public class turretIOInputs {
    }
    default void runTurretMotor(double speed) {}
}
