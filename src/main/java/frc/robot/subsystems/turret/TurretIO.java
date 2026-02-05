package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public interface TurretIO {
    TurretIO EMPTY = new TurretIO() {};

    @AutoLog
    public class TurretIOInputs {
        Angle turretAngle = Units.Degrees.of(0);
    }

    default void updateInputs(TurretIOInputs inputs) {
            inputs.turretAngle = Units.Degrees.of(50);
    }
    default void runTurretMotor(double speed) {}
}
