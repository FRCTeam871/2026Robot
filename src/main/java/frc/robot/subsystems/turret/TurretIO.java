package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;

public interface TurretIO {
    TurretIO EMPTY = new TurretIO() {
    };

    @AutoLog
    public class TurretIOInputs {
        Angle turretAngle = Units.Degrees.of(0);
        Angle setpointAngle = Units.Degrees.of(0);
        boolean softForwardLimit = false;
        boolean softReverseLimit = false;
    }

    default void updateInputs(TurretIOInputs inputs) {
    }

    default void setTarget(Angle angle) {
    }

    default void runDumn(double speed) {
    }
}
