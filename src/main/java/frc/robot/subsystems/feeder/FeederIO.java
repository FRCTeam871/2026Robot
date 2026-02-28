package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
    FeederIO EMPTY = new FeederIO() {
    };

    @AutoLog
    public class feederIOInputs {
    }

    default void runFeederMotor(double speed) {
    }
}
