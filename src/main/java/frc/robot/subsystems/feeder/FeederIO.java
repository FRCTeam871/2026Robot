package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.math.geometry.Pose3d;

public interface FeederIO {
    FeederIO EMPTY = new FeederIO() {};

    @AutoLog
    public class feederIOInputs {
    }
    default void runFeederMotor(double speed) {}
}
