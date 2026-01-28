package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public interface ShooterIO {
    ShooterIO EMPTY = new ShooterIO() {};

    @AutoLog
    class ShooterIOInputs {
        AngularVelocity velocity = Units.RotationsPerSecond.of(0);
        Voltage motorVoltage = Units.Volts.of(0);
        Angle position = Units.Rotations.of(0);
    }

    default void updateInputs(ShooterIOInputs inputs) {}

    
    default void setMotorSetpoint(AngularVelocity rpmSetpoint) {};
    default void runMotorSpeed(double speed) {};
    default void setVoltage(Voltage v) {};
}
