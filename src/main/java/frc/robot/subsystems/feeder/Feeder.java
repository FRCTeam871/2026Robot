package frc.robot.subsystems.feeder;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private final FeederIO io;

    public Feeder(FeederIO io) {
        this.io = io;
    }

    public Command runFeederMotor(DoubleSupplier speed) {
        return run(() -> {
            io.runFeederMotor(speed.getAsDouble());
            Logger.recordOutput("Feeder/speed", speed);
        }).finallyDo(() -> {
            io.runFeederMotor(0);
            Logger.recordOutput("Feeder/speed", 0);
        });
    }

    public void runDumbFeederMotor(double speed) {
        io.runFeederMotor(speed);
    }
}
