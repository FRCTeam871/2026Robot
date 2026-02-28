package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
    private final FeederIO io;

    public Feeder(FeederIO io) {
        this.io = io;
    }

    public Command runFeederMotor(double speed) {
        return run(() -> {
            io.runFeederMotor(speed);
        }).finallyDo(() -> {
            io.runFeederMotor(0);
        });
    }

    public void runDumbFeederMotor(double speed) {
        io.runFeederMotor(speed);
    }
}
