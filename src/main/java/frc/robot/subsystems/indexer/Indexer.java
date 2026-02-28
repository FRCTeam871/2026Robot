package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputs inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command runIndexMotor(double speed) {
        return run(() -> {
            io.runIndexMotor(speed);
        }).finallyDo(() -> {
            io.runIndexMotor(0);
        });
    }
}
