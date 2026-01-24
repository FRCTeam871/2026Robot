package frc.robot.subsystem.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {

    IndexerIO EMPTY = new IndexerIO() {};

    @AutoLog
    class IndexerIOInputs {

    }

    default void runIndexMotor(double speed) {}

    default void updateInputs(IndexerIOInputs inputs) {
        // holy jimy
    }

}
