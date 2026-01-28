package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public interface IntakeIO {
    IntakeIO EMPTY = new IntakeIO() {};
    
    @AutoLog
    class IntakeIOInputs {
        // pas
    }

    default void updateInputs(IntakeIOInputs inputs) {}

    default void setSpeed(double speed) {}

    default void setIntakeOut(boolean extend) {
        Logger.recordOutput("Intake/Piston", extend);
    }
}