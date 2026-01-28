package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    SparkFlex intakeMotor;
    IntakeIO io;

    public Intake(IntakeIO io){
        this.io = io;
        io.setIntakeOut(false);
    }

    public Command runIntakeMotor(DoubleSupplier speed){
        return run(()-> {
            io.setSpeed(speed.getAsDouble());
        }).finallyDo(()->{
            io.setSpeed(0);
        });
    }   
    public Command sendIntakeOut() {
        return Commands.run(() -> io.setIntakeOut(true))
                .finallyDo(canceled -> io.setIntakeOut(false))
                .ignoringDisable(true);
    }
}
