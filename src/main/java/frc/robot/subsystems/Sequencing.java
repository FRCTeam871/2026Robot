package frc.robot.subsystems;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.swervedrive.SwerveDrive;

public class Sequencing extends SubsystemBase {
    Shooter shooter;
    Intake intake;
    Indexer indexer;
    Feeder feeder;
    public Sequencing(Shooter shooter, Intake intake, Indexer indexer, Feeder feeder){
        this.shooter = shooter;
        this.indexer = indexer;
        this.intake = intake;
        this.feeder = feeder;
    }
    public Command shooterCommand(AngularVelocity rpmSetpoint) {

        // alongWith runs 2 commands at once
        return shooter.holdMotorSetpoint(rpmSetpoint)
            .alongWith(
                new ConditionalCommand(feeder.runFeederMotor(1), feeder.runFeederMotor(0), () -> shooter.isAtGoalRPM())); // <- This lambda converts the command into a BooleanSupplier function

        // return run(() -> {
        //     shooter.holdMotorSetpoint(rpmSetpoint);
        //     if (shooter.isAtGoalRPM()) {
        //         feeder.runDumbFeederMotor(1);
        //     }
        // });
    }
}
