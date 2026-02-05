package frc.robot.controls;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class XboxControls implements IControls {

    CommandXboxController driveXboxController;
    CommandXboxController systemXboxController;

    public XboxControls() {
        driveXboxController = new CommandXboxController(0); // xboxcontroller 1
        systemXboxController = new CommandXboxController(1); // xbox controlr 2
    }

    @Override
    public DoubleSupplier fowardsAndBackAxis() {
        return () -> Constants.deadbandAndExponential(-driveXboxController.getLeftY());
    }

    @Override
    public DoubleSupplier sideToSideAxis() {
        return () -> Constants.deadbandAndExponential(-driveXboxController.getLeftX());
    }

    @Override
    public DoubleSupplier driveRotation() {
        return () -> Constants.deadbandAndExponential(-driveXboxController.getRightX())*.6;
    }

    @Override
    public DoubleSupplier runTurret() {
        return () -> driveXboxController.getRightTriggerAxis();
    }

    @Override
    public Trigger FIREEEEEEEEEEEEEEEEE() {
        return driveXboxController.x();
    }

    @Override
    public Trigger compressorToggle() {
        return driveXboxController.back();
    }
    @Override
    public Trigger fiREEEE() {
        return driveXboxController.leftBumper();
    }
    @Override
    public Trigger fireLowPID() {
        return driveXboxController.povRight();
    }
    @Override
    public Trigger fireHighPID() {
        return driveXboxController.povLeft();
    }
    @Override
    public Trigger runFeeder() {
        return driveXboxController.leftBumper();
    }
    @Override
    public Trigger runIndexer() {
       return driveXboxController.rightBumper();
    }
    @Override
    public Trigger runIntake() {
        return driveXboxController.a();
    }

    @Override
    public Trigger runIntakePiston() {
        return driveXboxController.y();
    }
}