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
    public DoubleSupplier forwardsAndBackAxis() {
        return () -> Constants.deadbandAndExponential(-driveXboxController.getLeftY());
    }
    
    @Override
    public DoubleSupplier sideToSideAxis() {
        return () -> Constants.deadbandAndExponential(-driveXboxController.getLeftX());
    }
    
    @Override
    public DoubleSupplier driveRotation() {
        return () -> Constants.deadbandAndExponential(-driveXboxController.getRightX()) * .6;
    }
    
    @Override
    public Trigger shoot() {
        return driveXboxController.rightBumper();
    }

    @Override
    public Trigger runIntake() {
        return driveXboxController.leftBumper(); // hold
    }
    
    @Override
    public Trigger runSequence() {
        return driveXboxController.b(); // hold
    }
    
    @Override
    public Trigger runIntakePiston() {
        return driveXboxController.a();
    }
    
    @Override
    public Trigger compressorToggle() {
        return driveXboxController.back();
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
    public Trigger fiREEEE() {
        return driveXboxController.leftBumper();
    }

    @Override
    public Trigger fireLowPID() {
        return driveXboxController.povLeft();
    }

    @Override
    public Trigger fireHighPID() {
        return driveXboxController.leftTrigger();
    }

    @Override
    public Trigger runFeeder() {
        return driveXboxController.povUp();
    }

    @Override
    public Trigger runIndexer() {
        return driveXboxController.povRight();
    }



    @Override
    public DoubleSupplier runTurretPID() {
        return () -> driveXboxController.getRightY();
    }
}