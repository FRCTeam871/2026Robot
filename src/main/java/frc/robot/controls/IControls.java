package frc.robot.controls;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IControls {
    // positive is fowards
    public DoubleSupplier fowardsAndBackAxis();
    // positive is left
    public DoubleSupplier sideToSideAxis();
    // positive is counterclockwise
    public DoubleSupplier driveRotation();

    public DoubleSupplier runTurret();
    
    public Trigger FIREEEEEEEEEEEEEEEEE();

    public Trigger fiREEEE();

    public Trigger compressorToggle();

    public Trigger fireLowPID();

    public Trigger fireHighPID();

    public Trigger runIndexer();

    public Trigger runFeeder();

    public Trigger runIntake();

}