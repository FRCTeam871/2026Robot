package frc.robot.controls;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface IControls {
    public boolean isOk();

    // positive is forwards
    public DoubleSupplier forwardsAndBackAxis();

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

    public Trigger runIntakePiston();

    public Trigger runSequence();

    public Trigger shoot();

    public DoubleSupplier runTurretPID();

    public Trigger resetGyro();

    public Trigger regurgitate();
}