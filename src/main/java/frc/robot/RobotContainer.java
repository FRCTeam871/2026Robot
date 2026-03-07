// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Collections;
import java.util.function.BooleanSupplier;
import java.util.stream.IntStream;

import javax.sound.midi.Sequence;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ModuleConstants;
import frc.robot.controls.IControls;
import frc.robot.controls.XboxControls;
import frc.robot.subsystems.Aiming;
import frc.robot.subsystems.Sequencing;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.fieldtracking.FieldTracking;
import frc.robot.subsystems.fieldtracking.FieldTrackingIO;
import frc.robot.subsystems.fieldtracking.FieldTrackingIO.IMUMode;
import frc.robot.subsystems.fieldtracking.FieldTrackingIOLimeLight;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOReal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.swerveModule.SwerveModule;
import frc.robot.subsystems.swerveModule.SwerveModuleIO;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import frc.robot.subsystems.swervedrive.SwerveDriveIO;
import frc.robot.subsystems.swervedrive.SwerveDriveIOYaw;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;
import frc.robot.subsystems.turret.TurretIOSim;

public class RobotContainer {
    final Shooter shooter;
    final Indexer indexer;
    final Turret turret;
    final Intake intake;
    final Feeder feeder;
    final Compressor compressor;
    final Aiming aiming;
    final Sequencing sequencing;
    final FieldTracking fieldTracking;
    final SwerveDrive swerveDrive;
    final IControls controls;
    final AutonomousPlanner autonomousPlanner;

    public RobotContainer() {

        FieldTrackingIO fieldTrackingIO = FieldTrackingIO.EMPTY;
        FeederIO feederIO = FeederIO.EMPTY;
        ShooterIO shooterIO = ShooterIO.EMPTY;
        IndexerIO indexerIO = IndexerIO.EMPTY;
        TurretIO turretIO = new TurretIOSim();
        IntakeIO intakeIO = IntakeIO.EMPTY;
        SwerveModuleIO[] moduleIOs = Collections.nCopies(4, SwerveModuleIO.EMPTY).toArray(SwerveModuleIO[]::new);
        SwerveDriveIO swerveDriveIO = SwerveDriveIO.EMPTY;
        compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
        this.controls = new XboxControls();

        if (RobotBase.isSimulation() && Constants.shouldReplay) { // is the world a simulation?
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }

        if (RobotBase.isReal()) { // is it real?
            // fieldTrackingIO = new FieldTrackingIOLimeLight();
            shooterIO = new ShooterIOReal();
            indexerIO = new IndexerIOReal();
            turretIO = new TurretIOReal();
            intakeIO = new IntakeIOReal();
            feederIO = new FeederIOReal();
            // moduleIOs = Arrays.stream(
            //         Constants.MODULE_CONSTANTS)
            //         .map(Constants::getRealSwerveModuleIO)
            //         .toArray(SwerveModuleIO[]::new);
            // swerveDriveIO = new SwerveDriveIOYaw(new AHRS(NavXComType.kMXP_SPI));

        }
        final SwerveModuleIO[] moduleIOsFinal = moduleIOs;
        final SwerveModule[] swerveModules = IntStream.range(0, moduleIOs.length)
                .mapToObj(i -> {
                    final SwerveModuleIO io = moduleIOsFinal[i];
                    final ModuleConstants constants = Constants.MODULE_CONSTANTS[i];
                    return new SwerveModule(constants.leverArm(), io, constants.label());
                })
                .toArray(SwerveModule[]::new);
        swerveDrive = new SwerveDrive(swerveDriveIO, swerveModules);

        shooter = new Shooter(shooterIO);
        indexer = new Indexer(indexerIO);
        turret = new Turret(turretIO, swerveDrive);
        intake = new Intake(intakeIO);
        feeder = new Feeder(feederIO);
        sequencing = new Sequencing(shooter, intake, indexer, feeder);
        fieldTracking = new FieldTracking(swerveDrive, fieldTrackingIO);
        aiming = new Aiming(turret, shooter, fieldTracking, swerveDrive, sequencing);
        autonomousPlanner = new AutonomousPlanner(intake, swerveDrive, fieldTracking, aiming);
        configureBindings();
    }

    private void configureBindings() {

        new EventTrigger("Intake").whileTrue(intake.runIntakeMotor(() -> Constants.ocIntakeMotorSpeed));
        // controls.FIREEEEEEEEEEEEEEEEE().whileTrue(shooter.runMotorSpeed(.15));
        // controls.fiREEEE().whileTrue(shooter.runMotorSpeed(.3));
        controls.fireLowPID().whileTrue(shooter.holdMotorSetpoint(Units.RPM.of(1100)));
        controls.fireHighPID().whileTrue(shooter.holdMotorSetpoint(Units.RPM.of(5600)));

        // controls.fireHighPID().whileTrue(shooter.quasiStatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        // controls.fireLowPID().whileTrue(shooter.quasiStatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
        // controls.fiREEEE().whileTrue(shooter.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward));
        // controls.FIREEEEEEEEEEEEEEEEE().whileTrue(shooter.dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse));
        // 5600 rpm = 10.015m/s @ 52.5 deg = 29ft += 1.5 ft
        // 4200 rpm = 8.763125m/s          = 24 ft += 1 ft
        // 3500 rpm = 7.7894m/s            =xxxx 20 ft += 2 in

        // turret.setDefaultCommand(turret.runTurretMotor(controls.runTurretPID()));
        controls.runFeeder().whileTrue(feeder.runFeederMotor(()-> .5));

        controls.runIndexer().whileTrue(indexer.runIndexMotor(()->.3));

        controls.runIntake().whileTrue(intake.runIntakeMotor(() -> -.5));
        controls.runIntakePiston().toggleOnTrue(intake.sendIntakeOut()); // first

        controls.runSequence().whileTrue(sequencing.shooterCommand(() -> Units.RPM.of(2000)));

        controls.shoot().whileTrue(aiming.shootTrue());

        // controls.compressorToggle().onTrue(Commands.runOnce(() -> {
        //     if (compressor.isEnabled()) {
        //         compressor.disable();
        //     } else {
        //         compressor.enableDigital();
        //     }
        // }));
    }

    public Command getAutonomousCommand() {
        return autonomousPlanner.getAutonCommand();
    }

    public void autonomousInit() {
        // fieldTracking.setCameraIMUMode(IMUMode.InternalExternalAssist);
        fieldTracking.setCameraIMUMode(IMUMode.ExternalOnly);
        fieldTracking.setThrottle(0);

        final Command autoCommand = getAutonomousCommand();

        if (autoCommand != null) {
            CommandScheduler.getInstance().schedule(autoCommand);
        }
    }

    public void robotPeriodic() {
        autonomousPlanner.periodic();
    }

    public void teleopInit() {
        swerveDrive.setDefaultCommand(swerveDrive.manualDrive(
                controls.forwardsAndBackAxis(),
                controls.sideToSideAxis(),
                controls.driveRotation()));

    }
}
