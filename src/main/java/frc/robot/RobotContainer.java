// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Collections;
import java.util.stream.IntStream;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.units.Units;
import frc.robot.Constants.ModuleConstants;
import frc.robot.controls.IControls;
import frc.robot.controls.XboxControls;
import frc.robot.subsystems.Aiming;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.fieldtracking.FieldTracking;
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
import frc.robot.subsystems.swervedrive.SwerveDriveIORoll;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIO;
import frc.robot.subsystems.turret.TurretIOReal;

public class RobotContainer {
    Shooter shooter;
    Indexer indexer;
    Turret turret;
    Intake intake;
    Feeder feeder;
    Compressor compressor;
    Aiming aiming;
    FieldTracking fieldTracking;
    SwerveDrive swerveDrive;
    final IControls controls;

    public RobotContainer() {

        FeederIO feederIO = FeederIO.EMPTY;
        ShooterIO shooterIO = ShooterIO.EMPTY;
        IndexerIO indexerIO = IndexerIO.EMPTY;
        TurretIO turretIO = TurretIO.EMPTY;
        IntakeIO intakeIO = IntakeIO.EMPTY;
        // SwerveModuleIO[] moduleIOs = Collections.nCopies(4, SwerveModuleIO.EMPTY).toArray(SwerveModuleIO[]::new);
        // SwerveDriveIO swerveDriveIO = SwerveDriveIO.EMPTY;
        compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
        this.controls = new XboxControls();
        
        if (RobotBase.isSimulation() && Constants.shouldReplay) { // is the world a simulation?
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }
        
        if (RobotBase.isReal()) { // is it real?
            shooterIO = new ShooterIOReal();
            indexerIO = new IndexerIOReal();
            turretIO = new TurretIOReal();
            intakeIO = new IntakeIOReal();
            feederIO = new FeederIOReal();
            // moduleIOs = Arrays.stream(
            //         Constants.MODULE_CONSTANTS)
            //         .map(Constants::getRealSwerveModuleIO)
            //         .toArray(SwerveModuleIO[]::new);
            // swerveDriveIO = new SwerveDriveIORoll(new AHRS(NavXComType.kMXP_SPI));

        }
        // 0.0017509999452158809
        // final SwerveModuleIO[] moduleIOsFinal = moduleIOs;
        // final SwerveModule[] swerveModules = IntStream.range(0, moduleIOs.length)
        //         .mapToObj(i -> {
        //             final SwerveModuleIO io = moduleIOsFinal[i];
        //             final ModuleConstants constants = Constants.MODULE_CONSTANTS[i];
        //             return new SwerveModule(constants.leverArm(), io, constants.label());
        //         })
        //         .toArray(SwerveModule[]::new);
        // swerveDrive = new SwerveDrive(swerveDriveIO,swerveModules);

        shooter = new Shooter(shooterIO);
        indexer = new Indexer(indexerIO);
        turret = new Turret(turretIO,swerveDrive);
        intake = new Intake(intakeIO);
        feeder = new Feeder(feederIO);
        // aiming = new Aiming(turret, shooter, null, swerveDrive);
        configureBindings();
    }
    
    private void configureBindings() {
        
        // controls.FIREEEEEEEEEEEEEEEEE().whileTrue(shooter.runMotorSpeed(.15));
        // controls.fiREEEE().whileTrue(shooter.runMotorSpeed(.3));
        controls.fireLowPID().whileTrue(shooter.holdMotorSetpoint(Units.RPM.of(1100)));
        controls.fireHighPID().whileTrue(shooter.holdMotorSetpoint(Units.RPM.of(5000)));

        controls.runFeeder().whileTrue(feeder.runFeederMotor(-.5));

        controls.runIndexer().whileTrue(indexer.runIndexMotor(() -> .4));

        controls.runIntake().whileTrue(intake.runIntakeMotor(() -> -.5));
        controls.runIntakePiston().toggleOnTrue(intake.sendIntakeOut());

        turret.setDefaultCommand(turret.runTurretMotor(controls.runTurret()));
        controls.compressorToggle().onTrue(Commands.runOnce(() -> {
            if (compressor.isEnabled()) {
                compressor.disable();
            } else {
                compressor.enableDigital();
            }
        }));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
