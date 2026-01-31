// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Collections;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.units.Units;
import frc.robot.controls.IControls;
import frc.robot.controls.XboxControls;
import frc.robot.subsystems.Aiming;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOReal;
import frc.robot.subsystems.fieldtracking.FieldTracking;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOReal;
import frc.robot.subsystems.swerveModule.SwerveModule;
import frc.robot.subsystems.swerveModule.SwerveModuleIO;
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
    final IControls controls;

    public RobotContainer() {

        FeederIO feederIO = FeederIO.EMPTY;
        ShooterIO shooterIO = ShooterIO.EMPTY;
        IndexerIO indexerIO = IndexerIO.EMPTY;
        TurretIO turretIO = TurretIO.EMPTY;
        IntakeIO intakeIO = IntakeIO.EMPTY;
        compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
        this.controls = new XboxControls();
        
        if (RobotBase.isSimulation() && Constants.shouldReplay) { // is the world a simulation?
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        }
        
        if (RobotBase.isReal()) { // is it real?
            shooterIO = new ShooterIOReal();
            // indexerIO = new IndexerIOReal();
            // turretIO = new TurretIOReal();
            // intakeIO = new IntakeIOReal();
            // feederIO = new FeederIOReal();
        }
        // 0.0017509999452158809
        shooter = new Shooter(shooterIO);
        indexer = new Indexer(indexerIO);
        turret = new Turret(turretIO);
        intake = new Intake(intakeIO);
        feeder = new Feeder(feederIO);
        aiming = new Aiming(turret, shooter, null);
        configureBindings();
    }
    
    private void configureBindings() {
        
        // controls.FIREEEEEEEEEEEEEEEEE().whileTrue(shooter.runMotorSpeed(.15));
        // controls.fiREEEE().whileTrue(shooter.runMotorSpeed(.3));
        controls.fireLowPID().whileTrue(shooter.holdMotorSetpoint(Units.RPM.of(1100)));
        controls.fireHighPID().whileTrue(shooter.holdMotorSetpoint(Units.RPM.of(5000)));

        controls.runFeeder().whileTrue(feeder.runFeederMotor(.1));

        controls.runIndexer().whileTrue(indexer.runIndexMotor(() -> .1));

        controls.runIntake().whileTrue(intake.runIntakeMotor(() -> .1));

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
