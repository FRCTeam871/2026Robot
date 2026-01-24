// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.controls.IControls;
import frc.robot.controls.XboxControls;
import frc.robot.subsystem.indexer.Indexer;
import frc.robot.subsystem.intake.Intake;
import frc.robot.subsystem.shooter.Shooter;
import frc.robot.subsystem.shooter.ShooterIO;
import frc.robot.subsystem.shooter.ShooterIOReal;
import frc.robot.subsystem.turret.Turret;
import frc.robot.subsystem.turret.TurretIO;
import frc.robot.subsystem.turret.TurretIOReal;

public class RobotContainer {
  Shooter shooter;
  Indexer indexer;
  Turret turret;
  Intake intake;
  Compressor compressor;
  final IControls controls;

  public RobotContainer() {
    
    ShooterIO shooterIO = ShooterIO.EMPTY;
    // IndexerIO indexerIO = IndexerIO.EMPTY;
    TurretIO turretIO = TurretIO.EMPTY;
    // IntakeIO intakeIO = IntakeIO.EMPTY;
    compressor = new Compressor(1, PneumaticsModuleType.CTREPCM);
    this.controls = new XboxControls();

    if (RobotBase.isSimulation() && Constants.shouldReplay) {                                   // is the world a simulation?
      String logPath = LogFileUtil.findReplayLog();
      Logger.setReplaySource(new WPILOGReader(logPath));
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }

    configureBindings();
    if (RobotBase.isReal()) {                                         // is it real?
      shooterIO = new ShooterIOReal();  
      // indexerIO = new IndexerIOReal();
      turretIO = new TurretIOReal();
      // intakeIO = new IntakeIOReal();
    }

    shooter = new Shooter(shooterIO);
    // indexer = new Indexer(indexerIO);
    turret = new Turret(turretIO);
    // intake = new Intake(intakeIO);
  }

  private void configureBindings() {

    // controller.y().whileTrue(shooter.quasiStatic(Direction.kForward));
    // controller.b().whileTrue(shooter.dynamic(Direction.kForward));
    // controller.a().whileTrue(shooter.quasiStatic(Direction.kReverse));
    // controller.x().whileTrue(shooter.dynamic(Direction.kReverse));

    controls.FIREEEEEEEEEEEEEEEEE().whileTrue(shooter.runMotorSpeed(.15));
    controls.fiREEEE().whileTrue(shooter.runMotorSpeed(.3));
    controls.fireLowPID().whileTrue(shooter.holdMotorSetpoint(edu.wpi.first.units.Units.RPM.of(1100)));
    controls.fireHighPID().whileTrue(shooter.holdMotorSetpoint(edu.wpi.first.units.Units.RPM.of(5000)));
    // controller.x().whileTrue(shooter.runMotorSpeed(.6));
    // controller.y().whileTrue(shooter.runMotorSpeed(1));

    // controller.y().whileTrue(intake.runIntakeMotor(()->.3));

    // controller.a().whileTrue(turret.runTurretMotor(()->.3));
    
    // indexer.setDefaultCommand(indexer.runIndexMotor(()->-controller.getLeftTriggerAxis()*.3 + controller.getRightTriggerAxis()*.3));
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
 