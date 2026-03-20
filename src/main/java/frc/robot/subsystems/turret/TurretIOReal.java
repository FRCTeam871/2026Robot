package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

public class TurretIOReal implements TurretIO {
    private static final int FORWARD_LIMIT = 90;
    private static final int REVERSE_LIMIT = -68;
    private final SparkMax turretMotor;
    private final SparkMaxConfig config;
    private final SparkAnalogSensor m_Turret_Encoder;
    private SparkClosedLoopController m_TurretMotorController;
    private final double turretZero = 183.970;
    private REVLibError lastErr;

    public TurretIOReal() {
        this.turretMotor = new SparkMax(44, MotorType.kBrushless);
        m_Turret_Encoder = turretMotor.getAnalog();
        this.m_TurretMotorController = turretMotor.getClosedLoopController();

        this.config = new SparkMaxConfig();
        updatePIDConstants(0.02, 0, 0.0, 0, 0, 0, 60 * 60, 180 * 60, 720);
        SmartDashboard.putData(applyPIDConstants());
        config.apply(new SoftLimitConfig().forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true)
                .reverseSoftLimit(REVERSE_LIMIT + turretZero).forwardSoftLimit(FORWARD_LIMIT + turretZero)); // checks if the angle of the turret is what we set it in
        config.apply(config.analogSensor.positionConversionFactor(Constants.TURRETCONVERSIONFACTOR)
                .velocityConversionFactor(Constants.TURRETCONVERSIONFACTOR));
        config.apply(config.inverted(true));
        turretMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        Logger.recordOutput("Turret/reverseSoftLimit", REVERSE_LIMIT + turretZero);
        Logger.recordOutput("Turret/forwardSoftLimit", FORWARD_LIMIT + turretZero);
    }

    public void updatePIDConstants(double kP, double kI, double kD, double kS, double kV, double kA,
            double maxAcceleration, double cruiseVelocity, double allowedError) {
        config.closedLoop
                .p(kP, ClosedLoopSlot.kSlot1)
                .i(kI, ClosedLoopSlot.kSlot1)
                .d(kD, ClosedLoopSlot.kSlot1)
                .outputRange(0, 0).feedbackSensor(FeedbackSensor.kAnalogSensor);
        config.closedLoop.feedForward.kS(kS, ClosedLoopSlot.kSlot1).kV(kV, ClosedLoopSlot.kSlot1).kA(kA,
                ClosedLoopSlot.kSlot1);
        config.closedLoop.maxMotion.maxAcceleration(maxAcceleration, ClosedLoopSlot.kSlot1)
                .cruiseVelocity(cruiseVelocity, ClosedLoopSlot.kSlot1)
                .allowedProfileError(allowedError, ClosedLoopSlot.kSlot1);
        turretMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.putNumber("Turret/PID/kP", kP);
        SmartDashboard.putNumber("Turret/PID/kI", kI);
        SmartDashboard.putNumber("Turret/PID/kD", kD);
        SmartDashboard.putNumber("Turret/PID/kS", kS);
        SmartDashboard.putNumber("Turret/PID/kV", kV);
        SmartDashboard.putNumber("Turret/PID/kA", kA);
        SmartDashboard.putNumber("Turret/PID/cruiseVelocity", cruiseVelocity);
        SmartDashboard.putNumber("Turret/PID/allowedError", allowedError);
        SmartDashboard.putNumber("Turret/PID/maxAccerlation", maxAcceleration);
        System.out.printf("%f %f %f %f %f %f %f %f %f\n", kP, kI, kD, kS, kV, kA, maxAcceleration, allowedError,
                cruiseVelocity);
    }

    public Command applyPIDConstants() {
        return Commands.runOnce(() -> {
            double kP = SmartDashboard.getNumber("Turret/PID/kP", 0);
            double kI = SmartDashboard.getNumber("Turret/PID/kI", 0);
            double kD = SmartDashboard.getNumber("Turret/PID/kD", 0);
            double kS = SmartDashboard.getNumber("Turret/PID/kS", 0);
            double kV = SmartDashboard.getNumber("Turret/PID/kV", 0);
            double kA = SmartDashboard.getNumber("Turret/PID/kA", 0);
            double cruiseVelocity = SmartDashboard.getNumber("Turret/PID/cruiseVelocity", 0);
            double allowedError = SmartDashboard.getNumber("Turret/PID/allowedError", 0);
            double maxAcceleration = SmartDashboard.getNumber("Turret/PID/maxAccerlation", 0);
            updatePIDConstants(kP, kI, kD, kS, kV, kA, maxAcceleration, cruiseVelocity, allowedError);
        }).withName("apply1").ignoringDisable(true);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) { // function that updates the input values located in inputs
        inputs.setpointAngle = Units.Degree.of(m_TurretMotorController.getSetpoint() - turretZero);
        inputs.turretAngle = Units.Degree.of(m_Turret_Encoder.getPosition() - turretZero);
        inputs.softForwardLimit = turretMotor.getForwardSoftLimit().isReached();
        inputs.softReverseLimit = turretMotor.getReverseSoftLimit().isReached();
        inputs.isConnected = m_Turret_Encoder.getPosition() != 0;
        Logger.recordOutput("Turret/Angle", inputs.turretAngle);
        Logger.recordOutput("Turret/rawTurretAngle", m_Turret_Encoder.getPosition());
        Logger.recordOutput("Turret/SoftFowardLimitReached", turretMotor.getForwardSoftLimit().isReached());
        Logger.recordOutput("Turret/SoftReverseLimitReached", turretMotor.getReverseSoftLimit().isReached());
        // Logger.recordOutput("Turret/turretSpeed", m_Turret_Encoder.getVelocity());
    }

    @Override
    public void setTarget(Angle inputAngle) {
        Angle result = Units.Radians.of(MathUtil.angleModulus(inputAngle.in(Radians)));
        if(result.gte( Units.Degrees.of(FORWARD_LIMIT)) || result.lte(Units.Degrees.of(REVERSE_LIMIT))){
            return;

        }
        result = Units.Degrees.of(MathUtil.clamp(result.in(Degrees), REVERSE_LIMIT, FORWARD_LIMIT));
        result = result.plus(Units.Degrees.of(turretZero));
        Logger.recordOutput("Turret/rawSetpoint", result.in(Degrees));
        final REVLibError err = m_TurretMotorController.setSetpoint(result.in(Degrees), ControlType.kPosition,
                ClosedLoopSlot.kSlot1);
        if (err != lastErr) {
            System.out.println("Status Change: " + err);
            this.lastErr = err;
        }
    }

    @Override
    public void runDumn(double speed) {
        turretMotor.set(speed);
    }
}
