package frc.robot.subsystems.swervedrive;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;

public class SwerveDriveIORoll implements SwerveDriveIO {
    private final AHRS gyro;
    private double gyroZero;

    public SwerveDriveIORoll(final AHRS gyro) {
        SmartDashboard.putData("resetGyro", Commands.runOnce(this::zeroRoll).ignoringDisable(true));
        SmartDashboard.putData(
                "resetGyro180", Commands.runOnce(() -> setCurrentAngle(180.0)).ignoringDisable(true));
        this.gyro = gyro;
        zeroRoll();
    }

    @Override
    public void setCurrentAngle(double angle) {
        gyroZero = -(gyro.getYaw()) - angle;
    }

    @Override
    public void updateInputs(final SwerveDriveIOInputs inputs) {
        Logger.recordOutput("Drive/gyroZero", gyroZero);
        inputs.gyroRotation = Rotation2d.fromDegrees(-(gyro.getYaw()) - gyroZero);
        inputs.isCalibrating = gyro.isCalibrating();
        inputs.gyroRate = gyro.getRate();
    }

    public void zeroRoll() {
        gyroZero = -(gyro.getYaw());
    }
}
