package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Aiming;
import frc.robot.subsystems.fieldtracking.FieldTracking;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.swervedrive.SwerveDrive;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AutonomousPlanner {

    public enum FieldPosition {
        None;
        private Pose2d flipPose2d(Pose2d pose) {
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                return FlippingUtil.flipFieldPose(pose);
            } else {
                return pose;
            }
        }
    }

    public enum Action {
        None,

        Shoot,

        OutpostCollect,
        GroundCollect;

       //TODO: make more actions
        public boolean canDoAction(FieldPosition position) {
            if (this == None) {
                return true;
            }
            return false;
        }
    }

    private final SwerveDrive swerveDrive;
    private final Intake intake;
    private final FieldTracking fieldTracking;
    private final Aiming aiming;

    private final ArrayList<SendableChooser<FieldPosition>> positions = new ArrayList<>();
    private final ArrayList<SendableChooser<Action>> actions = new ArrayList<>();
    private final SendableChooser<FieldPosition> start;

    private Command command;

    private Field2d field;

    public AutonomousPlanner(Intake intake, SwerveDrive swerveDrive, FieldTracking fieldTracking, Aiming aiming) {
        this.swerveDrive = swerveDrive;
        this.intake = intake;
        this.fieldTracking = fieldTracking;
        this.aiming = aiming;

        this.field = new Field2d();

        this.start = positionDropdown();




        // goes for how many auton stages there are




        SmartDashboard.putData("Check auton", checkCommand());
        SmartDashboard.putData("Auton/Field", this.field);
        SmartDashboard.putData("Auton/StartPosition", this.start);

        for (int i = 0; i < Constants.AUTON_STAGES; i++) {
            final SendableChooser<FieldPosition> position = positionDropdown();
            positions.add(position);
            SmartDashboard.putData("Auton/Position" + i, position);
            final SendableChooser<Action> action = actionDropdown();
            actions.add(action);
            SmartDashboard.putData("Auton/Action" + i, action);

            SmartDashboard.putBoolean("Auton/Indicator" + i, false);
        }
    }

    private SendableChooser<FieldPosition> positionDropdown() {
        SendableChooser<FieldPosition> chooser = new SendableChooser<>();

        for (FieldPosition pos : FieldPosition.values()) {
            chooser.addOption(pos.toString(), pos);
        }

        chooser.setDefaultOption("None", FieldPosition.None);
        return chooser;
    }

    private SendableChooser<Action> actionDropdown() {
        SendableChooser<Action> choose = new SendableChooser<>();
        for (Action action : Action.values()) {
            choose.addOption(action.toString(), action);
        }

        choose.setDefaultOption("None", Action.None);
        return choose;
    }

    public PathPlannerPath loadPath(String pathFileName) {
        try {
            return PathPlannerPath.fromPathFile(pathFileName);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }

    private Command generateCommand() {
        final SequentialCommandGroup scg = new SequentialCommandGroup();
        FieldPosition currentPosition = start.getSelected();
        for (int i = 0; i < Constants.AUTON_STAGES; i++) {
            field.getObject("stage" + i).setPoses(new Pose2d[0]);
        }
        for (int i = 0; i < Constants.AUTON_STAGES; i++) {
            boolean continueLoop = generateStage(
                    scg,
                    currentPosition,
                    positions.get(i).getSelected(),
                    actions.get(i).getSelected(),
                    i);
            if (!continueLoop) {
                scg.addCommands(swerveDrive.manualDrive(() -> 0, () -> 0, () -> 0));
                break;
            }
            currentPosition = positions.get(i).getSelected();
        }
        return scg;
    }

    private boolean generateStage(final SequentialCommandGroup scg, final FieldPosition start, final FieldPosition end, Action action, int i) {
        PathPlannerPath path = findPath(start, end);

        if (action.canDoAction(end) && path != null) {
            SmartDashboard.putBoolean("Auton/Indicator" + i, true);
        } else {
            SmartDashboard.putBoolean("Auton/Indicator" + i, false);
        }

        if (path == null) {
            field.getObject("stage" + i).setPoses(new Pose2d[0]);
            return false;
        }

        final Pose2d[] points =
                path
                        .generateTrajectory(new ChassisSpeeds(), Rotation2d.kZero, swerveDrive.getConfig())
                        .getStates()
                        .stream()
                        .map(trajectory -> trajectory.pose)
                        .toArray(length -> new Pose2d[length]);

        field.getObject("stage" + i).setPoses(points);

        scg.addCommands(Commands.runOnce(() -> {
            Logger.recordOutput("Sequencing/Path", points);
        }));
        scg.addCommands(AutoBuilder.followPath(path));
        if (action.canDoAction(end)) {
            switch (action) {
                case None:
                    // nothing
                    break;
                case Shoot:
                    scg.addCommands(aiming.shootTrue().withTimeout(Constants.FIRERATE.asPeriod().times(32)));
                    break;
                case OutpostCollect:
                    scg.addCommands(intake.runIntakeMotor(() -> Constants.ocIntakeMotorSpeed));
                    break;
                case GroundCollect:
                    scg.addCommands(intake.runIntakeMotor(() -> (Constants.ocIntakeMotorSpeed / 2)).withTimeout(.5));
                    break;
                default:
                    break;
            }
        }
        return true;
    }

    private PathPlannerPath findPath(FieldPosition start, FieldPosition end) {
        String pathFileName = start + "-" + end;
        PathPlannerPath path = loadPath(pathFileName.toLowerCase());
        if (start == FieldPosition.None || end == FieldPosition.None) {
            return null;
        }
        if (path != null) {
            path.preventFlipping = true;
            if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
                path = path.flipPath();
            }
        }

        System.out.println(pathFileName + " " + path);

        return path;
    }

    private Command checkCommand() {
        return Commands.runOnce(() -> {
                    command = generateCommand();
                })
                .ignoringDisable(true)
                .withName("Check");
    }

    public Command getAutonCommand() {
        if (command == null) {
            command = generateCommand();
        }
        return command;
    }

    public void periodic() {
        field.setRobotPose(swerveDrive.getEstimatedPose());
    }
}
