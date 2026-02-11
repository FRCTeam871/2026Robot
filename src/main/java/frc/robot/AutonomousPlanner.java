// package frc;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.path.GoalEndState;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
// import com.pathplanner.lib.path.Waypoint;
// import com.pathplanner.lib.util.FlippingUtil;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Transform2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.units.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Constants;
// import frc.robot.subsystems.fieldtracking.FieldTracking;
// import frc.robot.subsystems.intake.Intake;
// import frc.robot.subsystems.swervedrive.SwerveDrive;
// import java.util.ArrayList;
// import java.util.List;
// import org.littletonrobotics.junction.Logger;

// public class AutonomousPlanner {

//     public enum FieldPosition {
//         None,

//         StartLeft,
//         StartMiddle,
//         StartRight,

//         ReefSide1,
//         ReefSide2,
//         ReefSide3,
//         ReefSide4,
//         ReefSide5,
//         ReefSide6,

//         CoralStationLeft,
//         CoralStationRight;

       
//         private Pose2d flipPose2d(Pose2d pose) {
//             if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
//                 return FlippingUtil.flipFieldPose(pose);
//             } else {
//                 return pose;
//             }
//         }

//     }

//     public enum Action {
//         None,

//         Shoot,

//         OutpostCollect,
//         groundCollect;

//        //TODO: make more actions
//         public boolean canDoAction(FieldPosition position) {
//             if (this == None) {
//                 return true;
//             }
           
//             if (position == FieldPosition.ReefSide1
//                     || position == FieldPosition.ReefSide2
//                     || position == FieldPosition.ReefSide3
//                     || position == FieldPosition.ReefSide4
//                     || position == FieldPosition.ReefSide5
//                     || position == FieldPosition.ReefSide6) {
//                 return true;
//             }
//             return false;
//         }
//     }

//     private final Sequencing sequencing;
//     private final Manipulator manipulator;
//     private final SwerveDrive swerveDrive;
//     private final FieldTracking fieldTracking;

//     private final ArrayList<SendableChooser<FieldPosition>> positions = new ArrayList<>();
//     private final ArrayList<SendableChooser<Action>> actions = new ArrayList<>();
//     private final SendableChooser<FieldPosition> start;

//     private Command command;

//     private Field2d field;

//     public AutonomousPlanner(
//             Elevator elevator,
//             Intake intake,
//             Manipulator manipulator,
//             SwerveDrive swerveDrive,
//             Sequencing sequencing,
//             FieldTracking fieldTracking) {
//         this.sequencing = sequencing;
//         this.manipulator = manipulator;
//         this.swerveDrive = swerveDrive;
//         this.fieldTracking = fieldTracking;

//         this.field = new Field2d();

//         this.start = positionDropdown();

//         // goes for how many auton stages there are

//         SmartDashboard.putData("Check auton", checkCommand());
//         SmartDashboard.putData("Auton/Field", this.field);
//         SmartDashboard.putData("Auton/StartPosition", this.start);

//         for (int i = 0; i < Constants.AUTON_STAGES; i++) {
//             final SendableChooser<FieldPosition> position = positionDropdown();
//             positions.add(position);
//             SmartDashboard.putData("Auton/Position" + i, position);
//             final SendableChooser<Action> action = actionDropdown();
//             actions.add(action);
//             SmartDashboard.putData("Auton/Action" + i, action);

//             SmartDashboard.putBoolean("Auton/Indicator" + i, false);
//         }
//     }

//     private SendableChooser<FieldPosition> positionDropdown() {
//         SendableChooser<FieldPosition> chooser = new SendableChooser<>();

//         for (FieldPosition pos : FieldPosition.values()) {
//             chooser.addOption(pos.toString(), pos);
//         }

//         chooser.setDefaultOption("None", FieldPosition.None);
//         return chooser;
//     }

//     private SendableChooser<Action> actionDropdown() {
//         SendableChooser<Action> choose = new SendableChooser<>();
//         for (Action action : Action.values()) {
//             choose.addOption(action.toString(), action);
//         }

//         choose.setDefaultOption("None", Action.None);
//         return choose;
//     }

//     public PathPlannerPath loadPath(String pathFileName) {
//         try {
//             return PathPlannerPath.fromPathFile(pathFileName);
//         } catch (Exception e) {
//             e.printStackTrace();
//             return null;
//         }
//     }

//     private Command generateCommand() {
//         final SequentialCommandGroup scg = new SequentialCommandGroup();
//         FieldPosition currentPosition = start.getSelected();
//         for (int i = 0; i < Constants.AUTON_STAGES; i++) {
//             field.getObject("stage" + i).setPoses(new Pose2d[0]);
//         }
//         for (int i = 0; i < Constants.AUTON_STAGES; i++) {
//             boolean continueLoop = generateStage(
//                     scg,
//                     currentPosition,
//                     positions.get(i).getSelected(),
//                     actions.get(i).getSelected(),
//                     i);
//             if (!continueLoop) {
//                 scg.addCommands(swerveDrive.manualDrive(() -> 0, () -> 0, () -> 0));
//                 break;
//             }
//             currentPosition = positions.get(i).getSelected();
//         }
//         return scg;
//     }

//     private boolean generateStage(
//             final SequentialCommandGroup scg,
//             final FieldPosition start,
//             final FieldPosition end,
//             Action action,
//             int i) {
//         PathPlannerPath path = findPath(start, end);

//         if (action.canDoAction(end) && path != null) {
//             SmartDashboard.putBoolean("Auton/Indicator" + i, true);
//         } else {
//             SmartDashboard.putBoolean("Auton/Indicator" + i, false);
//         }

//         if (path == null) {
//             field.getObject("stage" + i).setPoses(new Pose2d[0]);
//             return false;
//         }

//         final Pose2d[] points =
//                 path
//                         .generateTrajectory(new ChassisSpeeds(), Rotation2d.kZero, swerveDrive.getConfig())
//                         .getStates()
//                         .stream()
//                         .map(trajectory -> trajectory.pose)
//                         .toArray(length -> new Pose2d[length]);

//         field.getObject("stage" + i).setPoses(points);

//         scg.addCommands(Commands.runOnce(() -> {
//             Logger.recordOutput("Sequencing/Path", points);
//         }));
//         scg.addCommands(AutoBuilder.followPath(path));
//         if (action.canDoAction(end)) {
//             switch (action) {
//                 case Intake:
//                     // make the elevator dowm if its not down and then mantain its pose up againist the coral station
//                     // then stop when sensor sees the coral

//                     Pose2d pose = null;
//                     if (end == FieldPosition.CoralStationLeft) {
//                         pose = new Pose2d(1.115, 7.147, Rotation2d.fromDegrees(-54));
//                     } else if (end == FieldPosition.CoralStationRight) {
//                         pose = new Pose2d(1.057, 0.961, Rotation2d.fromDegrees(54));
//                     }
//                     if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
//                         pose = FlippingUtil.flipFieldPose(pose);
//                     }

//                     scg.addCommands(fieldTracking.maintainPose(pose).until(manipulator::hasCoral));
//                     break;
//                 case LeftScoreCoralL1:
//                     scg.addCommands(sequencing.scoreCoralNoPath(end.asReefSide(), LeftOrRight.Left, ReefLevel.L1));
//                     break;
//                 case LeftScoreCoralL3:
//                     scg.addCommands(sequencing.scoreCoralNoPath(end.asReefSide(), LeftOrRight.Left, ReefLevel.L3));
//                     break;
//                 case LeftScoreCoralL4:
//                     scg.addCommands(sequencing.scoreCoralNoPath(end.asReefSide(), LeftOrRight.Left, ReefLevel.L4));
//                     break;
//                 case LeftScorecoralL2:
//                     scg.addCommands(sequencing.scoreCoralNoPath(end.asReefSide(), LeftOrRight.Left, ReefLevel.L2));
//                     break;
//                 case None:
//                     // nothing
//                     break;
//                 case RightScoreCoralL1:
//                     scg.addCommands(sequencing.scoreCoralNoPath(end.asReefSide(), LeftOrRight.Right, ReefLevel.L1));
//                     break;
//                 case RightScoreCoralL3:
//                     scg.addCommands(sequencing.scoreCoralNoPath(end.asReefSide(), LeftOrRight.Right, ReefLevel.L3));
//                     break;
//                 case RightScoreCoralL4:
//                     scg.addCommands(sequencing.scoreCoralNoPath(end.asReefSide(), LeftOrRight.Right, ReefLevel.L4));
//                     break;
//                 case RightScorecoralL2:
//                     scg.addCommands(sequencing.scoreCoralNoPath(end.asReefSide(), LeftOrRight.Right, ReefLevel.L2));
//                     break;
//                 default:
//                     break;
//             }
//         }
//         return true;
//     }

//     private PathPlannerPath findPath(FieldPosition start, FieldPosition end) {
//         String pathFileName = start + "-" + end;
//         PathPlannerPath path = loadPath(pathFileName.toLowerCase());
//         if (start.asPose2d(sequencing) == null || end.asPose2d(sequencing) == null) {
//             return null;
//         }
//         if (path == null) {

//             final List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
//                     start.asPose2d(sequencing)
//                             .transformBy(new Transform2d(
//                                     0, 0, start.flipApproachStart() ? Rotation2d.k180deg : Rotation2d.kZero)),
//                     end.asPose2d(sequencing)
//                             .transformBy(new Transform2d(
//                                     0, 0, end.flipApproachEnd() ? Rotation2d.k180deg : Rotation2d.kZero)));

//             // if (DriverStation.getAlliance().equals(Optional.of(Alliance.Red)));
//             // waypoints.set(1, waypoints.get(1).flip());

//             final PathConstraints constraints = new PathConstraints(
//                     0.5 * Constants.DRIVE_SPEED_MULTIPLIER, 0.2, 2 * Math.PI, 4 * Math.PI); // The constraints
//             // for this path.

//             path = new PathPlannerPath(
//                     waypoints,
//                     List.of(),
//                     List.of(),
//                     List.of(),
//                     // List.of(new ConstraintsZone(0.8, 1.0, new
//                     // PathConstraints(0.5*Constants.DRIVE_SPEED_MULTIPLIER,
//                     // 0.5, 2 * Math.PI, 4 * Math.PI))),
//                     List.of(),
//                     constraints,
//                     null, // The ideal starting state, this is only relevant for pre-planned paths, so can
//                     // be null for on-the-fly paths.
//                     new GoalEndState(
//                             0.0,
//                             end.asPose2d(sequencing).getRotation()), // Goal end state. You can set a holonomic rotation
//                     // here. If using a differential drivetrain, the
//                     // rotation will have no effect.
//                     false);
//             path.preventFlipping = true;
//         } else {
//             path.preventFlipping = true;
//             if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
//                 path = path.flipPath();
//             }
//         }

//         System.out.println(pathFileName + " " + path);

//         return path;
//     }

//     private Command checkCommand() {
//         return Commands.runOnce(() -> {
//                     command = generateCommand();
//                 })
//                 .ignoringDisable(true)
//                 .withName("Check");
//     }

//     public Command getAutonCommand() {
//         if (command == null) {
//             command = generateCommand();
//         }
//         return command;
//     }

//     public void periodic() {
//         field.setRobotPose(swerveDrive.getEstimatedPose());
//     }
// }
