package frc.robot;


import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.units.measure.Angle;
import static org.wpilib.units.Units.InchesPerSecond;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.coralWrist.CoralWristSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.gyro.Gyro;

public class CommandFactory {

  private final DriveSubsystem drive;
  private final Gyro gyro;
  private final ElevatorSubsystem elevator;
  private final CoralWristSubsystem coralWrist;
  private final AlgaeWristSubsystem algaeWrist;
  private final AlgaeIntakeSubsystem algaeIntake;
  private final CoralIntakeSubsystem coralIntake;
  private final ClimberSubsystem climber;

  // private final NetworkTableEntry ntEntry; //one for each entry we want to read
  // (state changes)
  public static final NetworkTable scoringStateTables = NetworkTableInstance.getDefault().getTable("sidecarTable");
  // private final NetworkTableEntry newEntry;
  private final NetworkTableEntry levelEntry = scoringStateTables.getEntry("scoringLevel");
  private final NetworkTableEntry gameModeEntry = scoringStateTables.getEntry("gamePieceMode");
  private final NetworkTableEntry leftRightEntry = scoringStateTables.getEntry("Position");
  private final NetworkTableEntry endgameEntry = scoringStateTables.getEntry("endgame");

  public CommandFactory(DriveSubsystem drive, Gyro gyro, ElevatorSubsystem elevator, CoralWristSubsystem coralWrist,
      AlgaeWristSubsystem algaeWrist, AlgaeIntakeSubsystem algaeIntake, CoralIntakeSubsystem coralIntake, ClimberSubsystem climber) {
    this.drive = drive;
    this.elevator = elevator;
    this.coralWrist = coralWrist;
    this.algaeWrist = algaeWrist;
    this.algaeIntake = algaeIntake;
    this.coralIntake = coralIntake;
    this.gyro = gyro;
    this.climber = climber;
    setGameMode("coral");
    setScoringLevel("Level 1");
    setRobotAlignmentPosition("left");
    setEndgame(false);
    // ntEntry = scoringStateTables.getEntry("GameMode"); //one for each key
    // newEntry = scoringStateTables.getEntry("Indicator");
  }

  public enum RobotPosition {
    LEFT,
    RIGHT,
  }

  public enum Setpoint {
    L_ONE,
    L_TWO,
    L_THREE,
    L_FOUR,
    TURTLE,
    ZERO,
    AUTON
  }

  public enum GameMode {
    CORAL,
    ALGAE
  }

  public RobotPosition robotPosition;
  public GameMode gameMode;
  public Setpoint setpoint;

  public Setpoint getScoringLevel() {
    Setpoint scoringLevel;
    if (levelEntry.getString("None").equals("Level 1")) {
      scoringLevel = Setpoint.L_ONE;
    } else if (levelEntry.getString("None").equals("Level 2")) {
      scoringLevel = Setpoint.L_TWO;
    } else if (levelEntry.getString("None").equals("Level 3")) {
      scoringLevel = Setpoint.L_THREE;
    } else if (levelEntry.getString("None").equals("Level 4")) {
      scoringLevel = Setpoint.L_FOUR;
    } else {
      scoringLevel = Setpoint.L_ONE;
    }
    return scoringLevel;
  }

  public GameMode getGameMode() {
    if (gameModeEntry.getString("None").equals("coral")) {
      gameMode = GameMode.CORAL;
    } else if (gameModeEntry.getString("None").equals("algae")) {
      gameMode = GameMode.ALGAE;
    }
    SmartDashboard.putString("networktablesData/gameMode", gameMode.toString());
    return gameMode;
  }

  // sadly, the belt on the algae interferes with our camera visibility :( so this setpoint
  // allows the elevator to be slightly up for auton. it will only be called
  // directly by autonomous methods and SHOULD NOT BE USED in teleop
  public Command autonDefaultPosition() {
    return Commands.sequence(
        Commands.parallel(
            algaeWrist.goToSetpointCommand(() -> Setpoint.ZERO),
            elevator.goToGoalSetpointCmd(() -> Setpoint.AUTON, () -> GameMode.CORAL)),
        Commands.waitUntil(() -> elevator.atGoal()),
        coralWrist.goToSetpointCommand(() -> Setpoint.TURTLE));
  }

  public Command autonTurtleMode() {
    return Commands.sequence(
        coralWrist.goToSetpointCommand(() -> Setpoint.ZERO),
        Commands.waitUntil(() -> coralWrist.atGoal()),
        Commands.parallel(
            algaeWrist.goToSetpointCommand(() -> Setpoint.ZERO),
            elevator.goToGoalSetpointCmd(() -> Setpoint.AUTON, () -> GameMode.CORAL)),
        Commands.waitUntil(() -> elevator.atGoal()),
        coralWrist.goToSetpointCommand(() -> Setpoint.TURTLE));
  }

  public Command turtleBasedOnMode() {
    return Commands.either(
        algaeTurtleMode(),
        coralTurtleMode(),
        () -> (getGameMode() == GameMode.ALGAE));
  }

  public Command coralTurtleMode() {
    return Commands.sequence(
        coralWrist.goToSetpointCommand(() -> Setpoint.ZERO),
        Commands.waitUntil(() -> coralWrist.atGoal()),
        Commands.parallel(
            algaeWrist.goToSetpointCommand(() -> Setpoint.ZERO),
            elevator.goToGoalSetpointCmd(() -> Setpoint.TURTLE, () -> GameMode.CORAL)),
        Commands.waitUntil(() -> elevator.atGoal()),
        coralWrist.goToSetpointCommand(() -> Setpoint.TURTLE));
  }

  public Command algaeTurtleMode() {
    return Commands.sequence(
        coralWrist.goToSetpointCommand(() -> Setpoint.ZERO),
        Commands.waitUntil(() -> coralWrist.atGoal()),
        Commands.parallel(
            elevator.goToGoalSetpointCmd(() -> Setpoint.TURTLE, () -> GameMode.ALGAE),
            algaeWrist.goToSetpointCommand(() -> Setpoint.TURTLE)),
        Commands.waitUntil(() -> elevator.atGoal()),
        coralWrist.goToSetpointCommand(() -> Setpoint.TURTLE));
  }

  public Command elevatorBasedOnMode() {
    return Commands.either(
        moveElevatorAndAlgaeWrist(),
        moveElevatorAndCoralWrist(),
        () -> (getGameMode() == GameMode.ALGAE));
  }

  public Command moveElevatorAndCoralWrist() {
    return Commands.sequence(
        Commands.parallel(
            algaeWrist.goToSetpointCommand(() -> Setpoint.ZERO),
            coralWrist.goToSetpointCommand(() -> Setpoint.ZERO)),
        Commands.waitUntil(() -> coralWrist.atGoal()),
        Commands.parallel(
            elevator.goToGoalSetpointCmd(() -> getSetpoint(), () -> GameMode.CORAL),
            coralWrist.goToSetpointCommand(() -> getSetpoint())));
  }

  public Command moveElevatorAndAlgaeWrist() {
    return Commands.sequence(
        coralWrist.goToSetpointCommand(() -> Setpoint.ZERO),
        Commands.waitUntil(() -> coralWrist.atGoal()),
        algaeWrist.goToSetpointCommand(() -> getSetpoint()),
        elevator.goToGoalSetpointCmd(() -> getSetpoint(), () -> GameMode.ALGAE));
  }

  public Command intakeBasedOnMode() {
    return Commands.either(
        algaeIntake.intakeToStall(),
        coralIntake.intakeToStall(),
        () -> (getGameMode() == GameMode.ALGAE));
  }

  public Command outtakeBasedOnMode() {
    return Commands.either(
        algaeIntake.outtake(),
        coralIntake.setOuttakeSpeed(() -> getSetpoint()),
        () -> (getGameMode() == GameMode.ALGAE));
  }

  public Command climbIn() {
    return climber.setSpeed(InchesPerSecond.of(-3.5));
  }

  public Command climbOut() {
    return climber.setSpeed(InchesPerSecond.of(5));
  }

  public Command intakeOrClimbOutBasedOnMode() {
    return Commands.either(
      climbOut(),
      intakeBasedOnMode(),
      () -> (getEndgameMode())
    );
  }

  public Command outtakeOrClimbInBasedOnMode() {
    return Commands.either(
      climbIn(),
      outtakeBasedOnMode(),
      () -> (getEndgameMode())
    );
  }

  public Setpoint getSetpoint() {
    Setpoint setpoint;
    if (levelEntry.getString("None").equals("Level 1")) {
      setpoint = Setpoint.L_ONE;
    } else if (levelEntry.getString("None").equals("Level 2")) {
      setpoint = Setpoint.L_TWO;
    } else if (levelEntry.getString("None").equals("Level 3")) {
      setpoint = Setpoint.L_THREE;
    } else if (levelEntry.getString("None").equals("Level 4")) {
      setpoint = Setpoint.L_FOUR;
    } else {
      setpoint = Setpoint.L_ONE;
    }
    SmartDashboard.putString("networktablesData/setpoint", setpoint.toString());
    return setpoint;
  };

  public RobotPosition getRobotPosition() {
    if (leftRightEntry.getString("None").equals("left")) {
      robotPosition = RobotPosition.LEFT;
    } else if (leftRightEntry.getString("None").equals("right")) {
      robotPosition = RobotPosition.RIGHT;
    }
    SmartDashboard.putString("networktablesData/robotPosition", robotPosition.toString());
    return robotPosition;
  }

  public boolean getEndgameMode() {
    boolean endgame = endgameEntry.getBoolean(false);
    SmartDashboard.putBoolean("networktablesData/endgameMode", endgame);
    return endgame;
  }

  public void setGameMode(String level) {
    gameModeEntry.setString(level);
  }

  public void setScoringLevel(String level) {
    levelEntry.setString(level);
  }

  public void setRobotAlignmentPosition(String alignmentValue) {
    leftRightEntry.setString(alignmentValue);
  }

  public void setEndgame(Boolean endgame) {
    endgameEntry.setBoolean(endgame);
  }

  public Command resetHeading(Angle yaw) {
    return Commands.parallel(gyro.setYaw(yaw), Commands.runOnce(() -> drive.resetOdometryAfterGyro()));
  }
}