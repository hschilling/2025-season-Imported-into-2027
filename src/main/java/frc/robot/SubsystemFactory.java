package frc.robot;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;

import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;
import frc.robot.Constants.MotorIdConstants;

import frc.robot.subsystems.algaeIntake.AlgaeIntakeSubsystem;
import frc.robot.subsystems.algaeWrist.AlgaeWristHardware;
import frc.robot.subsystems.algaeWrist.AlgaeWristPlacebo;
import frc.robot.subsystems.algaeWrist.AlgaeWristSubsystem;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeBetaHardware;
import frc.robot.subsystems.algaeIntake.AlgaeIntakeCompHardware;
import frc.robot.subsystems.algaeIntake.AlgaeIntakePlacebo;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.ClimberHardware;
import frc.robot.subsystems.climber.ClimberPlacebo;
import frc.robot.subsystems.coralIntake.CoralIntakeAlphaHardware;
import frc.robot.subsystems.coralIntake.CoralIntakeBetaHardware;
import frc.robot.subsystems.coralIntake.CoralIntakePlacebo;
import frc.robot.subsystems.coralIntake.CoralIntakeSubsystem;
import frc.robot.subsystems.coralWrist.CoralWristHardware;
import frc.robot.subsystems.coralWrist.CoralWristPlacebo;
import frc.robot.subsystems.coralWrist.CoralWristSubsystem;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.drive.SwerveModuleHardwareNEO;
import frc.robot.subsystems.drive.SwerveModuleHardwareVortex;
import frc.robot.subsystems.drive.SwerveModulePlacebo;
import frc.robot.subsystems.elevator.AlphaElevatorHardware;
import frc.robot.subsystems.elevator.ElevatorPlacebo;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.elevator.KrakenElevatorHardware;
import frc.robot.subsystems.gyro.Gyro;
import frc.robot.subsystems.gyro.GyroHardware;
import frc.robot.subsystems.gyro.GyroPlacebo;

public class SubsystemFactory {
    private static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
    private static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    private static final double REAR_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
    private static final double REAR_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

    // 64:16 (4:1) gear ratio (through bore encoder on shaft)
    private static final double BETA_CORAL_ABSOLUTE_ENCODER_WRIST_POSITION_FACTOR = (2 * Math.PI) / 3.0; // radians
    // divide position factor by 60 for radians per second
    private static final double BETA_CORAL_ABSOLUTE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 180.0; // radians per
                                                                                                     // second
    private static final boolean BETA_CORAL_WRIST_SOFT_LIMIT = true;
    private static final Time BETA_CORAL_DEBOUNCER_TIME = Seconds.of(0.25); 
    private static final Current BETA_CORAL_INTAKE_STALL_THRESHOLD = Amps.of(25.0);
    
    private static final Time COMP_CORAL_DEBOUNCER_TIME = Seconds.of(0.45); 
    private static final Current COMP_CORAL_INTAKE_STALL_THRESHOLD = Amps.of(38.0);

    // 64:16 (4:1) gear ratio (through bore encoder on shaft)
    private static final double ALPHA_CORAL_ABSOLUTE_ENCODER_WRIST_POSITION_FACTOR = (2 * Math.PI) / 4.0; // radians
    // divide position factor by 60 for radians per second
    private static final double ALPHA_CORAL_ABSOLUTE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 240.0; // radians per
                                                                                                      // second
    private static final boolean ALPHA_CORAL_WRIST_SOFT_LIMIT = false;

    private static final String MOZART_SERIAL_NUMBER = "030ee8c8";
    private static final String ALPHA_SERIAL_NUMBER = "03260A64";
    private static final String BETA_SERIAL_NUMBER = "030589d5";
    private static final String COMP_SERIAL_NUMBER = "030fc267";

    private static final Distance ELEVATOR_ALPHA_MAX_HEIGHT = Inches.of(34.25);
    private static final Distance ELEVATOR_BETA_MAX_HEIGHT = Inches.of(50);

    public enum RobotType {
        MOZART,
        SIM,
        ALPHA,
        BETA,
        COMP
    }

    private RobotType robotType;

    // TODO(SystemCore): the roboRIO exposed the "serialnum" env var used to pick the robot
    // variant. Confirm SystemCore exposes the same var (and these serial values); if not,
    // update the detection. Defaulting to "" keeps a missing var falling through to SIM
    // instead of throwing a NullPointerException (e.g. in simulation).
    private String serialNum = System.getenv("serialnum") != null ? System.getenv("serialnum") : "";

    public SubsystemFactory() {
        if (serialNum.equals(ALPHA_SERIAL_NUMBER)) {
            robotType = RobotType.ALPHA;
        } else if (serialNum.equals(BETA_SERIAL_NUMBER)) {
            robotType = RobotType.BETA;
        } else if (serialNum.equals(COMP_SERIAL_NUMBER)) {
            robotType = RobotType.COMP;
        } else if (serialNum.equals(MOZART_SERIAL_NUMBER)) {
            robotType = RobotType.MOZART;
        } else {
            robotType = RobotType.SIM;
        }
    }

    public RobotType getRobotType() {
        return robotType;
    }

    public DriveSubsystem buildDriveSubsystem(Gyro gyro) {
        SwerveModule frontLeft;
        SwerveModule rearLeft;
        SwerveModule frontRight;
        SwerveModule rearRight;

        if (robotType == RobotType.ALPHA) {
            frontLeft = new SwerveModule(new SwerveModuleHardwareNEO(
                    MotorIdConstants.FRONT_LEFT_DRIVING_CAN_ID,
                    MotorIdConstants.FRONT_LEFT_TURNING_CAN_ID,
                    FRONT_LEFT_CHASSIS_ANGULAR_OFFSET, "front left"));
            frontRight = new SwerveModule(new SwerveModuleHardwareNEO(
                    MotorIdConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                    MotorIdConstants.FRONT_RIGHT_TURNING_CAN_ID,
                    FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET, "front right"));
            rearLeft = new SwerveModule(new SwerveModuleHardwareNEO(
                    MotorIdConstants.REAR_LEFT_DRIVING_CAN_ID,
                    MotorIdConstants.REAR_LEFT_TURNING_CAN_ID,
                    REAR_LEFT_CHASSIS_ANGULAR_OFFSET, "rear left"));
            rearRight = new SwerveModule(new SwerveModuleHardwareNEO(
                    MotorIdConstants.REAR_RIGHT_DRIVING_CAN_ID,
                    MotorIdConstants.REAR_RIGHT_TURNING_CAN_ID,
                    REAR_RIGHT_CHASSIS_ANGULAR_OFFSET, "rear right"));
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro,
                    Constants.DriveControlConstants.ALPHA_TRACK_WIDTH,
                    Constants.DriveControlConstants.ALPHA_TRACK_WIDTH);
        } else if (robotType == RobotType.BETA || robotType == RobotType.COMP) {
            frontLeft = new SwerveModule(new SwerveModuleHardwareVortex(
                    MotorIdConstants.FRONT_LEFT_DRIVING_CAN_ID,
                    MotorIdConstants.FRONT_LEFT_TURNING_CAN_ID,
                    FRONT_LEFT_CHASSIS_ANGULAR_OFFSET, "front left"));
            frontRight = new SwerveModule(new SwerveModuleHardwareVortex(
                    MotorIdConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                    MotorIdConstants.FRONT_RIGHT_TURNING_CAN_ID,
                    FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET, "front right"));
            rearLeft = new SwerveModule(new SwerveModuleHardwareVortex(
                    MotorIdConstants.REAR_LEFT_DRIVING_CAN_ID,
                    MotorIdConstants.REAR_LEFT_TURNING_CAN_ID,
                    REAR_LEFT_CHASSIS_ANGULAR_OFFSET, "rear left"));
            rearRight = new SwerveModule(new SwerveModuleHardwareVortex(
                    MotorIdConstants.REAR_RIGHT_DRIVING_CAN_ID,
                    MotorIdConstants.REAR_RIGHT_TURNING_CAN_ID,
                    REAR_RIGHT_CHASSIS_ANGULAR_OFFSET, "rear right"));
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro,
                    Constants.DriveControlConstants.BETA_XTRACK_WIDTH,
                    Constants.DriveControlConstants.BETA_YTRACK_WIDTH);
        } else if (robotType == RobotType.MOZART) {
            frontLeft = new SwerveModule(new SwerveModuleHardwareNEO(
                    MotorIdConstants.FRONT_LEFT_DRIVING_CAN_ID,
                    MotorIdConstants.FRONT_LEFT_TURNING_CAN_ID,
                    FRONT_LEFT_CHASSIS_ANGULAR_OFFSET, "front left"));
            frontRight = new SwerveModule(new SwerveModuleHardwareNEO(
                    MotorIdConstants.FRONT_RIGHT_DRIVING_CAN_ID,
                    MotorIdConstants.FRONT_RIGHT_TURNING_CAN_ID,
                    FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET, "front right"));
            rearLeft = new SwerveModule(new SwerveModuleHardwareNEO(
                    MotorIdConstants.REAR_LEFT_DRIVING_CAN_ID,
                    MotorIdConstants.REAR_LEFT_TURNING_CAN_ID,
                    REAR_LEFT_CHASSIS_ANGULAR_OFFSET, "rear left"));
            rearRight = new SwerveModule(new SwerveModuleHardwareNEO(
                    MotorIdConstants.REAR_RIGHT_DRIVING_CAN_ID,
                    MotorIdConstants.REAR_RIGHT_TURNING_CAN_ID,
                    REAR_RIGHT_CHASSIS_ANGULAR_OFFSET, "rear right"));
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro,
                    Constants.DriveControlConstants.MOZART_TRACK_WIDTH,
                    Constants.DriveControlConstants.MOZART_TRACK_WIDTH);
        } else {
            frontLeft = new SwerveModule(new SwerveModulePlacebo());
            frontRight = new SwerveModule(new SwerveModulePlacebo());
            rearLeft = new SwerveModule(new SwerveModulePlacebo());
            rearRight = new SwerveModule(new SwerveModulePlacebo());
            return new DriveSubsystem(frontLeft, frontRight, rearLeft, rearRight, gyro, Inches.of(10), Inches.of(10));
        }

        // 10 is a default value for sim lol
    }

    public Gyro buildGyro() {
        if (robotType == RobotType.COMP || robotType == RobotType.BETA || robotType == RobotType.ALPHA || robotType == RobotType.MOZART) {
            return new Gyro(new GyroHardware());
        } else {
            return new Gyro(new GyroPlacebo());
        }
    }

    public AlgaeIntakeSubsystem buildAlgaeIntake() {
        if (robotType == RobotType.BETA) {
            return new AlgaeIntakeSubsystem(new AlgaeIntakeBetaHardware());
        } else 
        if (robotType == RobotType.COMP) {
            return new AlgaeIntakeSubsystem(new AlgaeIntakeCompHardware());
        }else 
        {
            return new AlgaeIntakeSubsystem(new AlgaeIntakePlacebo());
        }
    }

    public AlgaeWristSubsystem buildAlgaeWrist() {
        if (robotType == RobotType.COMP) {
            return new AlgaeWristSubsystem(new AlgaeWristHardware(true, true));
        } else if (robotType == RobotType.BETA) {
            return new AlgaeWristSubsystem(new AlgaeWristHardware(false, false));
        }
        else {
            return new AlgaeWristSubsystem(new AlgaeWristPlacebo());
        }
    }

    public ClimberSubsystem buildClimber()
    {
        if (robotType == RobotType.COMP){
            return new ClimberSubsystem(new ClimberHardware());
        } else {
            return new ClimberSubsystem(new ClimberPlacebo()); 
        }
    }

    public CoralIntakeSubsystem buildCoralIntake() {
        if (robotType == RobotType.ALPHA) {
            return new CoralIntakeSubsystem(new CoralIntakeAlphaHardware());
        } else if (robotType == RobotType.BETA) {    
            return new CoralIntakeSubsystem(new CoralIntakeBetaHardware(BETA_CORAL_DEBOUNCER_TIME, BETA_CORAL_INTAKE_STALL_THRESHOLD));
        } else if (robotType == RobotType.COMP) 
        {
            return new CoralIntakeSubsystem(new CoralIntakeBetaHardware(COMP_CORAL_DEBOUNCER_TIME, COMP_CORAL_INTAKE_STALL_THRESHOLD));
        }
        else {
            return new CoralIntakeSubsystem(new CoralIntakePlacebo());
        }
    }

    public CoralWristSubsystem buildCoralWrist() {
        if (robotType == RobotType.ALPHA) {
            return new CoralWristSubsystem(new CoralWristHardware(ALPHA_CORAL_ABSOLUTE_ENCODER_WRIST_POSITION_FACTOR,
                    ALPHA_CORAL_ABSOLUTE_ENCODER_VELOCITY_FACTOR, ALPHA_CORAL_WRIST_SOFT_LIMIT,
                    MotorIdConstants.CORAL_ALPHA_INTAKE_WRIST_CAN_ID));
        } else if (robotType == RobotType.BETA || robotType == RobotType.COMP) {
            return new CoralWristSubsystem(new CoralWristHardware(BETA_CORAL_ABSOLUTE_ENCODER_WRIST_POSITION_FACTOR,
                    BETA_CORAL_ABSOLUTE_ENCODER_VELOCITY_FACTOR, BETA_CORAL_WRIST_SOFT_LIMIT,
                    MotorIdConstants.CORAL_BETA_WRIST_CAN_ID));
        } else {
            return new CoralWristSubsystem(new CoralWristPlacebo());
        }
    }

    protected ElevatorSubsystem buildElevator() {
        if (robotType == RobotType.ALPHA) {
            return new ElevatorSubsystem(new AlphaElevatorHardware(ELEVATOR_ALPHA_MAX_HEIGHT));
        }
        if (robotType == RobotType.BETA) {
            return new ElevatorSubsystem(new KrakenElevatorHardware(ELEVATOR_BETA_MAX_HEIGHT, InvertedValue.CounterClockwise_Positive));
        }
        if (robotType == RobotType.COMP) {
            return new ElevatorSubsystem(new KrakenElevatorHardware(ELEVATOR_BETA_MAX_HEIGHT, InvertedValue.Clockwise_Positive));
        } else {
            return new ElevatorSubsystem(new ElevatorPlacebo());
        }
    }
}