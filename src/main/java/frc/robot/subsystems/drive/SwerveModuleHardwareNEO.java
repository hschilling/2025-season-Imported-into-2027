package frc.robot.subsystems.drive;

import frc.robot.Constants.MotorIdConstants;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorConstants;

public class SwerveModuleHardwareNEO implements SwerveModuleIO {

        private SparkMax drivingSparkMax;
        private SparkMax turningSparkMax;

        private RelativeEncoder drivingRelativeEncoder;
        private SparkAbsoluteEncoder turningAbsoluteEncoder;
        private double driveDesiredVelocity;
        private double desiredAngle;

        private SparkClosedLoopController drivingPidController;
        private SparkClosedLoopController turningPidController;

        private double chassisAngularOffset;

        private String name;

        private static final SparkMaxConfig sparkMaxConfigDriving = new SparkMaxConfig();
        private static final SparkMaxConfig sparkMaxConfigTurning = new SparkMaxConfig();

        // drivings are NEOs, turnings are NEO 550s
        // THIS IS 13 ON COMP BOT
        private static final int DRIVING_MOTOR_PINION_TEETH = 14;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of the steering motor in the MAXSwerve Module.
        private static final boolean TURNING_ENCODER_INVERTED = true;
        private static final boolean DRIVING_MOTOR_INVERTED = false;
        private static final boolean TURNING_MOTOR_INVERTED = false;

        // Calculations required for driving motor conversion factors and feed forward
        private static final Distance WHEEL_DIAMETER = Inches.of(3);
        private static final Distance WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER.times(Math.PI);

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        // This is also the gear ratio (14T)

        private static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);

        private static final AngularVelocity DRIVE_WHEEL_FREE_SPEED = RotationsPerSecond
                        .of((MotorConstants.NEO_FREE_SPEED.in(RotationsPerSecond) *
                                        WHEEL_CIRCUMFERENCE.in(Meters)) / (DRIVING_MOTOR_REDUCTION));

        private static final Distance DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER.times(Math.PI))
                        .div(DRIVING_MOTOR_REDUCTION).div((260.0 / 254)); // meters
        private static final Distance DRIVING_ENCODER_VELOCITY_FACTOR = DRIVING_ENCODER_POSITION_FACTOR.div(60); // meters
                                                                                                                    // per
                                                                                                                    // second

        private static final double TURNING_ENCODER_POSITION_FACTOR = Units.rotationsToRadians(1);
        private static final double TURNING_ENCODER_VELOCITY_FACTOR = Units.rotationsToRadians(1) / 60.0;

        private static final boolean TURNING_ENCODER_POSITION_WRAPPING = true;
        private static final double TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        private static final double TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR; // radians

        private static final double DRIVING_P = 0.2;
        private static final double DRIVING_I = 0;
        private static final double DRIVING_D = 0;
        private static final double DRIVING_FF = 1 / DRIVE_WHEEL_FREE_SPEED.in(RotationsPerSecond);
        private static final double DRIVING_MIN_OUTPUT = -1;
        private static final double DRIVING_MAX_OUTPUT = 1;

        private static final double TURNING_P = 1.0;
        private static final double TURNING_I = 0;
        private static final double TURNING_D = 0.001;
        private static final double TURNING_FF = 0;
        private static final double TURNING_MIN_OUTPUT = -1;
        private static final double TURNING_MAX_OUTPUT = 1;

        private static final double VOLTAGE_COMPENSATION = 12;

        private static final SparkBaseConfig.IdleMode DRIVING_MOTOR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        private static final SparkBaseConfig.IdleMode TURNING_MOTOR_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

        public SwerveModuleHardwareNEO(int drivingCanId, int turningCanId, double chassisAngularOffset, String name) {
                this.chassisAngularOffset = chassisAngularOffset;
                this.name = name;

                drivingSparkMax = new SparkMax(MotorIdConstants.CAN_BUS_ID, drivingCanId, MotorType.kBrushless);
                turningSparkMax = new SparkMax(MotorIdConstants.CAN_BUS_ID, turningCanId, MotorType.kBrushless);

                sparkMaxConfigDriving.inverted(DRIVING_MOTOR_INVERTED).idleMode(DRIVING_MOTOR_IDLE_MODE)
                                .smartCurrentLimit(
                                                (int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps))
                                .voltageCompensation(VOLTAGE_COMPENSATION);
                sparkMaxConfigDriving.encoder.positionConversionFactor(DRIVING_ENCODER_POSITION_FACTOR.in(Meters))
                                .velocityConversionFactor(DRIVING_ENCODER_VELOCITY_FACTOR.in(Meters));
                sparkMaxConfigDriving.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(DRIVING_P, DRIVING_I, DRIVING_D)
                                .outputRange(DRIVING_MIN_OUTPUT, DRIVING_MAX_OUTPUT);
                // 2027 REVLib: the old pidf(..., ff) feedforward term is now the velocity
                // feedforward (kV) on the separate FeedForwardConfig.
                sparkMaxConfigDriving.closedLoop.feedForward.kV(DRIVING_FF);

                sparkMaxConfigTurning.inverted(TURNING_MOTOR_INVERTED).idleMode(TURNING_MOTOR_IDLE_MODE)
                                .smartCurrentLimit(
                                                (int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps))
                                .voltageCompensation(VOLTAGE_COMPENSATION);
                sparkMaxConfigTurning.absoluteEncoder.positionConversionFactor(TURNING_ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(TURNING_ENCODER_VELOCITY_FACTOR);
                sparkMaxConfigTurning.absoluteEncoder.inverted(TURNING_ENCODER_INVERTED);
                sparkMaxConfigTurning.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                                .pid(TURNING_P, TURNING_I, TURNING_D)
                                .outputRange(TURNING_MIN_OUTPUT, TURNING_MAX_OUTPUT)
                                .positionWrappingEnabled(TURNING_ENCODER_POSITION_WRAPPING)
                                .positionWrappingInputRange(
                                                TURNING_ENCODER_POSITION_PID_MIN_INPUT,
                                                TURNING_ENCODER_POSITION_PID_MAX_INPUT);

                drivingSparkMax.configure(sparkMaxConfigDriving, ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);
                turningSparkMax.configure(sparkMaxConfigTurning, ResetMode.kResetSafeParameters,
                                PersistMode.kNoPersistParameters);

                drivingRelativeEncoder = drivingSparkMax.getEncoder();
                turningAbsoluteEncoder = turningSparkMax.getAbsoluteEncoder();

                drivingPidController = drivingSparkMax.getClosedLoopController();
                turningPidController = turningSparkMax.getClosedLoopController();
        }

        public void setDriveEncoderPosition(double position) {
                drivingRelativeEncoder.setPosition(position);
        };

        public void setDesiredDriveSpeedMPS(double speed) {
                drivingPidController.setSetpoint(speed, ControlType.kVelocity);
                this.driveDesiredVelocity = speed;
        };

        public void setDesiredTurnAngle(double angle) {
                turningPidController.setSetpoint(angle, ControlType.kPosition);
                this.desiredAngle = angle;
        };

        public double getChassisAngularOffset() {
                return chassisAngularOffset;
        }

        public double getTurnEncoderPosition() {
                return turningAbsoluteEncoder.getPosition().get();
        }

        public double getDriveEncoderSpeedMPS() {
                return drivingRelativeEncoder.getVelocity().get();
        }

        public double getDriveEncoderPosition() {
                return drivingRelativeEncoder.getPosition().get();
        }

        public void updateStates(SwerveModuleIOStates states) {
                states.desiredAngle = Units.radiansToDegrees(this.desiredAngle);
                states.turnAngle = Units.radiansToDegrees(turningAbsoluteEncoder.getPosition().get());
                states.driveDesiredVelocity = this.driveDesiredVelocity;
                states.driveVelocity = drivingRelativeEncoder.getVelocity().get();
                states.driveEncoderPos = drivingRelativeEncoder.getPosition().get();
                states.driveVoltage = drivingSparkMax.getBusVoltage().get() * drivingSparkMax.getAppliedOutput().get();
                states.turnVoltage = turningSparkMax.getBusVoltage().get() * turningSparkMax.getAppliedOutput().get();
                states.driveCurrent = drivingSparkMax.getOutputCurrent().get();
                states.turnCurrent = turningSparkMax.getOutputCurrent().get();

                SmartDashboard.putNumber("Swerve/module " + name + "/turn desired angle(deg)", states.desiredAngle);
                SmartDashboard.putNumber("Swerve/module " + name + "/turn angle(deg)",
                                states.turnAngle);
                SmartDashboard.putNumber("Swerve/module " + name + "/drive desired velocity(mps)",
                                states.driveDesiredVelocity);
                SmartDashboard.putNumber("Swerve/module " + name + "/drive velocity(mps)", states.driveVelocity);
                SmartDashboard.putNumber("Swerve/module " + name + "/drive encoder position(m)",
                                states.driveEncoderPos);
                SmartDashboard.putNumber("Swerve/module " + name + "/drive voltage(volt)", states.driveVoltage);
                SmartDashboard.putNumber("Swerve/module " + name + "/turn voltage(volt)", states.turnVoltage);
                SmartDashboard.putNumber("Swerve/module " + name + "/drive current(amps)", states.driveCurrent);
                SmartDashboard.putNumber("Swerve/module " + name + "/turn current(amps)", states.turnCurrent);
        }
}