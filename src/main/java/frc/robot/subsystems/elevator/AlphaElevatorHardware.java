package frc.robot.subsystems.elevator;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.math.trajectory.TrapezoidProfile.State;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class AlphaElevatorHardware implements ElevatorIO {

        public static final class AlphaElevatorConstants {
                private static final Distance METERS_PER_REVOLUTION = Inches.of(0.67); // (1/9)(1.92 * pi)
                private static final Distance ALLOWED_SETPOINT_ERROR = Inches.of(.25);
                private static final LinearVelocity MAX_VEL = MetersPerSecond.of(1.5);
                private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(3);
                private static final Voltage P_VALUE = Volts.of(6.0);
                private static final Voltage I_VALUE = Volts.of(0);
                private static final Voltage D_VALUE = Volts.of(0);
                private static final Voltage FEEDFORWARD_VALUE = Volts.of(1.0 / 917);
                private static final Voltage ARBITRARY_FF_GRAVITY_COMPENSATION = Volts.of(0.28); // calculated using
                                                                                                 // recalc
                private static final double OUTPUT_RANGE_MIN_VALUE = -1;
                private static final double OUTPUT_RANGE_MAX_VALUE = 1;
                private static final double P_VALUE_VELOCITY = 0.0001;
                private static final double I_VALUE_VELOCITY = 0;
                private static final double D_VALUE_VELOCITY = 0;
                private static final double kDt = 0.02;
                private static final Distance MAX_ELEVATOR_HEIGHT = Inches.of(34.25); // inches
        }

        private SparkFlex elevatorRightMotorFollower, elevatorLeftMotorLeader;
        private SparkFlexConfig globalMotorConfig, rightMotorConfigFollower, leftMotorConfigLeader;
        private SparkClosedLoopController leftClosedLoopController;
        private RelativeEncoder leftEncoder;
        private TrapezoidProfile elevatorMotionProfile;
        public State intermediateSetpointState = new State();
        private State goalState = new State();

        public AlphaElevatorHardware(Distance maxElevatorHeight) {

                globalMotorConfig = new SparkFlexConfig();
                rightMotorConfigFollower = new SparkFlexConfig();
                leftMotorConfigLeader = new SparkFlexConfig();

                elevatorRightMotorFollower = new SparkFlex(MotorIdConstants.CAN_BUS_ID, MotorIdConstants.RIGHT_ALPHA_ELEVATOR_MOTOR_ID,
                                MotorType.kBrushless);
                elevatorLeftMotorLeader = new SparkFlex(MotorIdConstants.CAN_BUS_ID, MotorIdConstants.LEFT_ALPHA_ELEVATOR_MOTOR_ID,
                                MotorType.kBrushless);

                leftClosedLoopController = elevatorLeftMotorLeader.getClosedLoopController();

                leftEncoder = elevatorLeftMotorLeader.getEncoder();
                leftEncoder.setPosition(0);

                elevatorMotionProfile = new TrapezoidProfile(
                                new Constraints(AlphaElevatorConstants.MAX_VEL.in(MetersPerSecond),
                                                AlphaElevatorConstants.MAX_ACCEL.in(MetersPerSecondPerSecond)));

                globalMotorConfig.encoder
                                .positionConversionFactor(AlphaElevatorConstants.METERS_PER_REVOLUTION.in(Meters))
                                .velocityConversionFactor(AlphaElevatorConstants.METERS_PER_REVOLUTION.in(Meters) / 60);

                globalMotorConfig.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .p(AlphaElevatorConstants.P_VALUE.in(Volts), ClosedLoopSlot.kSlot0)
                                .i(AlphaElevatorConstants.I_VALUE.in(Volts), ClosedLoopSlot.kSlot0)
                                .d(AlphaElevatorConstants.D_VALUE.in(Volts), ClosedLoopSlot.kSlot0)
                                .outputRange(-1, 1)
                                .p(AlphaElevatorConstants.P_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
                                .i(AlphaElevatorConstants.I_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
                                .d(AlphaElevatorConstants.D_VALUE_VELOCITY, ClosedLoopSlot.kSlot1)
                                .outputRange(AlphaElevatorConstants.OUTPUT_RANGE_MIN_VALUE,
                                                AlphaElevatorConstants.OUTPUT_RANGE_MAX_VALUE, ClosedLoopSlot.kSlot1);
                // 2027 REVLib: velocityFF(...) was removed; velocity feedforward is now kV on
                // the separate FeedForwardConfig.
                // https://docs.revrobotics.com/revlib/spark/closed-loop/closed-loop-control-getting-started#f-parameter
                globalMotorConfig.closedLoop.feedForward.kV(AlphaElevatorConstants.FEEDFORWARD_VALUE.in(Volts),
                                ClosedLoopSlot.kSlot1);

                globalMotorConfig.softLimit
                                .forwardSoftLimit((maxElevatorHeight).in(Meters) - 0.02) // a little less
                                                                                         // than max height
                                                                                         // for safety
                                .forwardSoftLimitEnabled(true)
                                .reverseSoftLimit(0)
                                .reverseSoftLimitEnabled(true);

                globalMotorConfig.signals
                                .appliedOutputPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                                .busVoltagePeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                                .outputCurrentPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS);

                leftMotorConfigLeader
                                .apply(globalMotorConfig)
                                .inverted(false)
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));

                rightMotorConfigFollower
                                .follow(MotorIdConstants.LEFT_ALPHA_ELEVATOR_MOTOR_ID, true)
                                .apply(globalMotorConfig)
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));

                elevatorLeftMotorLeader.configure(leftMotorConfigLeader, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                elevatorRightMotorFollower.configure(rightMotorConfigFollower, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
        }

        @Override
        public void resetSetpointsToCurrentPosition() {
                goalState.position = leftEncoder.getPosition().get();
                intermediateSetpointState.position = leftEncoder.getPosition().get();

                goalState.velocity = 0;
                intermediateSetpointState.velocity = 0;
        }

        @Override
        public void setGoalPosition(Distance newGoalPosition) {
                goalState.position = newGoalPosition.in(Meters);
        }

        @Override
        public void setIntermediateSetpoint(Distance position, double velocity) {
                intermediateSetpointState.position = position.in(Meters);
                intermediateSetpointState.velocity = velocity;
        }

        public void incrementGoalPosition(Distance changeInGoalPosition) {
                goalState.position += changeInGoalPosition.in(Meters);
        }

        @Override
        public void calculateNextIntermediateSetpoint() {
                intermediateSetpointState = elevatorMotionProfile.calculate(AlphaElevatorConstants.kDt,
                                intermediateSetpointState, goalState);
                leftClosedLoopController.setSetpoint(intermediateSetpointState.position, ControlType.kPosition,
                                ClosedLoopSlot.kSlot0);
        }

        @Override
        public double getEncoderVelocity() {
                return leftEncoder.getVelocity().get();
        }

        @Override
        public double getEncoderPosition() {
                return leftEncoder.getPosition().get();
        }
        
        public boolean isElevatorHeightAboveSpeedLimitingThreshold()
        {
                return false; 
        }

        @Override
        public void updateStates(ElevatorIOInputs states) {
                states.position = getEncoderPosition();
                states.velocity = getEncoderVelocity();
                states.appliedVoltageRight = elevatorRightMotorFollower.getAppliedOutput().get()
                                * elevatorRightMotorFollower.getBusVoltage().get();
                states.appliedVoltageLeft = elevatorLeftMotorLeader.getAppliedOutput().get()
                                * elevatorLeftMotorLeader.getBusVoltage().get();
                states.current = elevatorLeftMotorLeader.getOutputCurrent().get();
                states.goalPosition = goalState.position;
                states.intermediateSetpointPosition = intermediateSetpointState.position;
        }

        public void setSpeedManualControl(double speed) {
                // place holding method
        }

}
