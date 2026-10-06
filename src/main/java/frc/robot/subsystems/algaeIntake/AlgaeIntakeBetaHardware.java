package frc.robot.subsystems.algaeIntake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Seconds;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import static org.wpilib.units.Units.RPM;
import org.wpilib.units.measure.Time;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SpeedConstants;

public class AlgaeIntakeBetaHardware implements AlgaeIntakeIO {
        private final SparkMax algaeIntakeSparkMax;
        private final SparkClosedLoopController algaeIntakeClosedLoopController;
        private final RelativeEncoder algaeIntakeEncoder;

        private static final SparkMaxConfig algaeIntakeSparkMaxConfig = new SparkMaxConfig();

        private static final boolean LEFT_MOTOR_INVERTED = false;
        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        // gearbox ratio 9:1
        private static final double ENCODER_POSITION_FACTOR = (2 * Math.PI) / 9; // radians
        private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 9 / 60.0; // radians per second

        private static final double ALGAE_MOTOR_P = 0.001;
        private static final double ALGAE_MOTOR_I = 0;
        private static final double ALGAE_MOTOR_D = 0;
        private static final double ALGAE_MOTOR_FF = 0.01;
        private static final double ALGAE_MOTOR_MIN_OUTPUT = -1;
        private static final double ALGAE_MOTOR_MAX_OUTPUT = 1;

        private static final boolean POSITION_WRAPPING_ENABLED = true;

        private static final Current ALGAE_INTAKE_STALL_THRESHOLD = Amps.of(21);
        private static final Time ALGAE_INTAKE_STALL_TIME = Seconds.of(0.14);

        private static final Debouncer algaeIntakeDebouncer = new Debouncer(ALGAE_INTAKE_STALL_TIME.in(Seconds));

        public AlgaeIntakeBetaHardware() {
                algaeIntakeSparkMaxConfig.inverted(LEFT_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));
                algaeIntakeSparkMaxConfig.encoder.positionConversionFactor(ENCODER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                algaeIntakeSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(ALGAE_MOTOR_P, ALGAE_MOTOR_I, ALGAE_MOTOR_D)
                                .pid(ALGAE_MOTOR_P, ALGAE_MOTOR_I, ALGAE_MOTOR_D, ClosedLoopSlot.kSlot1)
                                .outputRange(ALGAE_MOTOR_MIN_OUTPUT, ALGAE_MOTOR_MAX_OUTPUT)
                                .positionWrappingEnabled(POSITION_WRAPPING_ENABLED);
                // 2027 REVLib: pidf feedforward term -> velocity feedforward (kV), per slot.
                algaeIntakeSparkMaxConfig.closedLoop.feedForward.kV(ALGAE_MOTOR_FF);
                algaeIntakeSparkMaxConfig.closedLoop.feedForward.kV(ALGAE_MOTOR_FF, ClosedLoopSlot.kSlot1);

                algaeIntakeSparkMaxConfig.signals
                                .appliedOutputPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                                .outputCurrentPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                                .busVoltagePeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS);

                algaeIntakeSparkMax = new SparkMax(MotorIdConstants.CAN_BUS_ID, MotorIdConstants.ALGAE_BETA_INTAKE_CAN_ID, MotorType.kBrushless);

                algaeIntakeEncoder = algaeIntakeSparkMax.getEncoder();

                algaeIntakeSparkMax.configure(algaeIntakeSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                algaeIntakeClosedLoopController = algaeIntakeSparkMax.getClosedLoopController();

        }

        public void setRollerSpeed(AngularVelocity speed) {
                algaeIntakeClosedLoopController.setSetpoint(speed.in(RPM), ControlType.kVelocity);
        }

        public double getVelocity() {
                return algaeIntakeEncoder.getVelocity().get();
        }

        @Override
        public void intake() {
                setRollerSpeed(SpeedConstants.BETA_ALGAE_INTAKE_SPEED);
        }

        @Override
        public void outtake() {
                setRollerSpeed(SpeedConstants.BETA_ALGAE_OUTTAKE_SPEED);
        }

        public double getCurrent() {
                return algaeIntakeSparkMax.getOutputCurrent().get();
        }

        @Override
        public boolean isStalling() {
                return algaeIntakeDebouncer.calculate(getCurrent() > ALGAE_INTAKE_STALL_THRESHOLD.in(Amps));
        }

        @Override
        public void passiveIntake() {
                if (!isStalling()) {
                        algaeIntakeClosedLoopController.setSetpoint(SpeedConstants.BETA_ALGAE_PASSIVE_SPEED.in(RPM),
                                        ControlType.kVelocity);
                }
        }

        @Override
        public void updateStates(AlgaeIntakeIOStates states) {
                states.intakeVelocity = getVelocity();
                states.leftAppliedVoltage = algaeIntakeSparkMax.getAppliedOutput().get()
                                * algaeIntakeSparkMax.getBusVoltage().get();
                states.leftCurrent = algaeIntakeSparkMax.getOutputCurrent().get();
        }
}