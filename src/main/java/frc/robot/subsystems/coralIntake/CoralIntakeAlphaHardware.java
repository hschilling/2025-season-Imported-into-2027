package frc.robot.subsystems.coralIntake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Time;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.CommandFactory.Setpoint;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SpeedConstants;

public class CoralIntakeAlphaHardware implements CoralIntakeIO {

        private final SparkMax coralIntakeLeftSparkMax;
        private final SparkMax coralIntakeRightSparkMax;

        private final SparkClosedLoopController coralIntakeLeftClosedLoopController;
        private final SparkClosedLoopController coralIntakeRightClosedLoopController;

        private final RelativeEncoder coralIntakeLeftEncoder;
        private final RelativeEncoder coralIntakeRightEncoder;

        private static final SparkMaxConfig leftSparkMaxConfig = new SparkMaxConfig();
        private static final SparkMaxConfig rightSparkMaxConfig = new SparkMaxConfig();

        private static final boolean LEFT_MOTOR_INVERTED = false;
        private static final boolean RIGHT_MOTOR_INVERTED = true;
        private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
        private static final double ENCODER_ROLLER_POSITION_FACTOR = (2 * Math.PI); // radians
        private static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        private static final double INTAKE_MOTOR_P = 0.00018;
        private static final double INTAKE_MOTOR_I = 0.0;
        private static final double INTAKE_MOTOR_D = 0.0;
        private static final double INTAKE_MOTOR_FF = 0.001;
        private static final double INTAKE_MOTOR_MIN_OUTPUT = -1.0;
        private static final double INTAKE_MOTOR_MAX_OUTPUT = 1.0;

        private static final boolean POSITION_WRAPPING_ENABLED_SIDE_MOTORS = true;

        private static final Time ALPHA_CORAL_DEBOUNCER_TIME = Seconds.of(0.06);
        private static final Current CORAL_INTAKE_STALL_THRESHOLD = Amps.of(0.004);
        private static final Debouncer CORAL_ALPHA_DEBOUNCER = new Debouncer(ALPHA_CORAL_DEBOUNCER_TIME.in(Seconds));

        
        public CoralIntakeAlphaHardware() {
                leftSparkMaxConfig.inverted(LEFT_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));
                leftSparkMaxConfig.encoder.positionConversionFactor(ENCODER_ROLLER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                leftSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(INTAKE_MOTOR_P, INTAKE_MOTOR_I, INTAKE_MOTOR_D)
                                .outputRange(INTAKE_MOTOR_MIN_OUTPUT, INTAKE_MOTOR_MAX_OUTPUT)
                                .positionWrappingEnabled(POSITION_WRAPPING_ENABLED_SIDE_MOTORS);
                // 2027 REVLib: pidf feedforward term -> velocity feedforward (kV).
                leftSparkMaxConfig.closedLoop.feedForward.kV(INTAKE_MOTOR_FF);

                leftSparkMaxConfig.signals
                                .appliedOutputPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                                .busVoltagePeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                                .outputCurrentPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS);

                rightSparkMaxConfig.inverted(RIGHT_MOTOR_INVERTED).idleMode(IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO550_CURRENT_LIMIT.in(Amps));
                rightSparkMaxConfig.encoder.positionConversionFactor(ENCODER_ROLLER_POSITION_FACTOR)
                                .velocityConversionFactor(ENCODER_VELOCITY_FACTOR);
                rightSparkMaxConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(INTAKE_MOTOR_P, INTAKE_MOTOR_I, INTAKE_MOTOR_D)
                                .outputRange(INTAKE_MOTOR_MIN_OUTPUT, INTAKE_MOTOR_MAX_OUTPUT);
                // 2027 REVLib: pidf feedforward term -> velocity feedforward (kV).
                rightSparkMaxConfig.closedLoop.feedForward.kV(INTAKE_MOTOR_FF);

                rightSparkMaxConfig.signals
                                .appliedOutputPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                                .busVoltagePeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                                .outputCurrentPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS);

                coralIntakeLeftSparkMax = new SparkMax(MotorIdConstants.CAN_BUS_ID, MotorIdConstants.CORAL_ALPHA_INTAKE_LEFT_CAN_ID, MotorType.kBrushless);
                coralIntakeRightSparkMax = new SparkMax(MotorIdConstants.CAN_BUS_ID, MotorIdConstants.CORAL_ALPHA_INTAKE_RIGHT_CAN_ID,
                                MotorType.kBrushless);

                coralIntakeLeftEncoder = coralIntakeLeftSparkMax.getEncoder();
                coralIntakeRightEncoder = coralIntakeRightSparkMax.getEncoder();

                coralIntakeLeftSparkMax.configure(leftSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                coralIntakeRightSparkMax.configure(rightSparkMaxConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);

                coralIntakeLeftClosedLoopController = coralIntakeLeftSparkMax.getClosedLoopController();
                coralIntakeRightClosedLoopController = coralIntakeRightSparkMax.getClosedLoopController();
        }

        public void setRollerSpeed(AngularVelocity speed) {
                coralIntakeRightSparkMax.setThrottle(speed.in(RPM));
                coralIntakeLeftSparkMax.setThrottle(speed.in(RPM));
        }

        public double getVelocity() {
                return coralIntakeLeftEncoder.getVelocity().get();
        }

        public double getCurrent() {
                return coralIntakeLeftSparkMax.getOutputCurrent().get();
        }

        @Override
        public void intake() {
                coralIntakeLeftClosedLoopController.setSetpoint(SpeedConstants.ALPHA_CORAL_INTAKE_SPEED.in(RPM), ControlType.kVelocity);
                coralIntakeRightClosedLoopController.setSetpoint(SpeedConstants.ALPHA_CORAL_INTAKE_SPEED.in(RPM), ControlType.kVelocity);
        }

        @Override
        public void setOuttakeSpeed(Setpoint setpoint) {
                double desiredVelocity = 0;
                if (setpoint == Setpoint.L_ONE) {
                    desiredVelocity = SpeedConstants.ALPHA_CORAL_L1_OUTTAKE_SPEED.in(RPM);
                    coralIntakeLeftClosedLoopController.setSetpoint(desiredVelocity, ControlType.kVelocity);
                    coralIntakeRightClosedLoopController.setSetpoint(desiredVelocity, ControlType.kVelocity);   
                } else {
                     desiredVelocity = SpeedConstants.ALPHA_CORAL_OUTTAKE_SPEED.in(RPM);
                     coralIntakeLeftClosedLoopController.setSetpoint(desiredVelocity, ControlType.kVelocity);
                     coralIntakeRightClosedLoopController.setSetpoint(desiredVelocity, ControlType.kVelocity);  
                }

                SmartDashboard.putNumber("coralIntake/desiredVelocity", desiredVelocity);
            }

        @Override
        public void setZero() {
                coralIntakeLeftClosedLoopController.setSetpoint(0, ControlType.kVelocity);
                coralIntakeRightClosedLoopController.setSetpoint(0, ControlType.kVelocity);
        }

        @Override
        public boolean isStalling() {
                return (CORAL_ALPHA_DEBOUNCER.calculate(getCurrent() > CORAL_INTAKE_STALL_THRESHOLD.in(Amps)));
        }

        @Override
        public void passiveIntake() {
            if (!isStalling()) {
                coralIntakeLeftClosedLoopController.setSetpoint(SpeedConstants.ALPHA_CORAL_PASSIVE_SPEED.in(RPM), ControlType.kVelocity);
            }
        }

        public void passiveIntakeIgnoringStall() 
        {
                
        }

        @Override
        public void updateStates(CoralIntakeIOStates states) {
                states.velocity = getVelocity();
                states.leftAppliedVoltage = coralIntakeLeftSparkMax.getAppliedOutput().get()
                                * coralIntakeLeftSparkMax.getBusVoltage().get();
                states.rightAppliedVoltage = coralIntakeRightSparkMax.getAppliedOutput().get()
                                * coralIntakeRightSparkMax.getBusVoltage().get();
                states.leftCurrent = coralIntakeLeftSparkMax.getOutputCurrent().get();
                states.rightCurrent = coralIntakeRightSparkMax.getOutputCurrent().get();
        }
}