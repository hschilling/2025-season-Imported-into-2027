package frc.robot.subsystems.climber;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.InchesPerSecond;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Radians;

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

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.hardware.expansionhub.ExpansionHubServo;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;

public class ClimberHardware implements ClimberIO {

        public static final class ClimberConstants {
                // use motor current limit in constants
                private static final double P_VALUE = 0.0;
                private static final double I_VALUE = 0.0;
                private static final double D_VALUE = 0.0;

                private static final Distance UPPER_LIMIT = Inches.of(31.465);
                // INIT TO HERE
                private static final Distance LOWER_LIMIT = Inches.of(16);
                private static final Distance ZERO_POSITION = Inches.of(14.665);

                // TODO: check!
                private static final double CLIMBER_POSITION_CONVERSION_FACTOR = 1.5 * 1.0 / 100.0 * Math.PI;
                private static final double CLIMBER_VELOCITY_CONVERSION_FACTOR = CLIMBER_POSITION_CONVERSION_FACTOR
                                / 60.0;
                private static final double CLIMBER_VELOCITY_FF = 1.0
                                / (MotorConstants.NEO_FREE_SPEED.in(RPM) * CLIMBER_VELOCITY_CONVERSION_FACTOR);
                private static final int SERVO_CHANNEL = 0;
                // TODO(SystemCore): confirm which Expansion Hub (USB id) the climber ratchet
                // servo is wired to once the 2027 control system is wired up.
                private static final int SERVO_USB_ID = 0;

                private static final boolean LEFT_CLIMBER_INVERTED = false;
                private static final boolean RIGHT_CLIMBER_INVERTED = false;

                private static final SparkBaseConfig.IdleMode CLIMBER_IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;

        }

        private final SparkMax leftClimber = new SparkMax(MotorIdConstants.CAN_BUS_ID, MotorIdConstants.LEFT_CLIMBER_CAN_ID, MotorType.kBrushless);
        private final SparkMax rightClimber = new SparkMax(MotorIdConstants.CAN_BUS_ID, MotorIdConstants.RIGHT_CLIMBER_CAN_ID, MotorType.kBrushless);
        private final ExpansionHubServo climberServo = new ExpansionHubServo(ClimberConstants.SERVO_USB_ID,
                        ClimberConstants.SERVO_CHANNEL);

        private final SparkMaxConfig leftClimberConfig = new SparkMaxConfig();
        private final SparkMaxConfig rightClimberConfig = new SparkMaxConfig();

        private final RelativeEncoder leftClimberEncoder = leftClimber.getEncoder();
        private final SparkClosedLoopController climberClosedLoopController = leftClimber.getClosedLoopController();

        private Angle climberGoalAngle = Radians.of(0.0);
        private Angle servoGoalAngle = Radians.of(0.0);

        public ClimberHardware() {

                leftClimberConfig.inverted(ClimberConstants.LEFT_CLIMBER_INVERTED)
                                .idleMode(ClimberConstants.CLIMBER_IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));

                rightClimberConfig.inverted(ClimberConstants.RIGHT_CLIMBER_INVERTED)
                                .idleMode(ClimberConstants.CLIMBER_IDLE_MODE)
                                .smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));

                leftClimberConfig.encoder.positionConversionFactor(ClimberConstants.CLIMBER_POSITION_CONVERSION_FACTOR)
                                .velocityConversionFactor(ClimberConstants.CLIMBER_VELOCITY_CONVERSION_FACTOR);

                leftClimberConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(ClimberConstants.P_VALUE, ClimberConstants.I_VALUE, ClimberConstants.D_VALUE);
				leftClimberConfig.closedLoop.feedForward.kV(ClimberConstants.CLIMBER_VELOCITY_FF);

                leftClimberConfig.softLimit
                                .forwardSoftLimit(ClimberConstants.UPPER_LIMIT.in(Inches))
                                .forwardSoftLimitEnabled(true)
                                .reverseSoftLimit(ClimberConstants.LOWER_LIMIT.in(Inches))
                                .reverseSoftLimitEnabled(true);

                rightClimberConfig.follow(leftClimber.getDeviceId(), true);

                leftClimber.configure(leftClimberConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                rightClimber.configure(rightClimberConfig, ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters);
                // Will made me do this
                leftClimberEncoder.setPosition(ClimberConstants.ZERO_POSITION.in(Inches)); // inches
        }

        public void setSpeed(LinearVelocity speed) {
               climberClosedLoopController.setSetpoint(speed.in(InchesPerSecond), ControlType.kVelocity);
                if (speed.equals(InchesPerSecond.zero())) {
                        servoGoalAngle = Degrees.of(90);
                        climberServo.setAngle(servoGoalAngle);
                } else {
                        servoGoalAngle = Degrees.of(180);
                        climberServo.setAngle(servoGoalAngle);
                }
        }

        public void updateStates(ClimberIOInputs inputs) {
                inputs.climberAngle = leftClimberEncoder.getPosition().get();
                inputs.climberVelocity = leftClimberEncoder.getVelocity().get();
                inputs.climberGoalAngle = climberGoalAngle.in(Degrees);

                // ExpansionHubServo has no angle read-back; report the last commanded angle.
                inputs.servoAngle = servoGoalAngle.in(Degrees);
                // inputs.servoVelocity = climberServo.getSpeed();
                // inputs.servoGoalAngle = servoGoalAngle.in(Degrees);
        }

}
