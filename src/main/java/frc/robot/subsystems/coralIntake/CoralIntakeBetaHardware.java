package frc.robot.subsystems.coralIntake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Time;
import frc.robot.CommandFactory.Setpoint;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SpeedConstants;

//intake to stall
public class CoralIntakeBetaHardware implements CoralIntakeIO {

    private static final boolean BETA_CORAL_INTAKE_MOTOR_INVERTED = true;
    private static final IdleMode BETA_CORAL_INTAKE_IDLE_MODE = IdleMode.kBrake;

    // 5:1 gearbox ratio
    private static final double BETA_CORAL_INTAKE_POSITION_CONVERSION_FACTOR = 1.0 / 5.0; // Rotations
    private static final double BETA_CORAL_INTAKE_VELOCITY_CONVERSION_FACTOR = 1.0 / 5.0; // RPM

    private static final double BETA_CORAL_INTAKE_P = 0;
    private static final double BETA_CORAL_INTAKE_I = 0;
    private static final double BETA_CORAL_INTAKE_D = 0;
    private static final double BETA_CORAL_INTAKE_FF = 5.0 / MotorConstants.VORTEX_FREE_SPEED.in(RPM);
    private static final double BETA_CORAL_INTAKE_PID_MIN_OUTPUT = -1.0;
    private static final double BETA_CORAL_INTAKE_PID_MAX_OUTPUT = 1.0;

    private static final boolean BETA_CORAL_INTAKE_POSITION_WRAPPING_ENABLED = true;

    private final RelativeEncoder betaCoralIntakeEncoder;
    private final SparkClosedLoopController betaCoralIntakeClosedLoop;
    private final SparkFlexConfig betaCoralIntakeConfig = new SparkFlexConfig();

    private final SparkFlex betaCoralIntakeSparkFlex;

    private double velocityGoal = 0;

    private static Debouncer CORAL_BETA_DEBOUNCER;
    private static Current coralIntakeStallThreshold;

    public CoralIntakeBetaHardware(Time debouncerTime, Current stallThreshold) {
        CORAL_BETA_DEBOUNCER = new Debouncer(debouncerTime.in(Seconds));
        coralIntakeStallThreshold = stallThreshold;
        betaCoralIntakeConfig.inverted(BETA_CORAL_INTAKE_MOTOR_INVERTED).idleMode(BETA_CORAL_INTAKE_IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.VORTEX_CURRENT_LIMIT.in(Amps));
        betaCoralIntakeConfig.encoder.positionConversionFactor(BETA_CORAL_INTAKE_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(BETA_CORAL_INTAKE_VELOCITY_CONVERSION_FACTOR);
        betaCoralIntakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(BETA_CORAL_INTAKE_P, BETA_CORAL_INTAKE_I, BETA_CORAL_INTAKE_D)
                .outputRange(BETA_CORAL_INTAKE_PID_MIN_OUTPUT, BETA_CORAL_INTAKE_PID_MAX_OUTPUT)
                .positionWrappingEnabled(BETA_CORAL_INTAKE_POSITION_WRAPPING_ENABLED);
        // 2027 REVLib: pidf feedforward term -> velocity feedforward (kV).
        betaCoralIntakeConfig.closedLoop.feedForward.kV(BETA_CORAL_INTAKE_FF);

        betaCoralIntakeConfig.signals
                .appliedOutputPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                .busVoltagePeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS)
                .outputCurrentPeriodMs(Constants.SpeedConstants.LOGGING_FREQUENCY_MS);

        betaCoralIntakeSparkFlex = new SparkFlex(MotorIdConstants.CAN_BUS_ID, MotorIdConstants.CORAL_BETA_INTAKE_CAN_ID,
                MotorType.kBrushless);

        betaCoralIntakeSparkFlex.configure(betaCoralIntakeConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        betaCoralIntakeEncoder = betaCoralIntakeSparkFlex.getEncoder();
        betaCoralIntakeClosedLoop = betaCoralIntakeSparkFlex.getClosedLoopController();
    }

    @Override
    public void intake() {
        betaCoralIntakeClosedLoop.setSetpoint(SpeedConstants.BETA_CORAL_INTAKE_SPEED.in(RPM), ControlType.kVelocity);
        velocityGoal = SpeedConstants.BETA_CORAL_INTAKE_SPEED.in(RPM);
    }

    @Override
    public void setOuttakeSpeed(Setpoint setpoint) {
        double desiredVelocity = 0;
        if (setpoint == Setpoint.L_ONE) {
            desiredVelocity = SpeedConstants.BETA_CORAL_L1_OUTTAKE_SPEED.in(RPM);
        } else if (setpoint == Setpoint.L_FOUR) {
            desiredVelocity = SpeedConstants.BETA_CORAL_L4_OUTTAKE_SPEED.in(RPM);
        } else {
            desiredVelocity = SpeedConstants.BETA_CORAL_OUTTAKE_SPEED.in(RPM);
        }

        betaCoralIntakeClosedLoop.setSetpoint(desiredVelocity, ControlType.kVelocity);
        velocityGoal = desiredVelocity;
    }

    @Override
    public void setZero() {
        betaCoralIntakeClosedLoop.setSetpoint(0, ControlType.kVelocity);
        velocityGoal = 0;
    }

    @Override
    public boolean isStalling() {
        boolean isStalling = CORAL_BETA_DEBOUNCER
                .calculate(betaCoralIntakeSparkFlex.getOutputCurrent().get() > coralIntakeStallThreshold.in(Amps));
        return isStalling;
    }

    @Override
    public void passiveIntake() {
        if (!isStalling()) {
            betaCoralIntakeClosedLoop.setSetpoint(SpeedConstants.BETA_CORAL_PASSIVE_SPEED.in(RPM),
                    ControlType.kVelocity);
        }
    }

    @Override
    public void passiveIntakeIgnoringStall() {
            betaCoralIntakeClosedLoop.setSetpoint(SpeedConstants.BETA_CORAL_PASSIVE_SPEED.in(RPM),
                    ControlType.kVelocity);
    }

    @Override
    public void updateStates(CoralIntakeIOStates states) {
        states.velocity = betaCoralIntakeEncoder.getVelocity().get();
        states.goalVelocity = velocityGoal;
        states.leftCurrent = betaCoralIntakeSparkFlex.getOutputCurrent().get();
        states.rightCurrent = betaCoralIntakeSparkFlex.getOutputCurrent().get();
        states.leftAppliedVoltage = betaCoralIntakeSparkFlex.getAppliedOutput().get()
                * betaCoralIntakeSparkFlex.getBusVoltage().get();
        states.rightAppliedVoltage = betaCoralIntakeSparkFlex.getAppliedOutput().get()
                * betaCoralIntakeSparkFlex.getBusVoltage().get();
    }
}
