package frc.robot.subsystems.algaeIntake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.PersistMode;
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
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.MotorIdConstants;
import frc.robot.Constants.SpeedConstants;

public class AlgaeIntakeCompHardware implements AlgaeIntakeIO {
    private final SparkMax algaeIntakeSparkMax;
    private final SparkClosedLoopController compAlgaeIntakeClosedLoop;
    private final RelativeEncoder compAlgaeIntakeRelativeEncoder;

    private static final SparkMaxConfig compAlgaeIntakeConfig = new SparkMaxConfig();

    private static final boolean COMP_ALGAE_INTAKE_MOTOR_INVERTED = false;
    private static final SparkBaseConfig.IdleMode IDLE_MODE = SparkBaseConfig.IdleMode.kBrake;
    private static final double COMP_ALGAE_INTAKE_POSITION_CONVERSION_FACTOR = 1.0 / 3;
    private static final double COMP_ALGAE_INTAKE_VELOCITY_CONVERSION_FACTOR = 1.0 / 3 / 60;

    private static final double COMP_ALGAE_INTAKE_P = 0;
    private static final double COMP_ALGAE_INTAKE_I = 0;
    private static final double COMP_ALGAE_INTAKE_D = 0;
    private static final double COMP_ALGAE_INTAKE_FeedForward = 0.001;

    private static final double COMP_ALGAE_INTAKE_MIN_INPUT = -1;
    private static final double COMP_ALGAE_INTAKE_MAX_OUTPUT = 1;

    private double goalVelocity = 0.0;

    private static final boolean COMP_ALGAE_INTAKE_POSITION_WRAPPING_ENABLED = true;

     private static final Current ALGAE_INTAKE_STALL_THRESHOLD = Amps.of(35);
        private static final Time ALGAE_INTAKE_STALL_TIME = Seconds.of(0.15);
        private static final AngularVelocity ALGAE_INTAKE_STALL_VELOCITY = SpeedConstants.BETA_ALGAE_INTAKE_SPEED;

        private static final Debouncer algaeIntakeDebouncer = new Debouncer(ALGAE_INTAKE_STALL_TIME.in(Seconds));

    public AlgaeIntakeCompHardware() {
        compAlgaeIntakeConfig.inverted(COMP_ALGAE_INTAKE_MOTOR_INVERTED)
                .idleMode(IDLE_MODE)
                .smartCurrentLimit((int) MotorConstants.NEO_CURRENT_LIMIT.in(Amps));
        compAlgaeIntakeConfig.encoder.positionConversionFactor(COMP_ALGAE_INTAKE_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(COMP_ALGAE_INTAKE_VELOCITY_CONVERSION_FACTOR);
        compAlgaeIntakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(COMP_ALGAE_INTAKE_P, COMP_ALGAE_INTAKE_I, COMP_ALGAE_INTAKE_D)
                .outputRange(COMP_ALGAE_INTAKE_MIN_INPUT, COMP_ALGAE_INTAKE_MAX_OUTPUT)
                .positionWrappingEnabled(COMP_ALGAE_INTAKE_POSITION_WRAPPING_ENABLED);
        // 2027 REVLib: pidf feedforward term -> velocity feedforward (kV).
        compAlgaeIntakeConfig.closedLoop.feedForward.kV(COMP_ALGAE_INTAKE_FeedForward);

        algaeIntakeSparkMax = new SparkMax(MotorIdConstants.CAN_BUS_ID, MotorIdConstants.ALGAE_BETA_INTAKE_CAN_ID, MotorType.kBrushless);
        algaeIntakeSparkMax.configure(compAlgaeIntakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        compAlgaeIntakeClosedLoop = algaeIntakeSparkMax.getClosedLoopController();
        compAlgaeIntakeRelativeEncoder = algaeIntakeSparkMax.getEncoder();
    }

    @Override
    public void setRollerSpeed(AngularVelocity speed) {
        compAlgaeIntakeClosedLoop.setSetpoint(speed.in(RPM), ControlType.kVelocity);
        goalVelocity = speed.in(RPM);
    }

    @Override
    public void intake() {
       setRollerSpeed(SpeedConstants.COMP_ALGAE_INTAKE_SPEED);
    }

    @Override
    public void outtake() {
        setRollerSpeed(SpeedConstants.COMP_ALGAE_OUTTAKE_SPEED);
    }

    @Override
    public void updateStates(AlgaeIntakeIOStates states) {
        states.intakeVelocity = getVelocity();
        states.leftAppliedVoltage = algaeIntakeSparkMax.getAppliedOutput().get()
                                * algaeIntakeSparkMax.getBusVoltage().get();
        states.leftCurrent = algaeIntakeSparkMax.getOutputCurrent().get();
        states.goalVelocity = goalVelocity;
    }

    private double getVelocity() {
        return compAlgaeIntakeRelativeEncoder.getVelocity().get();
    }

    @Override
        public boolean isStalling() {
                return algaeIntakeDebouncer.calculate((algaeIntakeSparkMax.getOutputCurrent().get() > ALGAE_INTAKE_STALL_THRESHOLD.in(Amps)) && (compAlgaeIntakeRelativeEncoder.getVelocity().get() < 0.1*ALGAE_INTAKE_STALL_VELOCITY.in(RPM)));
        }

    @Override
        public void passiveIntake() {
            // if (!isStalling()) {
            //     //compAlgaeIntakeClosedLoop.setSetpoint(SpeedConstants.COMP_ALGAE_PASSIVE_SPEED.in(RPM), ControlType.kVelocity);
            // }
            compAlgaeIntakeClosedLoop.setSetpoint(SpeedConstants.COMP_ALGAE_PASSIVE_SPEED.in(RPM), ControlType.kVelocity);
        }
}
