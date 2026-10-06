package frc.robot.subsystems.drive;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import frc.robot.subsystems.drive.SwerveModuleIO.SwerveModuleIOStates;

public class SwerveModule {

    private SwerveModuleVelocity desiredState = new SwerveModuleVelocity(0.0, new Rotation2d());
    private SwerveModuleIOStates states = new SwerveModuleIOStates();

    private SwerveModuleIO io;

    public SwerveModule(SwerveModuleIO io) {
        this.io = io;
        io.setDriveEncoderPosition(0);
        desiredState.angle = new Rotation2d(getTurnEncoderPosition());
    }

    public double getDriveEncoderSpeedMPS() {
        return io.getDriveEncoderSpeedMPS();
    }

    public double getTurnEncoderPosition() {
        return io.getTurnEncoderPosition();
    }

    public double getDriveEncoderPosition() {
        return io.getDriveEncoderPosition();
    }

    public void resetEncoders() {
        io.setDriveEncoderPosition(0);
    }

    public void updateStates() {
        io.updateStates(states);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleVelocity getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModuleVelocity(getDriveEncoderSpeedMPS(),
                new Rotation2d((getTurnEncoderPosition()) - io.getChassisAngularOffset()));
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        return new SwerveModulePosition(
                getDriveEncoderPosition(),
                new Rotation2d(getTurnEncoderPosition() - io.getChassisAngularOffset()));
    }

    /**
     * Sets the desired state for the module.
     *
     * @param newDesiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleVelocity newDesiredState) {
        // Apply chassis angular offset to the desired state.
        SwerveModuleVelocity correctedDesiredState = new SwerveModuleVelocity();
        correctedDesiredState.velocity = newDesiredState.velocity;
        correctedDesiredState.angle = newDesiredState.angle.plus(Rotation2d.fromRadians(io.getChassisAngularOffset()));

        // Optimize the reference state to avoid spinning further than 90 degrees.

        correctedDesiredState = correctedDesiredState.optimize(new Rotation2d(getTurnEncoderPosition()));
        io.setDesiredDriveSpeedMPS(correctedDesiredState.velocity);
        io.setDesiredTurnAngle(correctedDesiredState.angle.getRadians());
        desiredState = newDesiredState; // TODO: this seems weird
    }

}