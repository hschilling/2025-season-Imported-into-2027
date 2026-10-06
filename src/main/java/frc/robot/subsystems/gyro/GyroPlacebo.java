package frc.robot.subsystems.gyro;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.StatusSignal;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public class GyroPlacebo implements GyroIO {
    public Angle getYaw() {
        return Degrees.of(0.0);
    }

    public Angle getYaw(boolean refresh) {
        return Degrees.of(0);
    }

    public void setYaw(Angle yaw) {
    }

    public StatusSignal<AngularVelocity> getAngularVelocity() {
        return new StatusSignal<AngularVelocity>(null, null, null);
    }

    public boolean hasFault() {
        return false;
    }
}
