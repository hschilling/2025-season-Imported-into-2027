// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.gyro;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import org.wpilib.units.measure.Angle;
import frc.robot.Constants;

/** IO implementation for Pigeon2 */
public class GyroHardware implements GyroIO {
    private final Pigeon2 pigeon;

    public GyroHardware() {
        // 2027 Phoenix6: the CAN bus is now a CANBus object, not a String.
        // TODO(SystemCore): confirm the CAN bus name once the control system is wired.
        pigeon = new Pigeon2(Constants.MotorIdConstants.GYRO_CAN_ID, new CANBus("rio"));
        // these three lines disable all status signal readings off the pigeon except
        // for the three we need (the three directly below). this reduces CAN network
        // utilization
        pigeon.getYaw().setUpdateFrequency(Constants.SpeedConstants.MAIN_LOOP_FREQUENCY_HZ);
        pigeon.getAngularVelocityZDevice().setUpdateFrequency(Constants.SpeedConstants.MAIN_LOOP_FREQUENCY_HZ);
        pigeon.getFault_Hardware().setUpdateFrequency(Constants.SpeedConstants.MAIN_LOOP_FREQUENCY_HZ);
        pigeon.optimizeBusUtilization();
    }

    public Angle getYaw() {
        // Don't refresh the status signal by default, we already get it at
        // MAIN_LOOP_FREQUENCY_HZ, and refreshing blocks, causing loop overruns
        return this.getYaw(true);
    }

    public Angle getYaw(boolean refresh) {
        return Degrees.of(pigeon.getYaw(refresh).getValueAsDouble());
    }

    public void setYaw(Angle yaw) {
        pigeon.setYaw(yaw.in(Degrees));
    }

    public StatusSignal<org.wpilib.units.measure.AngularVelocity> getAngularVelocity() {
        // Don't refresh the status signal by default, we already get it at
        // MAIN_LOOP_FREQUENCY_HZ, and refreshing blocks, causing loop overruns
        return this.getAngularVelocity(false);
    }

    public StatusSignal<org.wpilib.units.measure.AngularVelocity> getAngularVelocity(boolean refresh) {
        return pigeon.getAngularVelocityZDevice(refresh);
    }

    public boolean hasFault() {
        // Don't refresh the status signal by default, we already get it at
        // MAIN_LOOP_FREQUENCY_HZ, and refreshing blocks, causing loop overruns
        return this.hasFault(false);
    }

    public boolean hasFault(boolean refresh) {
        return pigeon.getFault_Hardware(refresh).getValue();
    }
}