// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.StatusSignal;

import org.wpilib.units.measure.Angle;

public interface GyroIO {

    public Angle getYaw();

    public Angle getYaw(boolean refresh);

    public void setYaw(Angle yaw);

    public StatusSignal<org.wpilib.units.measure.AngularVelocity> getAngularVelocity();

    public boolean hasFault();

}