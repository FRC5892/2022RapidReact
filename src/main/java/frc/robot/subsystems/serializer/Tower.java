// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.serializer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Tower extends SubsystemBase {
	private CANSparkMax towerMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushed);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.burnFlash();
		return sparkMax;
	}

	private CANSparkMax towerMotors = towerMotor(Constants.TOWER_MOTOR_PORT, false);
	private DigitalInput ballSensor = new DigitalInput(Constants.TOWER_SENSOR_PORT);

	/** Creates a new Tower. */
	public Tower() {
		// template
	}

	public void setMotors(double speed) {
		towerMotors.set(speed);
	}

	public void stopMotors() {
		towerMotors.stopMotor();
	}

	public boolean hasBall() {
		return ballSensor.get();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putBoolean("Tower Sensor", hasBall());
	}
}
