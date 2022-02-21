// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Hood extends PIDSubsystem {
	private CANSparkMax motor = hoodMotor(10, false);
	private AnalogInput potentiometer = new AnalogInput(1);

	public CANSparkMax hoodMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kCoast);
		sparkMax.getForwardLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(true);
		sparkMax.getReverseLimitSwitch(Type.kNormallyClosed).enableLimitSwitch(true);
		sparkMax.burnFlash();
		return sparkMax;
	}

	/** Creates a new Hood. */
	public Hood() {
		super(
				// The PIDController used by the subsystem
				new PIDController(0, 0, 0));
	}

	@Override
	public void useOutput(double output, double setpoint) {
		// Use the output here
		motor.set(output);
	}

	public void stop() {
		motor.stopMotor();
		this.disable();
	}

	@Override
	public double getMeasurement() {
		// Return the process variable measurement here
		return potentiometer.getVoltage();
	}
}
