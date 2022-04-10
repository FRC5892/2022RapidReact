// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
	private CANSparkMax climbMotor(int motorID, boolean inverted) {
		CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushed);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(inverted);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.getEncoder(Type.kNoSensor, 1);
		return sparkMax;
	}

	private CANSparkMax leftMotor = climbMotor(Constants.CLIMB_MOTOR_PORTS[0], false);
	private CANSparkMax rightMotor = climbMotor(Constants.CLIMB_MOTOR_PORTS[1], false);
	private DoubleSolenoid actuationSolenoid;

	boolean actuated = false;

	
	/** Creates a new Climb. */
	public Climb() {
		actuationSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,
				Constants.CLIMB_ACTUATION_SOLENOID_PORTS[0], Constants.CLIMB_ACTUATION_SOLENOID_PORTS[1]);
		actuationSolenoid.set(Value.kReverse);
	}

	public void driveArms(double leftSpeed, double rightSpeed) {
		leftMotor.set(leftSpeed);
		rightMotor.set(rightSpeed);
	}

	public void togglePistons(){
		if (actuated){
			actuationSolenoid.set(Value.kReverse);
			actuated = false;
		}
		else{
			actuationSolenoid.set(Value.kForward);
			actuated=true;
		}
	}

	public void pistonForward() {
		actuationSolenoid.set(Value.kForward);
	}

	public void pistonReverse() {
		actuationSolenoid.set(Value.kReverse);
	}

	public void stopMotors() {
		leftMotor.stopMotor();
		rightMotor.stopMotor();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putData("Climb Solenoid", actuationSolenoid);
	}
}
