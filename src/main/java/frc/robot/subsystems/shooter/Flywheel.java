// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Flywheel extends SubsystemBase {
  /** Creates a new FlywheelNew. */
  public CANSparkMax FlywheelMotor(int motorID, boolean inverted) {
    CANSparkMax sparkMax = new CANSparkMax(motorID, MotorType.kBrushless);
    sparkMax.restoreFactoryDefaults();
    sparkMax.setInverted(inverted);
    sparkMax.setIdleMode(IdleMode.kCoast);
    return sparkMax;
  }

  private CANSparkMax leftMotor;
  private CANSparkMax rightMotor;
  private RelativeEncoder neoEncoderLeft;
  private SparkMaxPIDController pidController;
  private double setPoint;

  public Flywheel() {
	leftMotor = FlywheelMotor(Constants.FLYWHEEL_MOTOR_IDS[0], false);
	rightMotor = FlywheelMotor(Constants.FLYWHEEL_MOTOR_IDS[1], true);
	neoEncoderLeft = leftMotor.getEncoder();
	neoEncoderLeft.setVelocityConversionFactor(36d/48d);
	pidController = leftMotor.getPIDController();
	pidController.setP(Constants.FLYWHEEL_SPARKMAX_PIDF[0]);
    pidController.setI(Constants.FLYWHEEL_SPARKMAX_PIDF[1]);
    pidController.setD(Constants.FLYWHEEL_SPARKMAX_PIDF[2]);
	pidController.setFF(Constants.FLYWHEEL_SPARKMAX_PIDF[3]);
	pidController.setOutputRange(0, 1);
	leftMotor.burnFlash();
    rightMotor.follow(leftMotor, true);
    SmartDashboard.putNumber("Flywheel P", Constants.FLYWHEEL_SPARKMAX_PIDF[0]);
	SmartDashboard.putNumber("Flywheel I", Constants.FLYWHEEL_SPARKMAX_PIDF[1]);
	SmartDashboard.putNumber("Flywheel D", Constants.FLYWHEEL_SPARKMAX_PIDF[2]);

  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void setSetpoint(double setpoint) {
    setPoint = setpoint;
	pidController.setReference(setpoint, ControlType.kVelocity);
	System.out.println(setpoint);
	}

	public double getVelocity(){
		return neoEncoderLeft.getVelocity();
	}

	public boolean atSetpoint() {
		return (Math.abs(setPoint - neoEncoderLeft.getVelocity()) <= 15);
	}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Velocity", neoEncoderLeft.getVelocity());
		SmartDashboard.putBoolean("Flywheel At Setpoint", this.atSetpoint());
  }
}
