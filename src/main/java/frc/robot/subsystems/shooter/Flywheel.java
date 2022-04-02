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
    sparkMax.burnFlash();
    return sparkMax;
  }

  private CANSparkMax leftMotor = FlywheelMotor(Constants.FLYWHEEL_MOTOR_IDS[0], false);
  private CANSparkMax rightMotor = FlywheelMotor(Constants.FLYWHEEL_MOTOR_IDS[1], true);
  private RelativeEncoder neoEncoderLeft = leftMotor.getEncoder();
  // private RelativeEncoder neoEncoderRight = rightMotor.getEncoder();
  private SparkMaxPIDController pidController = leftMotor.getPIDController();
  // private SparkMaxPIDController pidControllerRight = rightMotor.getPIDController();
  private double setPoint;

  public Flywheel() {
    //rightMotor.follow(leftMotor, true);
    pidController.setP(Constants.FLYWHEEL_SPARKMAX_PID[0]);
    pidController.setI(Constants.FLYWHEEL_SPARKMAX_PID[1]);
    pidController.setD(Constants.FLYWHEEL_SPARKMAX_PID[2]);
    // pidControllerRight.setP(Constants.FLYWHEEL_SPARKMAX_PID[0]);
    // pidControllerRight.setI(Constants.FLYWHEEL_SPARKMAX_PID[1]);
    // pidControllerRight.setD(Constants.FLYWHEEL_SPARKMAX_PID[2]);
    //neoEncoder.setVelocityConversionFactor(Constants.FLYWHEEL_ENCODER_CONVERSION_FACTOR);
    //leftMotor.burnFlash();
    rightMotor.follow(leftMotor, true);
    SmartDashboard.putNumber("Flywheel P", Constants.FLYWHEEL_SPARKMAX_PID[0]);
		SmartDashboard.putNumber("Flywheel I", Constants.FLYWHEEL_SPARKMAX_PID[1]);
		SmartDashboard.putNumber("Flywheel D", Constants.FLYWHEEL_SPARKMAX_PID[2]);
  }

  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void setSetpoint(double setpoint) {
    setPoint = setpoint;
		pidController.setReference(setpoint, ControlType.kVelocity);
    // pidControllerRight.setReference(setpoint, ControlType.kVelocity);
	}

  public double getVelocity(){
    return neoEncoderLeft.getVelocity();
  }

	public boolean atSetpoint() {
		return (Math.abs(setPoint - neoEncoderLeft.getVelocity()) <= 50);
	}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Flywheel Velocity", neoEncoderLeft.getVelocity());
		SmartDashboard.putBoolean("Flywheel At Setpoint", this.atSetpoint());
    // SmartDashboard.putNumber("Flywheel Setpoint", setPoint);

		pidController.setP(SmartDashboard.getNumber("Flywheel P", Constants.FLYWHEEL_SPARKMAX_PID[0]));
		pidController.setI(SmartDashboard.getNumber("Flywheel I", Constants.FLYWHEEL_SPARKMAX_PID[1]));
		pidController.setD(SmartDashboard.getNumber("Flywheel D", Constants.FLYWHEEL_SPARKMAX_PID[2]));
    // pidControllerRight.setP(SmartDashboard.getNumber("Flywheel P", Constants.FLYWHEEL_SPARKMAX_PID[0]));
		// pidControllerRight.setI(SmartDashboard.getNumber("Flywheel I", Constants.FLYWHEEL_SPARKMAX_PID[1]));
		// pidControllerRight.setD(SmartDashboard.getNumber("Flywheel D", Constants.FLYWHEEL_SPARKMAX_PID[2]));
    
    // This method will be called once per scheduler run
  }
}
