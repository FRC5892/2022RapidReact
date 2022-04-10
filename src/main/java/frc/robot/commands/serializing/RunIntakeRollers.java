// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.serializing;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.serializer.Intake;
import frc.robot.subsystems.serializer.Kicker;
import frc.robot.subsystems.serializer.Tower;

public class RunIntakeRollers extends CommandBase {
	private Intake intake;
	private Tower tower;
	private Kicker kicker;

	/** Creates a new RunIntakeRollers. */
	public RunIntakeRollers(Intake i, Tower t, Kicker k) {
		// Use addRequirements() here to declare subsystem dependencies.
		intake = i;
		tower = t;
		kicker = k;
		addRequirements(intake);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		// default
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if (intake.returnPistons() == Value.kForward){
			if (OperatorInput.driverJoystick.getLeftTriggerAxis() > 0) {
				// outake
				intake.setMotors(0);
			}
			else if (OperatorInput.driverJoystick.getRightTriggerAxis() > 0.02) {
				// intake
				intake.setMotors(-OperatorInput.driverJoystick.getRightTriggerAxis() * Constants.INTAKE_SPEED_MULTIPLIER);
			}

			else if (!tower.hasBall() || !kicker.hasBall()){
				intake.setMotors(Constants.INTAKE_SPEED_MULTIPLIER);
			}
			else{
				intake.setMotors(0);
			}
		}
		else{
			intake.setMotors(0);
		}
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		intake.stopMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
