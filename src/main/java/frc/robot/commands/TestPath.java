// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestPath extends SequentialCommandGroup {
	private DriveTrain driveTrain;
	private Object ramseteCommand;

	/** Creates a new TestPath. */
	public TestPath(DriveTrain dt) {
		driveTrain = dt;
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());

		addCommands(RamseteCommandGenerator.generate(driveTrain, "paths/testPath.wpilib.json"));
	}

	private Command RamseteCommandGenerator() {
		return null;
	}
}
