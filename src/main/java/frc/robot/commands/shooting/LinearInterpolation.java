// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooting;

/** Add your docs here. */
public class LinearInterpolation {
    public static double calculate(double[] xCoords, double[] yCoords, double value) {
        int index = 1;

        for (int i = 0; i < xCoords.length; i++) {
            if (xCoords[i] > value) {
                index = i;
                break;
            }
        }

        double slope = (yCoords[index] - yCoords[index - 1]) / (xCoords[index] - xCoords[index - 1]);
        double intercept = yCoords[index];
        double deltaValue = xCoords[index] - value;
        System.out.println("Slope " + slope);
        System.out.println("Intercept " + intercept);
        System.out.println("Value" + value);
        return deltaValue * slope + intercept;    
    }
}
