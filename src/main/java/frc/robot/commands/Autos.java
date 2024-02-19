// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.LauncherConstants.kLauncherDelay;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import frc.robot.subsystems.PWMDrivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.CANDrivetrain;
import frc.robot.subsystems.CANLauncher;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(CANDrivetrain drivetrain) {
    /**
     * RunCommand is a helper class that creates a command from a single method, in this case we
     * pass it the arcadeDrive method to drive straight back at half power. We modify that command
     * with the .withTimeout(1) decorator to timeout after 1 second, and use the .andThen decorator
     * to stop the drivetrain after the first command times out
     */
    return new RunCommand(() -> drivetrain.arcadeDrive(-.5, 0))
        .withTimeout(1)
        .andThen(new RunCommand(() -> drivetrain.arcadeDrive(0, 0)));
  }

  public static Command autoAdjustable(CANDrivetrain drivetrain, CANLauncher launcher){
    double side =  SmartDashboard.getNumber("Auto Side", 1);

    return new SequentialCommandGroup(
      //adjustable delay before starting next step
      new WaitCommand(SmartDashboard.getNumber("Auto Delay", 0)),

      //Shoot
      new SequentialCommandGroup(
        new WaitCommand(SmartDashboard.getNumber("Auto Shoot Delay", 0)),

        new RunCommand(() -> drivetrain.arcadeDrive(-.8, 0), drivetrain).withTimeout(.2),
        new RunCommand(() -> drivetrain.arcadeDrive(.7, 0), drivetrain).withTimeout(.2),
        new RunCommand(() -> drivetrain.arcadeDrive(0, 0), drivetrain).withTimeout(.5),
        //due to uneven shooting, turning should not invert when aiming into goal
        new RunCommand(() -> drivetrain.arcadeDrive(0, (.55)), drivetrain).withTimeout(.62),

        new PrepareLaunch(launcher)
                  .withTimeout(2)
                  .andThen(new LaunchNote(launcher).withTimeout(2))
                  .handleInterrupt(() -> launcher.stop()),

        new RunCommand(() -> launcher.stop(), launcher).withTimeout(.06),

        new RunCommand(() -> drivetrain.arcadeDrive(-.55, 0), drivetrain).withTimeout(.2),
        new RunCommand(() -> drivetrain.arcadeDrive(0, 0), drivetrain).withTimeout(.1),
        new RunCommand(() -> drivetrain.arcadeDrive(0, (-.7 * side)), drivetrain).withTimeout(1),
        new RunCommand(() -> drivetrain.arcadeDrive(0, 0), drivetrain).withTimeout(.1)
      ).onlyIf(() -> SmartDashboard.getBoolean("Auto Shoot", false)),

      //Cross Line
      new SequentialCommandGroup(
        new WaitCommand(SmartDashboard.getNumber("Auto Cross Line Delay", 0)),

        new RunCommand(() -> drivetrain.arcadeDrive(.4, 0), drivetrain).withTimeout(3),
        new RunCommand(() -> drivetrain.arcadeDrive(0, 0), drivetrain).withTimeout(.1)
      ).onlyIf(() -> SmartDashboard.getBoolean("Auto Cross Line", false))
  );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
