// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingAmpPreSequence extends SequentialCommandGroup {
  /** Creates a new ShootingAmpSequence. */
  public ShootingAmpPreSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Spin the shooter and raise arm
     
      new ShooterToPower(RobotContainer.gpmHelpers.getShooterPowerTouchingAmp()).
        alongWith(
            // Arm to angle
            new ArmTurnToAngle(() -> RobotContainer.gpmHelpers.getAngleBeforeTouchingAmp())
          )
    );
  }
}
