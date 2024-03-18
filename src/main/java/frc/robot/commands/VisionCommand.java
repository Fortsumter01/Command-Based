// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class VisionCommand extends Command {
  /** Creates a new VisionCommand. */
  private VisionSubsystem visionSubsystem;
  private DriveSubsystem driveSubsystem;
  private double targetX;
  private double targetY;

  public VisionCommand(VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    this.visionSubsystem = visionSubsystem;
    this.driveSubsystem = driveSubsystem;
    addRequirements(visionSubsystem, driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    targetX = visionSubsystem.getAprilTagX();
    targetY = visionSubsystem.getAprilTagY();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double deltaX = targetX - visionSubsystem.getAprilTagX();
    double deltaY = targetY - visionSubsystem.getAprilTagY();

    //Change value to the robot speed from RobotConstants.
    //Turn speed could be another constant????
    double forwardSpeed = 0.5;
    double turnSpeed = deltaX * 0.1;

    driveSubsystem.arcadeDrive(forwardSpeed, turnSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}