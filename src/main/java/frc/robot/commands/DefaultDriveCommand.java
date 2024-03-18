// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.DriveSubsystem;

// public class DefaultDriveCommand extends Command {
//   /** Creates a new DefaultDriveCommand. */
 
//  private DriveSubsystem DRIVE_SUBSYSTEM;
//  private Joystick DRIVE_CONTROLLER;
 
//   double right;
//   double left;
 
//   public DefaultDriveCommand(DriveSubsystem drive, Joystick controller) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.DRIVE_SUBSYSTEM = drive;
//     this.DRIVE_CONTROLLER = controller;  
//     addRequirements(DRIVE_SUBSYSTEM);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     right = 0;
//     left = 0;
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     right = DRIVE_CONTROLLER.getRawAxis(1);
//     left = DRIVE_CONTROLLER.getRawAxis(5);  

//     DRIVE_SUBSYSTEM.set(left, right);
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
