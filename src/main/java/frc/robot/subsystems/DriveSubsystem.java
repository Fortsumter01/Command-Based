// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
 
 private CANSparkMax leftFrontMotor = new CANSparkMax(0, MotorType.kBrushless);
 private CANSparkMax leftBackMotor = new CANSparkMax(1, MotorType.kBrushless);
 
 private CANSparkMax rightFrontMotor = new CANSparkMax(2, MotorType.kBrushless);
 private CANSparkMax rightBackMotor = new CANSparkMax(3, MotorType.kBrushless);
 
 private RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();
 private RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
 
 private DifferentialDrive m_Drive = 
     new DifferentialDrive(leftFrontMotor::set, rightFrontMotor::set);

     // The gyro sensor
 private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();
 
 
 public DriveSubsystem() {

    leftFrontMotor.restoreFactoryDefaults();
    leftBackMotor.restoreFactoryDefaults();
    rightFrontMotor.restoreFactoryDefaults();
    rightBackMotor.restoreFactoryDefaults();

    SendableRegistry.addChild(m_Drive, leftFrontMotor);
    SendableRegistry.addChild(m_Drive, rightFrontMotor);
    
    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightBackMotor);

    leftFrontMotor.setInverted(false);
    rightFrontMotor.setInverted(true);

    ((Encoder) leftEncoder).setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    
    ((Encoder) rightEncoder).setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("right encoder", rightFrontEncoderValue());
    SmartDashboard.putNumber("lleft encoder", leftFrontEncoderValue());
  }

  public double leftFrontEncoderValue(){
    return leftEncoder.getPosition();
  }

  public double rightFrontEncoderValue(){
    return rightEncoder.getPosition();
  }

  public void arcadeDrive(double fwd, double rot){
    m_Drive.arcadeDrive(fwd, rot);
  }

  public void resetEncoders() {
    ((Encoder) leftEncoder).reset();
    ((Encoder) rightEncoder).reset();
  }

  public double getAverageEncoderDistance() {
    return (((Encoder) leftEncoder).getDistance() + ((Encoder) rightEncoder).getDistance()) / 2.0;
  }

  public Encoder getleftEncoder() {
    return (Encoder) leftEncoder;
  }

  public Encoder getrigthEncoder() {
    return (Encoder) rightEncoder;
  }

  public void setMaxOutput(double maxOutput) {
     m_Drive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(),360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}