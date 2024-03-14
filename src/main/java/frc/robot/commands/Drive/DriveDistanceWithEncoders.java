// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import static frc.robot.Constants.DriveConstants.MAX_DRIVE_SPEED;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.SwerveModule;


public class DriveDistanceWithEncoders extends Command {
  /** Creates a new DriveDistanceWithEncoders. */

  private double distance;
  private SwerveDrive swerve;
  private ChassisSpeeds chassisSpeeds;

  //private final PIDController rotateController;

  private RelativeEncoder m_Encoder;
  private double startPosition;
  private double endPosition;
  private double currentPosition; 


  public DriveDistanceWithEncoders(SwerveDrive swerve, ChassisSpeeds chassisSpeeds,  double distance) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.distance = distance;
    this.swerve = swerve; 
    this.chassisSpeeds = chassisSpeeds;
  

    //rotateController = new PIDController(
     // 0,
      //0,
      //0
    //);

    addRequirements(swerve);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startPosition = swerve.leftGetter().getPosition();
    endPosition = startPosition + distance;
  

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentPosition = swerve.leftGetter().getPosition();

    ChassisSpeeds speed = new ChassisSpeeds(.5,0,0);
  
    swerve.drive(speed, MAX_DRIVE_SPEED);

    SmartDashboard.putNumber("Current Position", currentPosition);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    swerve.stopModules();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return -currentPosition >= endPosition;
  }
}
