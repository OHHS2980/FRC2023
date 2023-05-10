// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoDriveCmd extends CommandBase {
  private SwerveSubsystem swerveSubsystem;

  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  private Timer timer = new Timer();

  private double targetX;
  private double targetY;
  private double targetAngle;

  private Pose2d current;

  private double speedX;
  private double speedY;
  private double speedAngle;

  private double distance;

  private PIDController Xpid = new PIDController(1, 0.5, 0);
  private PIDController Ypid = new PIDController(1, 0.5, 0);
  private PIDController Anglepid = new PIDController(0.1, 0, 0);

  /** Creates a new AutoDriveCmd. */
  public AutoDriveCmd(SwerveSubsystem swerveSubsystem, double targetX, double targetY, double targetAngle) {
    this.swerveSubsystem = swerveSubsystem;

    this.targetX = targetX;
    this.targetY = targetY;
    this.targetAngle = targetAngle;

    current = swerveSubsystem.getPose();

    distance = Math.abs(current.getX() - targetX) + Math.abs(current.getY() - targetY);

    Xpid.setSetpoint(targetX);
    Ypid.setSetpoint(targetY);
    Anglepid.enableContinuousInput(-180, 180);
    Anglepid.setSetpoint(targetAngle);

    Xpid.setTolerance(0.07, 0.2);
    Ypid.setTolerance(0.07, 0.2);
    Anglepid.setTolerance(0.07, 0.2);
    
    this.xLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    this.yLimiter = new SlewRateLimiter(AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    this.turningLimiter = new SlewRateLimiter(AutoConstants.kMaxAngularAccelerationRadiansPerSecondSquared);

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    current = swerveSubsystem.getPose();

    speedX = Xpid.calculate(current.getX());
    speedY = Ypid.calculate(current.getY());
    speedAngle = Anglepid.calculate(current.getRotation().getDegrees());

    
    speedX = xLimiter.calculate(speedX) * AutoConstants.kMaxSpeedMetersPerSecond;
    speedY = yLimiter.calculate(speedY) * AutoConstants.kMaxSpeedMetersPerSecond;
    speedAngle = turningLimiter.calculate(speedAngle) * AutoConstants.kMaxAngularSpeedRadiansPerSecond;


    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speedX, speedY, speedAngle, swerveSubsystem.getRotation2d());

    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    swerveSubsystem.setModuleStates(moduleStates);
    
    SmartDashboard.putString("position", current.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
    return (
      Math.abs(current.getX() - targetX) + 
      Math.abs(current.getY() - targetY) + 
      Math.abs(current.getRotation().getDegrees() - targetAngle) < 0.2) || 
      (Math.abs(speedX + speedY + speedAngle) < 0.05);
      */
      return (
        (
          Xpid.atSetpoint() && 
          Ypid.atSetpoint() &&
          Anglepid.atSetpoint()
        ) || 
        timer.hasElapsed(Math.max(distance, 3)));
  }
}
