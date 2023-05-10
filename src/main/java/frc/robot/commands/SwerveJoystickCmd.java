package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;


public class SwerveJoystickCmd extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction, driveSpeedFunction, turnSpeedFunction;
    private final Supplier<Boolean> angleHoldFWDFunction, angleHoldBKWDFunction, angleHoldLFTFunction, angleHoldRGTFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    private PIDController headingPID = new PIDController(0.025, 1e-3, 0);

    private boolean fieldOriented;
    private double goalAngle;
    
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, Supplier<Boolean> angleHoldFWDFunction, Supplier<Boolean> angleHoldBKWDFunction, Supplier<Boolean> angleHoldLFTFunction, Supplier<Boolean> angleHoldRGTFunction, Supplier<Double> driveSpeedFunction, Supplier<Double> turnSpeedFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.angleHoldFWDFunction = angleHoldFWDFunction;
        this.angleHoldBKWDFunction = angleHoldBKWDFunction;
        this.angleHoldLFTFunction = angleHoldLFTFunction;
        this.angleHoldRGTFunction = angleHoldRGTFunction;
        this.driveSpeedFunction = driveSpeedFunction;
        this.turnSpeedFunction = turnSpeedFunction;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationForwardUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationSideUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        headingPID.enableContinuousInput(-180, 180);

        fieldOriented = true;
        goalAngle = 0;
    }

    @Override
    public void execute() {
        // 1. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed;
        if(angleHoldFWDFunction.get() || angleHoldBKWDFunction.get() || angleHoldLFTFunction.get() || angleHoldRGTFunction.get()){

            if (angleHoldFWDFunction.get()){
                goalAngle = 0;
            }else if (angleHoldBKWDFunction.get()){
                goalAngle = 180;
            }else if (angleHoldLFTFunction.get()){
                goalAngle = 270;
            }else if (angleHoldRGTFunction.get()){
                goalAngle = 90;
            }
            
            double angle = swerveSubsystem.getHeading();

            if (goalAngle > angle - 3 && goalAngle < angle + 3){
                turningSpeed = 0;
            }else {
                turningSpeed = headingPID.calculate(angle, goalAngle);
            }
        }else{
            turningSpeed = turningSpdFunction.get();
        }
        

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        double driveSpeedDial = (driveSpeedFunction.get() + 1) / 2;
        double turnSpeedDial = (turnSpeedFunction.get() + 1) / 2;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * driveSpeedDial;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond * driveSpeedDial;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond * turnSpeedDial;


        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOriented) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);

        SmartDashboard.putBoolean("field Oriented: ", fieldOriented);

        
        SmartDashboard.putNumber("expected turning speed: ", turningSpeed);
    }

    public double switchFeildOriented(){
        fieldOriented = !fieldOriented;
        return 0.0;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
