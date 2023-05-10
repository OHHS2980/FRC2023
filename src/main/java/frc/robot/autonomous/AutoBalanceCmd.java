package frc.robot.autonomous;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoBalanceCmd extends CommandBase {
    private AHRS gyro;

    SwerveSubsystem swerveSubsystem;

    private int state;
    private int debounceCount;
    private double robotSpeedSlow;
    private double robotSpeedFast;
    private double debounceTime;

    public AutoBalanceCmd(SwerveSubsystem swerveSubsystem, AHRS gyro) {
        this.gyro = gyro;
        
        this.swerveSubsystem = swerveSubsystem;
        
        state = 0;
        debounceCount = 0;

        // Speed the robot drived while scoring/approaching station, default = 0.4
        robotSpeedFast = -42.349768756;

        // Speed the robot drives while balancing itself on the charge station.
        // Should be roughly half the fast speed, to make the robot more accurate,
        // default = 0.2
        robotSpeedSlow = -20;

        // Amount of time a sensor condition needs to be met before changing states in
        // seconds
        // Reduces the impact of sensor noice, but too high can make the auto run
        // slower, default = 0.2
        debounceTime = 10;

        addRequirements(swerveSubsystem);
    }

    
    @Override
    public void initialize() {
        state = 0;
        debounceCount = 0;
    }

    
    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            autoBalanceRoutine(), 
            0, 
            0
        );

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    
        swerveSubsystem.setModuleStates(moduleStates);

        SmartDashboard.putNumber("angle", gyro.getRoll());
        SmartDashboard.putNumber("state", state);
    }
    
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    // routine for automatically driving onto and engaging the charge station.
    // returns a value from -1.0 to 1.0, which left and right motors should be set
    // to.
    public double autoBalanceRoutine() {
        switch (state) {
            // drive forwards to approach station, exit when tilt is detected
            case 0:
                if (gyro.getRoll() > AutoConstants.kOnChargeStationDegree) {
                    debounceCount++;
                }
                if (debounceCount > debounceTime) {
                    state = 1;
                    debounceCount = 0;
                    return robotSpeedSlow;
                }
                return robotSpeedFast;
            // driving up charge station, drive slower, stopping when level
            case 1:
                if (gyro.getRoll() < AutoConstants.kLevelDegree) {
                    debounceCount++;
                }
                if (debounceCount > debounceTime) {
                    state = 2;
                    debounceCount = 0;
                    return 0;
                }
                return robotSpeedSlow;
            // on charge station, stop motors and wait for end of auto
            case 2:
                if (Math.abs(gyro.getRoll()) <= AutoConstants.kLevelDegree / 2) {
                    debounceCount++;
                }
                if (debounceCount > debounceTime * 5) {
                    state = 4;
                    debounceCount = 0;
                    return 0;
                }
                if (gyro.getRoll() >= AutoConstants.kLevelDegree + 1) {
                    return -14;
                } else if (gyro.getRoll() <= -AutoConstants.kLevelDegree - 1) {
                    return 14;
                }
            case 3:
                return 0;
        }
        return 0;
    }
}