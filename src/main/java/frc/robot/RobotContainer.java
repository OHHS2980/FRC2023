package frc.robot;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.autonomous.AutoArmCmd;
import frc.robot.autonomous.AutoBalanceCmd;
import frc.robot.autonomous.Autonomous;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.I2C;


public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem;
  private final ArmSubsystem armSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  
  private SwerveJoystickCmd swerveJoystickCmd;
  private IntakeCmd intakeCmd;
  private ArmCmd armCmd;

  private Autonomous autonomous;

  private final Joystick driveJoystick = new Joystick(OIConstants.kDriveControllerPort);
  private final Joystick turnJoystick = new Joystick(OIConstants.kTurnControllerPort);
  private final Joystick operatorJoystick = new Joystick(OIConstants.kOperatorControllerPort);

  private AHRS gyro = new AHRS(I2C.Port.kOnboard);

  public RobotContainer() {
    swerveSubsystem = new SwerveSubsystem(gyro);
    intakeSubsystem = new IntakeSubsystem();
    armSubsystem = new ArmSubsystem();
  
    swerveJoystickCmd = new SwerveJoystickCmd(
                swerveSubsystem,
                () -> -driveJoystick.getRawAxis(OIConstants.kDriverYAxis),
                () -> driveJoystick.getRawAxis(OIConstants.kDriverXAxis),
                () -> turnJoystick.getRawAxis(OIConstants.kDriverRotAxis),
                () -> driveJoystick.getRawButton(OIConstants.kAngleHoldFWDButtonPort),
                () -> driveJoystick.getRawButton(OIConstants.kAngleHoldBKWDButtonPort),
                () -> driveJoystick.getRawButton(OIConstants.kAngleHoldLFTButtonPort),
                () -> driveJoystick.getRawButton(OIConstants.kAngleHoldRGTButtonPort),
                () -> -driveJoystick.getRawAxis(OIConstants.kDriveSpeedAxis),
                () -> -turnJoystick.getRawAxis(OIConstants.kTurnSpeedAxis));

    intakeCmd = new IntakeCmd(intakeSubsystem,
                () -> driveJoystick.getRawAxis(IntakeConstants.kIntakeSliderPort),
                () -> operatorJoystick.getRawButtonPressed(ArmConstants.kCubeConeToggleButtonPort),
                () -> driveJoystick.getRawButton(IntakeConstants.kIntakeTriggerPort),
                () -> turnJoystick.getRawButton(IntakeConstants.kIntakeTriggerPort));
    
    armCmd = new ArmCmd(armSubsystem, gyro,
                () -> operatorJoystick.getRawAxis(ArmConstants.kArmSliderPort),
                () -> operatorJoystick.getRawAxis(ArmConstants.kLinearSliderPort),
                () -> operatorJoystick.getRawAxis(ArmConstants.kWristSliderPort),
                () -> operatorJoystick.getRawButton(ArmConstants.kHighButtonPort),
                () -> operatorJoystick.getRawButton(ArmConstants.kMidButtonPort),
                () -> operatorJoystick.getRawButton(ArmConstants.kLowButtonPort),
                () -> operatorJoystick.getPOV(),
                () -> operatorJoystick.getRawButton(ArmConstants.kResetPosButtonPort),
                () -> operatorJoystick.getRawButtonPressed(ArmConstants.kIntakePlaceToggleButtonPort),
                () -> operatorJoystick.getRawButtonPressed(ArmConstants.kFrontBackToggleButtonPort),
                () -> operatorJoystick.getRawButton(ArmConstants.kArmResetButtonPort),
                () -> operatorJoystick.getRawButtonPressed(ArmConstants.kManualOverrideButtonPort),
                () -> swerveJoystickCmd.switchFeildOriented());

    swerveSubsystem.setDefaultCommand(swerveJoystickCmd);

    intakeSubsystem.setDefaultCommand(intakeCmd);

    armSubsystem.setDefaultCommand(armCmd);

    autonomous = new Autonomous(swerveSubsystem, armSubsystem, intakeSubsystem, gyro);
    
    configureBindings();

    CameraServer.startAutomaticCapture(0);
  }

  private void configureBindings() {
    new JoystickButton(turnJoystick, OIConstants.kResetGyroButtonPort).onTrue(new InstantCommand(() -> swerveSubsystem.zeroHeading()));
    new JoystickButton(turnJoystick, OIConstants.kConeIntakeButtonPort).onTrue(new InstantCommand(() -> armCmd.setConeIntake()));
    new JoystickButton(turnJoystick, OIConstants.kCubeIntakeButtonPort).onTrue(new InstantCommand(() -> armCmd.setCubeIntake()));
    new JoystickButton(turnJoystick, OIConstants.kConeIntakeButtonPort).onFalse(new InstantCommand(() -> armCmd.resetRelease()));
    new JoystickButton(turnJoystick, OIConstants.kCubeIntakeButtonPort).onFalse(new InstantCommand(() -> armCmd.resetRelease()));
    new JoystickButton(turnJoystick, OIConstants.kDriverFieldOrientedButtonPort).onTrue(new InstantCommand(() -> swerveJoystickCmd.switchFeildOriented()));
  }

  public Command getAutonomousCommand() {
    /*
    return new SequentialCommandGroup(
            //reset position
            new InstantCommand(() -> swerveSubsystem.zeroHeading()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),

            new AutoArmCmd(armSubsystem, 60, 0, -0.2, 2),// home arm
            new AutoBalanceCmd(swerveSubsystem, gyro)                                  // balance
        );
        */
    return autonomous.getSelected();
  }
}
