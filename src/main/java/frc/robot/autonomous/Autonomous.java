package frc.robot.autonomous;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Autonomous {
    private SwerveSubsystem swerveSubsystem;
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private AHRS gyro;

    SendableChooser<Integer> autoPicker;

    public Autonomous(SwerveSubsystem swerveSubsystem, ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, AHRS gyro){
        this.swerveSubsystem = swerveSubsystem;
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;

        this.gyro = gyro;

        autoPicker = new SendableChooser<Integer>();

        autoPicker.addOption("mobility only", 0);
        autoPicker.addOption("mid cube + mobility", 1);
        autoPicker.addOption("mid cube + balance", 2);
        autoPicker.addOption("high cube + mobility", 3);
        autoPicker.addOption("high cube + balance", 4);
        autoPicker.addOption("balance only", 5);
        autoPicker.addOption("mid cube only", 6);
        autoPicker.addOption("click if ur high", 7);

        autoPicker.setDefaultOption("mobility only", 0);

        SmartDashboard.putData(autoPicker);
    }

    public Command mobilityOnly(){
        return new SequentialCommandGroup(
            //reset position
            new InstantCommand(() -> swerveSubsystem.zeroHeading()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),

            new ParallelCommandGroup(
                new AutoDriveCmd(swerveSubsystem, -0.5, 0, 0.75),                // back up slightly
                new AutoArmCmd(armSubsystem, 60, 0, 0, 2)             // home arm
            ),
            new AutoDriveCmd(swerveSubsystem, -5.25, 0, 0.75),                      // back up for mobility
            new InstantCommand(() -> swerveSubsystem.setOffset(180))
        );
    }

    public Command midCubeOnly(){
        return new SequentialCommandGroup(
            //reset position
            new InstantCommand(() -> swerveSubsystem.zeroHeading()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
      
            new AutoArmCmd(armSubsystem, intakeSubsystem,30, 10, -10.5, 0, false, 2.75), // move to cube place mid
            new AutoArmCmd(intakeSubsystem, 1, false, 0.5),                 // outtake the cube
            new ParallelCommandGroup(
                new AutoDriveCmd(swerveSubsystem, -0.5, 0, 0.75),                // back up slightly
                new AutoArmCmd(armSubsystem, 60, 0, 0, 2)             // home arm
            ),
            new InstantCommand(() -> swerveSubsystem.setOffset(180))
        );
    }

    public Command highCubeOnly(){
        return new SequentialCommandGroup(
            //reset position
            new InstantCommand(() -> swerveSubsystem.zeroHeading()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
          
            new AutoArmCmd(armSubsystem, intakeSubsystem,18, 30, -10, 0, false, 4), // move to cube place high
            new AutoArmCmd(intakeSubsystem, 1, false, 0.5),              // outtake the cube
            new ParallelCommandGroup(
                new AutoDriveCmd(swerveSubsystem, -0.5, 0, 0.75),                // back up slightly
                new AutoArmCmd(armSubsystem, 60, 0, 0, 2)            // home arm
            ),
            new InstantCommand(() -> swerveSubsystem.setOffset(180))
        );
    }

    public Command midCubeMobility(){
        return new SequentialCommandGroup(
            //reset position
            new InstantCommand(() -> swerveSubsystem.zeroHeading()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
      
            new AutoArmCmd(armSubsystem, intakeSubsystem,30, 10, -10.5, 0, false, 2.75), // move to cube place mid
            new AutoArmCmd(intakeSubsystem, 1, false, 0.5),                 // outtake the cube
            new ParallelCommandGroup(
                new AutoDriveCmd(swerveSubsystem, -0.5, 0, 0.75),                // back up slightly
                new AutoArmCmd(armSubsystem, 60, 0, 0, 2)             // home arm
            ),
            new AutoDriveCmd(swerveSubsystem, -5.25, 0, 0.75),                      // back up for mobility
            new InstantCommand(() -> swerveSubsystem.setOffset(180))
        );
    }

    public Command midCubeBalance(){
        return new SequentialCommandGroup(
            //reset position
            new InstantCommand(() -> swerveSubsystem.zeroHeading()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
      
            new AutoArmCmd(armSubsystem, intakeSubsystem, 30, 10, -10.5, 0, false, 2.75), // move to cube place mid
            new AutoArmCmd(intakeSubsystem, 1, false, 0.5),                  // outtake the cube
            new ParallelCommandGroup(
                new AutoDriveCmd(swerveSubsystem, -0.5, 0, 0.75),                 // back up slightly
                new AutoArmCmd(armSubsystem, 60, 0, 0, 2)             // home arm
            ),
            new AutoBalanceCmd(swerveSubsystem, gyro),                                                  // balance
            new InstantCommand(() -> swerveSubsystem.setOffset(180))
        );
    }

    public Command midCubeMobilityBalance(){
        return new SequentialCommandGroup(
            //reset position
            new InstantCommand(() -> swerveSubsystem.zeroHeading()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
      
            new AutoArmCmd(armSubsystem, intakeSubsystem, 30, 10, -10.5, 0, false, 2.75), // move to cube place mid
            new AutoArmCmd(intakeSubsystem, 1, false, 0.5),              // outtake the cube
            new ParallelCommandGroup(
                new AutoDriveCmd(swerveSubsystem, -0.5, 0, 0.75),              // back up slightly
                new AutoArmCmd(armSubsystem, 60, 0, 0, 2)          // home arm
            ),
            new AutoDriveCmd(swerveSubsystem, -5.25, 0, 0.75),                    // back up over charge station
            new AutoDriveCmd(swerveSubsystem, -5.25, 0, 180),                     // spin around
            new AutoBalanceCmd(swerveSubsystem, gyro),                                               // balance
            new InstantCommand(() -> swerveSubsystem.setOffset(180))
        );
    }

    public Command highCubeMobility(){
        return new SequentialCommandGroup(
            //reset position
            new InstantCommand(() -> swerveSubsystem.zeroHeading()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
      
            new AutoArmCmd(armSubsystem, intakeSubsystem,18, 30, -10, 0, false, 4), // move to cube place high
            new AutoArmCmd(intakeSubsystem, 1, false, 0.5),              // outtake the cube
            new ParallelCommandGroup(
                new AutoDriveCmd(swerveSubsystem, -0.5, 0, 0.75),                // back up slightly
                new AutoArmCmd(armSubsystem, 60, 0, 0, 2)            // home arm
            ),
            new AutoDriveCmd(swerveSubsystem, -5.25, 0, 0.75),                       // back up for mobility
            new InstantCommand(() -> swerveSubsystem.setOffset(180))
        );
    }

    public Command highCubeBalance(){
        return new SequentialCommandGroup(
            //reset position
            new InstantCommand(() -> swerveSubsystem.zeroHeading()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
      
            new AutoArmCmd(armSubsystem, intakeSubsystem,18, 30, -10, 0, false, 4), // move to cube place high
            new AutoArmCmd(intakeSubsystem, 1, false, 0.5),               // outtake the cube
            new ParallelCommandGroup(
                new AutoDriveCmd(swerveSubsystem, -0.5, 0, 0.75),                // back up slightly
                new AutoArmCmd(armSubsystem, 60, 0, 0, 2)           // home arm
            ),
            new AutoBalanceCmd(swerveSubsystem, gyro),                                                 // balance
            new InstantCommand(() -> swerveSubsystem.setOffset(180))
        );
    }
    public Command highCubeMobilityBalance(){
        return new SequentialCommandGroup(
            //reset position
            new InstantCommand(() -> swerveSubsystem.zeroHeading()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
      
            new AutoArmCmd(armSubsystem, intakeSubsystem,18, 30, -10, 0, false, 4), // move to cube place high
            new AutoArmCmd(intakeSubsystem, 1, false, 0.5),             // outtake the cube
            new ParallelCommandGroup(
                new AutoDriveCmd(swerveSubsystem, -0.5, 0, 0.75),              // back up slightly
                new AutoArmCmd(armSubsystem, 60, 0, 0, 2)          // home arm
            ),
            new AutoDriveCmd(swerveSubsystem, -5.25, 0, 0.75),                    // back up over charge station
            new AutoDriveCmd(swerveSubsystem, -5.25 , 0, 180),                     // spin around
            new AutoBalanceCmd(swerveSubsystem, gyro),                                              // balance
            new InstantCommand(() -> swerveSubsystem.setOffset(180))
        );
    }
    
    public Command balanceOnly(){
        return new SequentialCommandGroup(
            //reset position
            new InstantCommand(() -> swerveSubsystem.zeroHeading()),
            new InstantCommand(() -> swerveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),

            new AutoArmCmd(armSubsystem, 60, 0, -0.2, 2),// home arm
            new AutoBalanceCmd(swerveSubsystem, gyro),                            // balance
            new InstantCommand(() -> swerveSubsystem.setOffset(180))
        );
    }

    public Command getSelected(){
        Command auto;
        switch(autoPicker.getSelected()){
            case 0: auto = mobilityOnly();
                    break;
            case 1: auto = midCubeMobility();
                    break;
            case 2: auto = midCubeBalance();
                    break;
            case 3: auto = highCubeMobility();
                    break;
            case 4: auto = highCubeBalance();
                    break;
            case 5: auto = balanceOnly();
                break;
            case 6: auto = midCubeOnly();
                    break;
            case 7: auto = highCubeOnly();
                    break;
            default:auto = mobilityOnly();
                    break;
        }
        return auto;
    }
}
