package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoArmCmd extends CommandBase{
    private ArmSubsystem armSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private Timer timer;

    private double armPos;
    private double linearPos;
    private double wristPos;
    private double intakePower;
    private double wait;

    private boolean coneMode;

    public AutoArmCmd(ArmSubsystem armSubsystem, IntakeSubsystem intakeSubsystem, double armPos, double linearPos, double wristPos, double intakePower, boolean coneMode, double wait){
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.wristPos = wristPos;
        this.armPos = armPos;
        this.linearPos = linearPos;
        this.intakePower = intakePower;
        this.wait = wait;

        this.coneMode = coneMode;

        timer  = new Timer();

        addRequirements(armSubsystem);
        addRequirements(intakeSubsystem);
    }

    public AutoArmCmd(ArmSubsystem armSubsystem, double armPos, double linearPos, double wristPos,double wait){
        this.armSubsystem = armSubsystem;
        this.intakeSubsystem = null;
        this.wristPos = wristPos;
        this.armPos = armPos;
        this.linearPos = linearPos;
        this.intakePower = Integer.MAX_VALUE;
        this.wait = wait;

        this.coneMode = false;

        timer = new Timer();

        addRequirements(armSubsystem);
    }

    public AutoArmCmd(IntakeSubsystem intakeSubsystem, double intakePower, boolean coneMode, double wait){
        this.armSubsystem = null;
        this.intakeSubsystem = intakeSubsystem;
        this.wristPos = Integer.MAX_VALUE;
        this.armPos = Integer.MAX_VALUE;
        this.linearPos = Integer.MAX_VALUE;
        this.intakePower = intakePower;
        this.wait = wait;

        this.coneMode = coneMode;

        timer = new Timer();

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        timer.start();

        if (armPos != Integer.MAX_VALUE){
            armSubsystem.setArmPos(armPos, linearPos, wristPos);
        }

        if (intakePower != Integer.MAX_VALUE){
            intakeSubsystem.setIntakePower(coneMode, intakePower);
        }
    } 
    
    @Override
    public void execute() {
        if (armPos != Integer.MAX_VALUE){
            armSubsystem.updateArmPosPower();

            SmartDashboard.putNumber("arm auto", armPos);
            SmartDashboard.putNumber("linear auto", linearPos);
            SmartDashboard.putNumber("wrist auto", wristPos);
            
            SmartDashboard.putBoolean("finished", isFinished());
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        if (intakePower != Integer.MAX_VALUE){
            intakeSubsystem.setIntakePower(false, 0);
        }
    }

    @Override
    public boolean isFinished() {
        return (armPos != Integer.MAX_VALUE && armSubsystem.isFinished()) || timer.hasElapsed(wait);
    }
}