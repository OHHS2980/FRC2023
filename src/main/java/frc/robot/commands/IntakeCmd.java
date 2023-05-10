package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class IntakeCmd extends CommandBase{
    private final IntakeSubsystem intakeSubsystem; 

    private static boolean coneMode = false;

    private Supplier<Double> intakeSlider;

    private Supplier<Boolean> coneCubeToggleButton;
    private Supplier<Boolean> intakeTrigger;
    private Supplier<Boolean> outtakeTrigger;

    public IntakeCmd(IntakeSubsystem intakeSubsystem,
            Supplier<Double> rampSliderSupplier,
            Supplier<Boolean> coneCubeToggleButtonSupplier,
            Supplier<Boolean> intakeTriggerSupplier,
            Supplier<Boolean> outtakeTriggerSupplier) {

        this.intakeSlider = rampSliderSupplier;
        this.coneCubeToggleButton = coneCubeToggleButtonSupplier;
        this.intakeTrigger = intakeTriggerSupplier;
        this.outtakeTrigger = outtakeTriggerSupplier;

        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }


    public static boolean getConeMode(){
        return coneMode;
    }

    public static void setConeMode(boolean cone){
        coneMode = cone;
    }

    @Override
    public void initialize() {

        SmartDashboard.putBoolean("coneMode: ", coneMode);
    }
    
    @Override
    public void execute() {
        if(coneCubeToggleButton.get()){
            coneMode = !coneMode;
        }
        double power = 0;
        if (intakeTrigger.get()){
            power = 1;
        }
        if (outtakeTrigger.get()){
            power = -1;
        }

        //intakeSubsystem.setIntakePower(coneMode, intakeSlider.get());
        intakeSubsystem.setIntakePower(coneMode, power);

        SmartDashboard.putBoolean("coneMode: ", coneMode);
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
