package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LuckyDuckySubsystem;

public class LuckyDuckyCmd extends CommandBase{

    private final LuckyDuckySubsystem luckyDuckySubsystem;
    
    private boolean duckyUp = false;
    private boolean pressed;

    private Supplier<Boolean> duckyToggleButton;

    private Timer elapsedTime = new Timer();

    private double holdStart;
    private double lastFlip;

    public LuckyDuckyCmd(LuckyDuckySubsystem luckyDuckySubsystem, 
        Supplier<Boolean> duckyToggleButton) {

        this.duckyToggleButton = duckyToggleButton;

        this.luckyDuckySubsystem = luckyDuckySubsystem;
        addRequirements(luckyDuckySubsystem);
    }

    @Override
    public void initialize() {
        duckyUp = false;
        pressed = false;
        holdStart = 0;
        lastFlip = 0;

        elapsedTime.start();

        luckyDuckySubsystem.setDucky(false);

        SmartDashboard.putBoolean("Ducky is Up: ", duckyUp);
    }

    @Override
    public void execute(){
        if (duckyToggleButton.get() && !pressed){
            holdStart = elapsedTime.get();
            duckyUp = !duckyUp;
        }

        pressed = false;
        if (duckyToggleButton.get()){
            pressed = true;
            if (elapsedTime.get() > holdStart + 2 && elapsedTime.get() > lastFlip + 0.5 ){
                duckyUp = !duckyUp;
            }
        }
        luckyDuckySubsystem.setDucky(duckyUp);
    }
    
    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
