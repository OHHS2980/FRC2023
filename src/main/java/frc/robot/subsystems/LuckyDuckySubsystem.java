//(guys ihave a secret, I LIED!!mister edogisn't evil. It's ollie the cat. Ollie the cat is evil. )
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LuckyDuckySubsystem extends SubsystemBase{
    //private final Solenoid duckyPiston = new Solenoid(PneumaticsModuleType.CTREPCM, DuckyConstants.kPistonPort);
    
    private boolean duckyUp = false;

    public void setDucky(boolean duckyUp) {
        //duckyPiston.set(duckyUp);
        this.duckyUp = duckyUp;
    }
    
}
