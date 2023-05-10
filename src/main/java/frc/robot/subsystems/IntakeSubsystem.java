package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{
    private final CANSparkMax cubeIntakeMotor = new CANSparkMax(IntakeConstants.kCubeIntakeMotorPort, MotorType.kBrushless);
    private final CANSparkMax coneIntakeMotor = new CANSparkMax(IntakeConstants.kConeIntakeMotorPort, MotorType.kBrushless);
    
    private boolean intaking;

    public void setIntakePower(boolean coneMode, double power){
        if(power > 0.75 || power < -0.75){
            if(coneMode){
                cubeIntakeMotor.set(power);
                coneIntakeMotor.set(power);
            }
            else{
                cubeIntakeMotor.set(-power);
                coneIntakeMotor.set(power);
            }
            intaking = power > 0.75;
        }else{
            cubeIntakeMotor.set(0);
            coneIntakeMotor.set(0);
        }
    }
}
