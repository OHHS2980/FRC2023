package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;


public class ArmSubsystem extends SubsystemBase{
    private final CANSparkMax armMotor = new CANSparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
    private final CANSparkMax armMotorSlave = new CANSparkMax(ArmConstants.kArmMotorSlavePort, MotorType.kBrushless);
    //private final CANSparkMax linearStage1Motor = new CANSparkMax(ArmConstants.kLinearMotor1Port, MotorType.kBrushless);
    private final CANSparkMax linearMotor = new CANSparkMax(ArmConstants.kLinearMotorPort, MotorType.kBrushless);

    private final CANSparkMax wristMotor = new CANSparkMax(ArmConstants.kWristMotorPort, MotorType.kBrushless);

    private PIDController linearPID = new PIDController(0.08, 1e-3, 0);
    private PIDController armPID = new PIDController(0.1, 1e-3, 0);
    private PIDController wristPID = new PIDController(0.2, 1e-3, 0);

    private Timer time = new Timer();

    private double prevTime = 0;
    private double armSpeedMultiplier = 1;

    private boolean movingOut = false;
    private boolean armThere = false;
    private boolean linearThere = false;

    public void initMotors(){
        armMotorSlave.follow(armMotor);
        linearPID.setTolerance(0.1,0.5);
        armPID.setTolerance(0.25,0.5);
        wristPID.setTolerance(0.1, 0.5);

        time.start();
    }

    public void print(){
        SmartDashboard.putNumber("Arm pos: ", armMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Linear Pos: ", linearMotor.getEncoder().getPosition());
        SmartDashboard.putNumber("Wrist Pos: ", wristMotor.getEncoder().getPosition());
        
        SmartDashboard.putNumber("Arm goal: ", armPID.getSetpoint());
        SmartDashboard.putNumber("Linear goal: ", linearPID.getSetpoint());
        SmartDashboard.putNumber("Wrist goal: ", wristPID.getSetpoint());
    }

    public void updateArmPosPower(){
        armThere = armMotor.getEncoder().getPosition() < armPID.getSetpoint() + 8;
        
        linearThere = linearMotor.getEncoder().getPosition() < linearPID.getSetpoint() + 4;

        if (movingOut){
            armMotor.set(armPID.calculate(armMotor.getEncoder().getPosition()) / 7 * armSpeedMultiplier);
            if (armThere){
                linearMotor.set(linearPID.calculate(linearMotor.getEncoder().getPosition()) / 2);
            }else{
                linearMotor.set(0);
            }
        }else{
            if(linearThere){
                armMotor.set(armPID.calculate(armMotor.getEncoder().getPosition()) / 7 * armSpeedMultiplier);
            }else{
                armMotor.set(0);
            }

            linearMotor.set(linearPID.calculate(linearMotor.getEncoder().getPosition()) / 2);
        }

        wristMotor.set(wristPID.calculate(wristMotor.getEncoder().getPosition()) / 7);

        SmartDashboard.putBoolean("Arm finished: ", armPID.atSetpoint());
        SmartDashboard.putBoolean("Linear finished: ", linearPID.atSetpoint());
        SmartDashboard.putBoolean("Wrist finished: ", wristPID.atSetpoint());
    }

    public void wereHome(){
        armMotor.getEncoder().setPosition(60);
        linearMotor.getEncoder().setPosition(0);
        wristMotor.getEncoder().setPosition(0);

        armPID.setSetpoint(60);
        linearPID.setSetpoint(0);
        wristPID.setSetpoint(0);
    }

    public void setArmPower(double armPower, double linearPower, double wristPower, boolean armOverride, boolean linearOverride, boolean wristOverride){
        double currTime = time.get();

        if(armOverride)
            armPID.setSetpoint(armPID.getSetpoint() + (-armPower * ((0.01) * 15)));

        if(linearOverride)
            linearPID.setSetpoint(linearPID.getSetpoint() + (-linearPower  * ((0.01) * 10)));


        if(wristOverride)
            wristPID.setSetpoint(wristPID.getSetpoint() + (-wristPower * ((0.01) * 5)));


        prevTime = time.get();
    }

    public void home(){
        setArmPos(60, 0, -0.2);
        armSpeedMultiplier = 0.5;
    }
    
    public void setArmPos(double armSet, double linearSet, double wristSet){
        movingOut = (armPID.getSetpoint() > 0 && armSet < 0) || (armPID.getSetpoint() < 0 && armSet > 0);

        armPID.setSetpoint(armSet);
        linearPID.setSetpoint(linearSet);
        wristPID.setSetpoint(wristSet);

        armSpeedMultiplier = 1;
    }

    public void setArmPos(){
        System.out.println("no setpoint here!");
    }

    public boolean isFinished(){
        return armPID.atSetpoint() && linearPID.atSetpoint() && wristPID.atSetpoint();
    }

}
