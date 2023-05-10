package frc.robot.commands;

import frc.robot.ColorSensor;
import frc.robot.subsystems.ArmSubsystem;

import java.util.function.Supplier;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmCmd extends CommandBase{
    private final ArmSubsystem armSubsystem;

    private AHRS gyro;

    private ColorSensor colorSensor;
    
    
    private Supplier<Boolean> highButton;
    private Supplier<Boolean> midButton;
    private Supplier<Boolean> lowButton;
    private Supplier<Boolean> resetPosButton;
    private Supplier<Boolean> intakePlaceToggleButton;
    private Supplier<Boolean> frontBackToggleButton;
    private Supplier<Boolean> armResetButton;
    private Supplier<Boolean> manualOverrideToggleButton;

    private Supplier<Double> armSlider;
    private Supplier<Double> linearSlider;
    private Supplier<Double> wristSlider;

    private Supplier<Double> feildOriented;

    private Supplier<Integer> operatorPOV;

    private boolean coneMode = false;
    private boolean backward = false;
    private boolean intaking = false;

    private boolean armOverride = false;
    private boolean linearOverride = false;
    private boolean wristOverride = false;

    private boolean override = false;

    
    public ArmCmd(ArmSubsystem armSubsystem,
        AHRS gyro,
        Supplier<Double> armSliderSupplier,
        Supplier<Double> linearSliderSupplier,
        Supplier<Double> wristSliderSupplier,
        Supplier<Boolean> highButtonSupplier,
        Supplier<Boolean> midButtonSupplier,
        Supplier<Boolean> lowButtonSupplier,
        Supplier<Integer> operatorPOVSupplier,
        Supplier<Boolean> resetPosButtonSupplier,
        Supplier<Boolean> intakePlaceButtonSupplier,
        Supplier<Boolean> frontBackButtonSupplier,
        Supplier<Boolean> armResetSupplier,
        Supplier<Boolean> manualOverrideSupplier,
        Supplier<Double> feildOriented){


        this.armSlider = armSliderSupplier;
        this.linearSlider = linearSliderSupplier;
        this.wristSlider = wristSliderSupplier;

        this.highButton = highButtonSupplier;
        this.midButton = midButtonSupplier;
        this.lowButton = lowButtonSupplier;
        this.resetPosButton = resetPosButtonSupplier;
        this.intakePlaceToggleButton = intakePlaceButtonSupplier;
        this.frontBackToggleButton = frontBackButtonSupplier;
        this.armResetButton = armResetSupplier;
        this.manualOverrideToggleButton = manualOverrideSupplier;
        
        this.operatorPOV = operatorPOVSupplier;

        this.feildOriented = feildOriented;

        this.gyro = gyro;

        colorSensor = new ColorSensor(0x3C, false);//perhaps 0x1E
        colorSensor.issueCommand(ColorSensor.ACTIVE_MODE_COMMAND);// turn on light

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        armSubsystem.initMotors();
    }

    @Override
    public void execute() {
        coneMode = IntakeCmd.getConeMode();

        //  intaking = figure out the dang color sensor
        if (intakePlaceToggleButton.get()){
            intaking = !intaking;
        }

        if (frontBackToggleButton.get()){
            backward = !backward;
        }

        if (resetPosButton.get()){
            //if (override){
                armSubsystem.wereHome();
            /*}else{
                feildOriented.get();
            }*/
        }
/*
        if (intaking){
            if (frontBackToggleButton.get()){
                backward = !backward;
            }
        }else{
            backward = Math.abs(Math.IEEEremainder(gyro.getAngle(), 360)) > 90;
        }
        */


        boolean[] bools = {coneMode, backward, intaking};
        byte b = 0;
        for (int i = 0; i < bools.length; i++) {
            b |= ((bools[i])?(1<< (bools.length - i - 1)):0);
        }
        
        if (!override){
            //extents arm: (-60?,63) linear: (0,29.75) wrist: (0,-4.ehh)  ✓
            if (operatorPOV.get() == 0){
                switch(b){
                    case(0): armSubsystem.setArmPos(18, 30, -10);//cube front place
                            break;
                    case(1): armSubsystem.setArmPos(19.9, 11.8, -17.4);//cube front intake
                            break;
                    case(2): armSubsystem.setArmPos(-20, 28, -0.25);//cube back place
                            break;
                    case(3): armSubsystem.setArmPos();//cube back intake
                            break;
                    case(4): armSubsystem.setArmPos();//cone front place
                            break;
                    case(5): armSubsystem.setArmPos();//cone front intake
                            break;
                    case(6): armSubsystem.setArmPos(-14.9, 27.8, -12.5);//cone back place was 29.5
                            break;
                    case(7): armSubsystem.setArmPos(-15, 25, -7);//cone back intake
                            break;
                }
            }

            if (operatorPOV.get() == 270 || operatorPOV.get() == 90){
                switch(b){
                    case(0): armSubsystem.setArmPos(30, 10, -10.5);//cube front place
                            break;
                    case(1): armSubsystem.setArmPos(40, 0, -0.2);//cube front intake
                            break;
                    case(2): armSubsystem.setArmPos(-25, 10, -0.3);//cube back place
                            break;
                    case(3): armSubsystem.setArmPos();//cube back intake
                            break;
                    case(4): armSubsystem.setArmPos(23.9, 9.6, -19.3);//cone front place
                            break;
                    case(5): armSubsystem.setArmPos(35, 0, -5);//cone front intake
                            break;
                    case(6): armSubsystem.setArmPos(-15, 19, -8);//cone back place
                            break;
                    case(7): armSubsystem.setArmPos(-30, 0, -5);//cone back intake
                            break;
                }
            }

            if (operatorPOV.get() == 180){
                switch(b){
                    case(0): armSubsystem.setArmPos(60, 0, -6);//cube front place
                            break;
                    case(1): armSubsystem.setArmPos(60, 0, -12);//cube front intake
                            break;
                    case(2): armSubsystem.setArmPos();//cube back place
                            break;
                    case(3): armSubsystem.setArmPos();//cube back intake
                            break;
                    case(4): armSubsystem.setArmPos(60, 0, -9);//cone front place
                            break;
                    case(5): armSubsystem.setArmPos(60, 0, -16.35);//cone front intake
                            break;
                    case(6): armSubsystem.setArmPos();//cone back place
                            break;
                    case(7): armSubsystem.setArmPos(-35, 0, 0);//cone back intake (upright cone)
                            break;
                }
            }
        }

        if (armResetButton.get()){
            armSubsystem.home();
        }

        if (manualOverrideToggleButton.get()){
            override = !override;
        }

        armOverride = armSlider.get() > 0.6 || armSlider.get() < -0.6;

        linearOverride = operatorPOV.get() == 0 || operatorPOV.get() == 45 || operatorPOV.get() == 315 || operatorPOV.get() == 135 || operatorPOV.get() == 180 || operatorPOV.get() == 225;

        wristOverride = wristSlider.get() > 0.6 || wristSlider.get() < -0.6;

        double linearPower;
        if(operatorPOV.get() == 0 || operatorPOV.get() == 45 || operatorPOV.get() == 315){
             linearPower = -1;
        }else if(operatorPOV.get() == 135 || operatorPOV.get() == 180 || operatorPOV.get() == 225){
             linearPower = 1;
        }else{
             linearPower = 0;
        }
//nerd meowmeo wemomwowmeomewo weweomewm  w
//michi is awesome and cool
//coment
//meowmeowmoemwoemowmeowmeomqwoemwomeoemowmew
//TOP SECRET: mister edoga is acshwalley evil!!1111 HES THE ROBOT MASTERMIND source: I'm super smart
//mister edoga puts sourcream on his robots..... evil...... 
// he's a cannibal (because hes actually seretly a robot, the realmister edoga dies ages ago, he was replaced by an evil AI robot pretending to be him)
        if (override){
            // power control on sliders
            armSubsystem.setArmPower(-armSlider.get(), linearPower, wristSlider.get(), armOverride, linearOverride, wristOverride);
        }
        armSubsystem.updateArmPosPower();


        armSubsystem.print();

        SmartDashboard.putBoolean("cone mode: ", coneMode);
        SmartDashboard.putBoolean("backward: ", backward);
        SmartDashboard.putBoolean("intaking: ", intaking);
        SmartDashboard.putBoolean("override: ", override);
        
        SmartDashboard.putString("setting: ", ((coneMode)?"cone ":"cube ") + ((intaking)?"intake ":"place ") + ((backward)?"back ":"front "));
        

        SmartDashboard.putNumber("color sensor value: ", colorSensor.readByte(ColorSensor.COLOR_NUMBER_REGISTER));
        
        SmartDashboard.putNumber("color sensor red: ", colorSensor.readByte(ColorSensor.RED_VALUE_REGISTER));
        SmartDashboard.putNumber("color sensor greeen: ", colorSensor.readByte(ColorSensor.GREEN_VALUE_REGISTER));
        SmartDashboard.putNumber("color sensor blue: ", colorSensor.readByte(ColorSensor.BLUE_VALUE_REGISTER));
        
        SmartDashboard.putNumber("Operator POV: ", operatorPOV.get());
    }

    
    public void setCubeIntake(){
        armSubsystem.setArmPos(60, 0, -13);
        intaking = false;
        backward = false;
        IntakeCmd.setConeMode(false);
    }

    public void setConeIntake(){
        armSubsystem.setArmPos(60, 0, -17);
        intaking = false;
        backward = true;
        IntakeCmd.setConeMode(true);
    }

    public void resetRelease(){
        armSubsystem.home();
    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    

}
