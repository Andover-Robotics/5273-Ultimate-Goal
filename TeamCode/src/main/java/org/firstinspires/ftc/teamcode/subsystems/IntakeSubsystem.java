package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.teamcode.GlobalConfig;

public class IntakeSubsystem extends SubsystemBase {
    private final MotorEx intakeMotor;

    public IntakeSubsystem(HardwareMap hardwareMap, String intakeName){
        intakeMotor= new MotorEx(hardwareMap, intakeName);
        intakeMotor.setInverted(true);
    }

    public void reverseIntake(){
        intakeMotor.set(-1* GlobalConfig.INTAKE_MAX_POWER);
    }

    public void forwardIntake(){
        intakeMotor.set(GlobalConfig.INTAKE_MAX_POWER);
    }

    public void stopMotor(){
        intakeMotor.stopMotor();
    }
}
