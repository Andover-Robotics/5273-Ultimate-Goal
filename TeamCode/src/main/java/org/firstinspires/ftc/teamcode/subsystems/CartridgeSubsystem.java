package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalConfig;

public class CartridgeSubsystem extends SubsystemBase {
    private final SimpleServo cartridgeTilt, cartridgeArm;
    public enum ArmState{
        Pushed, Reset
    }
    public ArmState armState;


    public CartridgeSubsystem(HardwareMap hardwareMap, String tiltName, String armName) {
        cartridgeTilt = new SimpleServo(hardwareMap, tiltName, 300, 0);
        cartridgeArm = new SimpleServo(hardwareMap, armName, 300, 0);
        this.armState = ArmState.Reset;
    }

    public void levelCartridge() {
        cartridgeTilt.setPosition(GlobalConfig.CARTRIDGE_LEVEL_POSITION);
    }

    public void lowerCartridge() {
        cartridgeTilt.setPosition(GlobalConfig.CARTRIDGE_INTAKE_POSITION);
    }

    public void raiseCartridge() {
        cartridgeTilt.setPosition(GlobalConfig.CARTRIDGE_SHOOTER_POSITION);
    }

    public void pushArm() {
        cartridgeArm.setPosition(GlobalConfig.CARTRIDGE_ARM_PUSH_RING_POSITION);
        this.armState = ArmState.Pushed;
    }

    public void resetArm() {
        cartridgeArm.setPosition(GlobalConfig.CARTRIDGE_ARM_NEUTRAL_POSITION);
        this.armState = ArmState.Reset;
    }

    public void initCartridge() {
        raiseCartridge();
        resetArm();
    }

    public double getArmPosition() {
        return cartridgeArm.getPosition();
    }
}
