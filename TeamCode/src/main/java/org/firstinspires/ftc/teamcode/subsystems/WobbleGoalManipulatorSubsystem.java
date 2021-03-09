package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GlobalConfig;

public class WobbleGoalManipulatorSubsystem extends SubsystemBase {
    private final SimpleServo wobbleGoalTilt, wobbleGoalClaw;


    public WobbleGoalManipulatorSubsystem(HardwareMap hardwareMap, String tiltName, String clawName) {
        wobbleGoalTilt = new SimpleServo(hardwareMap, tiltName, 300, 0);
        wobbleGoalClaw = new SimpleServo(hardwareMap, clawName, 300, 0);
    }

    public void lowerArm() {
        wobbleGoalTilt.setPosition(GlobalConfig.WOBBLE_GOAL_ARM_DOWN_POSITION);
    }

    public void raiseArm() {
        wobbleGoalTilt.setPosition(GlobalConfig.WOBBLE_GOAL_MOVING_POSITION);
    }

    public void tuckArm() {
        wobbleGoalTilt.setPosition(GlobalConfig.WOBBLE_GOAL_ARM_TUCKED_POSITION);
    }

    public void raiseOverWall() {
        wobbleGoalTilt.setPosition(GlobalConfig.WOBBLE_GOAL_ARM_OVER_WALL_POSITION);
    }

    public void grip() {
        wobbleGoalClaw.setPosition(GlobalConfig.WOBBLE_GOAL_CLAW_GRAB_POSITION);
    }

    public void release() {
        wobbleGoalClaw.setPosition(GlobalConfig.WOBBLE_GOAL_CLAW_RELEASE_POSITION);
    }

    public void openWide() {
        wobbleGoalClaw.setPosition(GlobalConfig.WOBBLE_GOAL_CLAW_OPEN_WIDE);
    }

    public double getArmPosition() {
        return wobbleGoalTilt.getPosition();
    }

    public double getClawPosition() {
        return wobbleGoalClaw.getPosition();
    }

}
