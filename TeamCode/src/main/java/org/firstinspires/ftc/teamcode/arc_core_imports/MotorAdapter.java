package org.firstinspires.ftc.teamcode.arc_core_imports;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;

/**
 * Adapts a DcMotor to the IMotor interface for the DriveTrains.
 */
public class MotorAdapter extends CachedMotor implements IMotor {

    /**
     * Creates a new MotorAdapter for the given motor.
     *
     * @param motor The motor to send commands to
     */
    public MotorAdapter(DcMotor motor) {
        super(motor);
    }

    @Override
    public void addTargetPosition(int tickOffset) {
        setTargetPosition(motor.getCurrentPosition() + tickOffset);
    }

    @Override
    public boolean isBusy() {
        return motor.isBusy();
    }
}
