package org.firstinspires.ftc.teamcode.arc_core_imports;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class CachedMotor {
    protected final DcMotor motor;

    private double power;
    private int targetPosition;
    private DcMotor.RunMode mode;

    public CachedMotor(DcMotor motor) {
        this.motor = motor;

        power = motor.getPower();
        targetPosition = Integer.MIN_VALUE;
        mode = motor.getMode();
    }

    public static CachedMotor fromHardware(HardwareMap map, String motorName) {
        return new CachedMotor(map.dcMotor.get(motorName));
    }

    public void setPower(double unboundedPower) {
        final double power = Range.clip(unboundedPower, -1, 1);

        if (Math.abs(power - this.power) < 1e-4)
            return;

        this.power = power;
        motor.setPower(power);
    }

    public double getPower() {
        return power;
    }

    public void setTargetPosition(int pos) {
        if (pos == targetPosition)
            return;

        targetPosition = pos;
        motor.setTargetPosition(pos);
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public void setMode(DcMotor.RunMode runMode) {
        if (runMode == mode)
            return;

        mode = runMode;
        motor.setMode(mode);
    }

    public DcMotor.RunMode getMode() {
        return mode;
    }

    public DcMotor getMotor() {
        return motor;
    }
}
