package org.firstinspires.ftc.teamcode.autonomous.alliance_insignificant;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.GlobalConfig;
import org.firstinspires.ftc.teamcode.arc_core_imports.MecanumDrive;

@Autonomous(name = "Parking Auto", group = "AA")
public class ParkingAuto extends LinearOpMode {
    private MecanumDrive mecanumDrive;

    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        waitForStartWithPings();

        mecanumDrive.driveForwards(GlobalConfig.TILE_LENGTH_IN * 3.25);

        stop();
    }

    private void setup() {
        DcMotor motorFR = hardwareMap.dcMotor.get("motorFR");
        DcMotor motorFL = hardwareMap.dcMotor.get("motorFL");
        DcMotor motorBR = hardwareMap.dcMotor.get("motorBR");
        DcMotor motorBL = hardwareMap.dcMotor.get("motorBL");

        mecanumDrive = MecanumDrive.fromOctagonalMotors(motorFL, motorFR, motorBL, motorBR, this, (int)(GlobalConfig.TICKS_PER_INCH + 0.5), (int)(GlobalConfig.TICKS_PER_MOTOR_REVOLUTION + 0.5));
    }

    private void waitForStartWithPings() {
        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Waiting in Init", System.currentTimeMillis());
            telemetry.update();
        }

        telemetry.update();
    }
}
