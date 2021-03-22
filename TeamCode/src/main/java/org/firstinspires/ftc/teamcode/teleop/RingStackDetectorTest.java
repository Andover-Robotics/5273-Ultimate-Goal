package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.RingStackDetector;

@TeleOp(name = "Ring Stack Detector Test", group = "Test")
public class RingStackDetectorTest extends OpMode {
    private RingStackDetector ringStackDetector;

    @Override
    public void init() {
        ringStackDetector = new RingStackDetector(this, telemetry);
    }

    @Override
    public void loop() {
        ringStackDetector.currentlyDetected()
                .ifPresent((pair) -> {
                    telemetry.addData("Status", "LOOKING FOR RINGS");
                    telemetry.addData("Detected Configuration", pair.first);
                    telemetry.addData("Detection Confidence", pair.second);
                    telemetry.update();
                });
    }

    @Override
    public void stop() {
        ringStackDetector.close();
    }
}