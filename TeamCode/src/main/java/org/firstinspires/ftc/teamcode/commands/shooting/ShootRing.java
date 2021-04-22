package org.firstinspires.ftc.teamcode.commands.shooting;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.cartridge.PushRing;
import org.firstinspires.ftc.teamcode.commands.cartridge.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.CartridgeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootRing extends SequentialCommandGroup {
    private final ShooterSubsystem shooter;
    private final CartridgeSubsystem cartridge;
    private final int armRetractionDelay = 750;

    public ShootRing(ShooterSubsystem shooter, CartridgeSubsystem cartridge, Telemetry telemetry) {
        this.shooter = shooter;
        this.cartridge = cartridge;

        addCommands(new StartShooter(shooter, telemetry), new PushRing(cartridge), new WaitCommand(armRetractionDelay),
                new RetractArm(cartridge));

        addRequirements(shooter, cartridge);
    }
}
