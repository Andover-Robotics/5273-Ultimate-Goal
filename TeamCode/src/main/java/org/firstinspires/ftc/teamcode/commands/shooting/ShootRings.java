package org.firstinspires.ftc.teamcode.commands.shooting;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.commands.cartridge.PushRing;
import org.firstinspires.ftc.teamcode.commands.cartridge.RetractArm;
import org.firstinspires.ftc.teamcode.subsystems.CartridgeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class ShootRings extends SequentialCommandGroup {
    private final ShooterSubsystem shooter;
    private final CartridgeSubsystem cartridge;
    private final int numRings;

    public ShootRings(ShooterSubsystem shooter, CartridgeSubsystem cartridge, int numRings) {
        this.shooter = shooter;
        this.cartridge = cartridge;
        this.numRings = numRings;

        addCommands(new ShootRing(shooter, cartridge), new ShootRing(shooter, cartridge), new ShootRing(shooter, cartridge));

        addRequirements(shooter, cartridge);
    }
}
