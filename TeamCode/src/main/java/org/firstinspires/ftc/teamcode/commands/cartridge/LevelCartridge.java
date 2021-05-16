package org.firstinspires.ftc.teamcode.commands.cartridge;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.CartridgeSubsystem;

public class LevelCartridge extends CommandBase {
    private final CartridgeSubsystem cartridgeSubsystem;

    public LevelCartridge(CartridgeSubsystem cartridgeSubsystem){
        this.cartridgeSubsystem=cartridgeSubsystem;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
        cartridgeSubsystem.levelCartridge();
    }
}
