package ca.warp7.frc2020.auton.commands;

import ca.warp7.frc2020.subsystems.Feeder;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitForShotsCommand extends CommandBase {
    Feeder feeder = Feeder.getInstance();
    private int n;
    private int shotCountAtStart;

    public WaitForShotsCommand(int n) {
        this.n = n;
    }

    @Override
    public void initialize() {
        shotCountAtStart = feeder.getShotCount();
    }

    @Override
    public boolean isFinished() {
        return feeder.getShotCount() >= shotCountAtStart + n;
    }
}
