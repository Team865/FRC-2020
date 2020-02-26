package ca.warp7.frc2020.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Infrastructure implements Subsystem {
    private static Infrastructure instance;

    public static Infrastructure getInstance() {
        if (instance == null) {
            instance = new Infrastructure();
        }
        return instance;
    }

    private Compressor compressor = new Compressor();

    private Infrastructure() {
    }

    public void startCompressor() {
        compressor.start();
    }

    public void stopCompressor() {
        compressor.stop();
    }
}
