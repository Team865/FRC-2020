package ca.warp7.frc2020.lib.trajectory;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;

import java.io.*;
import java.text.SimpleDateFormat;
import java.util.Date;

public class TrajectoryLogger {

    private static final SimpleDateFormat format = new SimpleDateFormat("MMdd_HHmm");

    ByteArrayOutputStream stream = new ByteArrayOutputStream();
    PrintWriter writer = new PrintWriter(stream);

    String name;

    public TrajectoryLogger(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public void writeToBuffer(double... data) {
        for (double datum : data) {
            writer.print(datum);
            writer.print(',');
        }
        writer.print('\n');
        writer.flush();
    }

    public void saveToFile() {
        File file = new File(System.getProperty("user.home") + "/" +
                format.format(new Date()) + "-" + name + ".csv");
        try(FileOutputStream out = new FileOutputStream(file)) {
            stream.writeTo(out);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
