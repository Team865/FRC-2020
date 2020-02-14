/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package ca.warp7.frc2020.subsystems;

import ca.warp7.frc2020.lib.motor.MotorControlHelper;
import com.revrobotics.CANSparkMax;
/*
    Colour sensor documentation link
    http://www.revrobotics.com/content/sw/color-sensor-v3/sdk/docs/javadoc/com/revrobotics/ColorSensorV3.html
*/
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static ca.warp7.frc2020.Constants.kControlPanelManipulatorID;

public final class ControlPanelSpinner implements Subsystem {

    private int count = 0;

    private static ControlPanelSpinner instance;

    public static ControlPanelSpinner getInstance() {
        if (instance == null) instance = new ControlPanelSpinner();
        return instance;
    }

    @Override
    public void periodic(){
        System.out.println("Testy");
        if(count % 5 == 0){
            System.out.println(getCurrentColor());
        }
        count++;
    }

    /*
        Gets the current SpinnerColour that the
        ControlPanelSpinner's colour sensor is looking
        at.
    */
    private SpinnerColour getCurrentSpinnerColor(){
        final Color SENCED_COLOR = getCurrentColor();
        final int R = (int)(255 * SENCED_COLOR.red);
        final int G = (int)(255 * SENCED_COLOR.green);
        final int B = (int)(255 * SENCED_COLOR.blue);
        final float[] HSB = new float[3];
        java.awt.Color.RGBtoHSB(R, G, B, HSB);
        return (convertHueToColour(HSB[0]));
    }

    //needs to be filled in
    public SpinnerColour convertHueToColour(final float HUE){
        return (SpinnerColour.UNKNOWN);
    }

    //Acsess the RGB precentages inside the returned Color with .red .green and .blue
    private Color getCurrentColor(){
        return colorSensor.getColor();
    }

    private CANSparkMax controlPanelMiniNeo = MotorControlHelper.createMasterSparkMAX(kControlPanelManipulatorID);

    private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    public void setSpeed(double speed) {
        controlPanelMiniNeo.set(speed);
    }

    public ColorSensorV3 getColorSensor() {
        return colorSensor;
    }
}

/*
    Enum to be used to represent one of the 4 colours on
    the control panel
*/
enum SpinnerColour {
    RED,
    YELLOW,
    GREEN,
    BLUE,
    UNKNOWN
}