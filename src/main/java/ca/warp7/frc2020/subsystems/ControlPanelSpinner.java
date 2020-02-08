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

    //@Override
    //public void periodic() {
    //}

    /*
        Converts a RGB value to it's HSL hue value.
        Depending on what value is the highest, we use a difrent formula.
        ===This function needs some work and testing===
    */
    public static double rgbToHue(final double R, final double G, final double B){ 
        //used in hue calculation
        final double altR = R / 255;
        final double altG = G / 255;
        final double altB = B / 255;
        //we'll loop through this
        double[] RGB = new double[3];
        RGB[0] = R;
        RGB[1] = G;
        RGB[2] = B;
        //stores max and min values
        double min = 0;
        double max = 0;
        //ult return value
        double hue;
        
        //gets the highest value
        for(double v : RGB){
            if (v < min){
                min = v;
            }
            else if(v > max){
                max = v;
            }
        }

        //gets hue value
        if(max == R){
            hue = (altG - altB) / (max - min);
        }
        else if(max == G){
            hue = 2.0 + (altB - altR) / (max - min);
        }
        else if(max == B){
            hue = 4.0 + (altB - altR) / (max - min);
        }
        else{
            hue = 0.0;
        }

        //makes the value into degrees
        hue *= 60;
        if(hue < 0){
            hue += 360;
        }

        return (hue);
    }

    //Acsess the RGB precentages inside the returned Color with .red .green and .blue
    public Color getCurrentColor(){
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
