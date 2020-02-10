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
    
    public ControlPanelSpinner(){
        testRgbToHue();
    }

    private int count = 0;

    private static ControlPanelSpinner instance;

    public static ControlPanelSpinner getInstance() {
        if (instance == null) instance = new ControlPanelSpinner();
        return instance;
    }

    @Override
    public void periodic() {
        if(count % 5 == 0){
            //System.out.println("Going...going..going...");
        }
        count++;
    }

    /*
        Converts a 0-1 R G and B value to it's HSL hue value.
        Depending on what value is the highest, we use a difrent formula.
    */
    public static int rgbToHue(final double R, final double G, final double B){
        //stores max and min values
        double max = Math.max(R, Math.max(G, B));
        double min = Math.min(R, Math.min(G, B));;
        //ult return value
        double hue;
        boolean hasHue; //is true if the colour is not white black or gray

        //gets hue value
        double maxMinDiff = max - min;

        if(maxMinDiff == 0){
            hue = 0; //white or gray or black
            hasHue = false;
        }
        else if(max == R){
            hue = (G - B) / maxMinDiff;
            hasHue = true;
        }
        else if(max == G){
            hue = 2.0 + (B - R) / maxMinDiff;
            hasHue = true;
        }
        else if(max == B){
            hue = 4.0 + (B - R) / maxMinDiff;
            hasHue = true;
        }
        else{
            hue = 0.0;
            hasHue = false;
        }

        //The hue is a number from 0-6.
        if(hue < 0){
            hue += 6;
        }

        hue *= 60; //convert to angle

        int theCoolerHue = (int)hue;

        //-1 will be our value for no hue.
        if(hasHue == false){
            hue = -1;
        }

        return (theCoolerHue);
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


    //===TEST FUNCTIONS===

    private void testRgbToHue(){
        System.out.println("Testing rgbToHue in ControlPanelSpinner");
        //red
        System.out.println(rgbToHue(1.0, 0.0, 0.0));
        //orange
        System.out.println(rgbToHue(1.0, 0.5, 0.0));
        //yellow
        System.out.println(rgbToHue(1.0, 1.0, 0.0));
        //grean
        System.out.println(rgbToHue(0.0, 1.0, 0.0));
        //blue
        System.out.println(rgbToHue(0.0, 0.0, 1.0));
        //purple
        System.out.println(rgbToHue(0.0, 1.0, 1.0));
    }

}
