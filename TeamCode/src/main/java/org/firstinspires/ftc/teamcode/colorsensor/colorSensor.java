/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.colorsensor;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

/*
 * This OpMode illustrates the concept of driving up to a line and then stopping.
 * The code is structured as a LinearOpMode
 *
 * The Sensor used here can be a REV Color Sensor V2 or V3.  Make sure the white LED is turned on.
 * The sensor can be plugged into any I2C port, and must be named "sensor_color" in the active configuration.
 *
 *   Depending on the height of your color sensor, you may want to set the sensor "gain".
 *   The higher the gain, the greater the reflected light reading will be.
 *   Use the SensorColor sample in this folder to determine the minimum gain value that provides an
 *   "Alpha" reading of 1.0 when you are on top of the white line.  In this sample, we use a gain of 15
 *   which works well with a Rev V2 color sensor
 *
 *   Setting the correct WHITE_THRESHOLD value is key to stopping correctly.
 *   This should be set halfway between the bare-tile, and white-line "Alpha" values.
 *   The reflected light value can be read on the screen once the OpMode has been INIT, but before it is STARTED.
 *   Move the sensor on and off the white line and note the min and max readings.
 *   Edit this code to make WHITE_THRESHOLD halfway between the min and max.
 *
 *   Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 *   Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Disabled
@Autonomous(name="Colors", group="Robot")

public class colorSensor extends LinearOpMode {

    NormalizedColorSensor colorSensor;

    static final double     WHITE_THRESHOLD = 0.5;  // spans between 0.0 - 1.0 from dark to light
    static final double     APPROACH_SPEED  = 0.25;

    double gain = 15;
    double alpha = 1.0;



    public void runOpMode() {

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "sensor_color");



        // If necessary, turn ON the white LED (if there is no LED switch on the sensor)
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        // Some sensors allow you to set your light sensor gain for optimal sensitivity...
        // See the SensorColor sample in this folder for how to determine the optimal gain.
        // A gain of 15 causes a Rev Color Sensor V2 to produce an Alpha value of 1.0 at about 1.5" above the floor.
        waitForStart();
        while (opModeIsActive()) {

            float blue = colorSensor.getNormalizedColors().blue * 255;
            float red = colorSensor.getNormalizedColors().red * 255;
            float green = colorSensor.getNormalizedColors().green * 255;

            telemetry.addData("Blue", blue);
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);

            if(blue > red) {
                telemetry.addData("Color!", "Purple");
            } else if (green > 30){
                telemetry.addData("Color!", "Green");
            } else {
                telemetry.addData("Color!", "No Color");
            }

            telemetry.update();
        }

    }
}
