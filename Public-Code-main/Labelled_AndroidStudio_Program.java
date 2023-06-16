/**
This file is a fully structured FTC program, with no acutal movement coded.
It is meant to be used for beginners still learning the structure and syntax of Android Studios.
it also works for more lazy programmers who don't feel like writing the structure of the program out.
*/

package org.firstinspires.ftc.teamcode;

//Importing all the classes and or objects from the FTC APK
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//This tells the program that this file is a teleop type, with its name being "Default Name". @TeleOp can be changed to @Autonomous to designate the file as an autonomus program.
@TeleOp(name="Default Name")

public class Labelled_AndroidStudio_Program extends LinearOpMode {
//Line 16 designates the program as a LinearOpMode, meaning the code runs from the top line to the bottom line. It can also be labeled as an OpMode, giving more flexibility to the structure of the program.
    public void runOpMode() {
        //This is the initialization phase of the program (Setting up motor or servo positions, establishing motors and servos, or setting up webcams and sensors all go here).

        //This line tells the program to wait for the player to press the play button before executing any other code.
        waitForStart();

        while (opModeIsActive()) {
            // This is the run phase of the program, where the actions/movwement of the robot should be coded. If this is an auto program, change the while loop to an if statement to prevent the program from running again after finishing

        }

    }
}
