//The movement code will only work for robots with two drivetrain motors

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Basic Tankdrive Control")
public class Barebones_Bot_BaseProgram extends LinearOpMode {
    //Example Motors
    DcMotor leftMotor;
    DcMotor rightMotor;
    //Initialization Phase
    public void runOpMode(){
        //Configure motors here. This matches the motor variable in the code to the name the motor has in the robot configuration.
        leftMotor = hardwareMap.dcMotor.get("[Insert Motor Name Here]");
        rightMotor = hardwareMap.dcMotor.get("[Insert Motor Name Here]");

        //Left side motors have to be reversed, as motors on one side of the drivetrain spin opposite of the other side.
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        //Run phase
        while(opModeIsActive()){
            
        }
    }
}
