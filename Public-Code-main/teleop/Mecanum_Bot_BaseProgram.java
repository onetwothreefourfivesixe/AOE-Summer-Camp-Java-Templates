/* The movement code should work for the following FTC prebuilt drivetrains:

Andymark MecanAM FTC Chassis
Andymark TileRiser 4WD

REV Mecanum Drivetrain Kit

Gobilda Strafer™ Chassis Kit V5

Feel free to notify us about any extra drivetrains the code might work for. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Basic Mecanum Control")
public class Mecanum_Bot_BaseProgram extends LinearOpMode {
    //Example Motors
    DcMotor topLeftMotor;
    DcMotor bottomLeftMotor;
    DcMotor topRightMotor;
    DcMotor bottomRightMotor;
    //Initialization Phase
    public void runOpMode(){
        //Configure motors here. This matches the motor variable in the code to the name the motor has in the robot configuration.
        topLeftMotor = hardwareMap.dcMotor.get("[Insert Motor Name Here]");
        bottomLeftMotor = hardwareMap.dcMotor.get("[Insert Motor Name Here]");
        topRightMotor = hardwareMap.dcMotor.get("[Insert Motor Name Here]");
        bottomRightMotor = hardwareMap.dcMotor.get("[Insert Motor Name Here]");

        //Left side motors have to be reversed, as motors on one side of the drivetrain spin opposite of the other side.
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        //Run phase
        while(opModeIsActive()){

            /* All drivetrain movement is controlled here. Here is an example of one of the motor setPowers:

            topLeftMotor.setPower(-gamepad1.right_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);

            The first "-gamepad1.right_stick_y" represents the forward and backward controls.
            The second "+ gamepad1.left_stick_x" represents the strafing controls. Check the movement diagram in the github for specifics.
            the third and final "+ gamepad1.right_stick_x" represents the turning controls. */

            topLeftMotor.setPower(-gamepad1.right_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x);
            bottomLeftMotor.setPower(-gamepad1.right_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x);
            topRightMotor.setPower(-gamepad1.right_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x) ;
            bottomLeftMotor.setPower(-gamepad1.right_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x);

            //Add your own code!
        }
    }
}
