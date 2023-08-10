package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous (name="hi")
public class testAuto extends LinearOpMode {
    DcMotorEx topLeftMotor;
    DcMotorEx bottomLeftMotor;
    DcMotorEx topRightMotor;
    DcMotorEx bottomRightMotor;
    DcMotorEx[] motors;
    BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double currentAngle = 0.0;


    public void runOpMode() throws InterruptedException {

        //Turning setup

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        topLeftMotor = hardwareMap.get(DcMotorEx.class, "");
        bottomLeftMotor = hardwareMap.get(DcMotorEx.class, "");
        topRightMotor = hardwareMap.get(DcMotorEx.class, "");
        bottomRightMotor = hardwareMap.get(DcMotorEx.class, "");

        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx[] motors = {topLeftMotor,bottomLeftMotor,topRightMotor,bottomRightMotor};

        for (DcMotorEx motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        waitForStart();
        if(opModeIsActive()){
            moveForward(1000,1000);
            turnTo(90);
        }

    }

    public void moveMotors(int topLeft, int bottomLeft, int topRight, int bottomRight, int speed) {
        for (DcMotorEx motor : motors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        topLeftMotor.setTargetPosition(topLeft);
        bottomLeftMotor.setTargetPosition(bottomLeft);
        topRightMotor.setTargetPosition(topRight);
        bottomRightMotor.setTargetPosition(bottomRight);
        topLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topLeftMotor.setVelocity(speed);
        bottomLeftMotor.setVelocity(speed);
        topRightMotor.setVelocity(speed);
        bottomRightMotor.setVelocity(speed);

        while(opModeIsActive() && topLeftMotor.isBusy() && topRightMotor.isBusy() && bottomLeftMotor.isBusy() && bottomRightMotor.isBusy()){
            idle();
        }
    }

    public void moveForward(int targetDistance,int velocity) {
        moveMotors(targetDistance,targetDistance,targetDistance,targetDistance,velocity);
    }

    public void strafe(int targetDistance, int velocity){
        moveMotors(targetDistance,-targetDistance,-targetDistance,targetDistance,velocity);
    }

    // resets currAngle Value
    public void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        currentAngle = 0;
    }

    public double getAngle() {
        // Get current orientation
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);

        // Change in angle = current angle - previous angle
        double deltaAngle = orientation.firstAngle - lastAngles.firstAngle;

        // Gyro only ranges from -179 to 180
        // If it turns -1 degree over from -179 to 180, subtract 360 from the 359 to get -1
        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        // Add change in angle to current angle to get current angle
        currentAngle += deltaAngle;
        lastAngles = orientation;
        telemetry.addData("gyro", orientation.firstAngle);
        return currentAngle;
    }

    public void turn(double degrees){
        resetAngle();

        double error = degrees;

        while (opModeIsActive() && Math.abs(error) > 2) {
            double motorPower = (error < 0 ? -0.2 : 0.2);
            power(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("error", error);
            telemetry.update();
        }
        allPower(0);
    }

    public void turnTo(double degrees){
        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double error = degrees - orientation.firstAngle;
        if (error > 180) {
            error -= 360;
        } else if (error < -180) {
            error += 360;
        }
        turn(error);
    }

    public void allPower(double p){
        power(p,p,p,p);
    }

    public void power(double lF, double rF, double lB, double rB){
        for (DcMotorEx motor : motors){
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        topLeftMotor.setPower(lF);
        topRightMotor.setPower(rF);
        bottomLeftMotor.setPower(lB);
        topRightMotor.setPower(rB);
    }
}
