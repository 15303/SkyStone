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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

import static java.lang.Math.abs;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="1: auto testing", group="Linear Opmode")
@Disabled
public class R1_red_stone_out_testing extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_front = null;
    private DcMotor right_front = null;
    private DcMotor left_back = null;
    private DcMotor right_back = null;

    private DcMotor arm_1 = null;
    private DcMotor arm_2 = null;

    private CRServo rotate = null;
    private CRServo grab = null;
    private CRServo foundation = null;

    private ColorSensor color = null;
    private DistanceSensor dist = null;

    private BNO055IMU imu;
    private Orientation angles;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        left_front  = hardwareMap.get(DcMotor.class, "left_front" );
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        left_back   = hardwareMap.get(DcMotor.class, "left_back"  );
        right_back  = hardwareMap.get(DcMotor.class, "right_back" );

        arm_1 = hardwareMap.get(DcMotor.class, "arm_1");
        arm_2 = hardwareMap.get(DcMotor.class, "arm_2");

        rotate = hardwareMap.get(CRServo.class, "rotate");
        grab = hardwareMap.get(CRServo.class, "grab");
        foundation = hardwareMap.get(CRServo.class, "foundation");

        color = hardwareMap.get(ColorSensor.class, "color");
        dist = hardwareMap.get(DistanceSensor.class, "color");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        left_front.setDirection(DcMotor.Direction.REVERSE);
        right_front.setDirection(DcMotor.Direction.FORWARD);
        left_back.setDirection(DcMotor.Direction.REVERSE);
        right_back.setDirection(DcMotor.Direction.FORWARD);

        //encoders because autonomous is autonomous
//        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double s1;
        double s2;
        double s3;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        side(-0.75, 90, 2000);

        s1 = (double) color.red()+color.green();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("value",color.red()+color.green());
            telemetry.addData("distance", dist.getDistance(DistanceUnit.INCH));
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES);
            if (angles.firstAngle < 0) {
                angles.firstAngle += 180;
            }
            telemetry.addData("heading", angles.firstAngle);
            telemetry.update();
        }
    }



    private void pause() {
        stopp();

        sleep(200);
    }
    private void drive(double power, int time) {
        left_front.setPower(power);
        right_front.setPower(power);
        left_back.setPower(power);
        right_back.setPower(power);

        sleep(time);
        pause();
    }
    private void side(double power, double degree, int time) {
//        double startt = getRuntime();
//        double endt = startt+time;

//        while (opModeIsActive() && getRuntime() < endt) {
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYZ, AngleUnit.DEGREES);
//            if (angles.firstAngle < 0) {
//                angles.firstAngle += 180;
//            }
//
//            if (angles.firstAngle > degree+3) {
//                left_front.setPower(-power);
//                right_front.setPower(power);
//                left_back.setPower(power - abs((degree-angles.firstAngle)/20));
//                right_back.setPower(-power + abs((degree-angles.firstAngle)/20));
//            } else if (angles.firstAngle < degree - 3) {
//                left_front.setPower(-power + abs((degree-angles.firstAngle)/20));
//                right_front.setPower(power - abs((degree-angles.firstAngle)/20));
//                left_back.setPower(power);
//                right_back.setPower(-power);
//            } else {
                left_front.setPower(-power);
                right_front.setPower(power);
                left_back.setPower(power);
                right_back.setPower(-power);
//            }
//            telemetry.addData("correction power", ((degree-angles.firstAngle)/20));
//            telemetry.addData("angle", angles.firstAngle);
//            telemetry.addData("runtime", getRuntime());
//            telemetry.addData("should be 1", abs(-1));
//            telemetry.update();
//
//        }
        sleep(time);

        pause();
    }
    private void turn(double powerL, double powerR, int time) {
        left_front.setPower(powerL);
        right_front.setPower(powerR);
        left_back.setPower(powerL);
        right_back.setPower(powerR);

        sleep(time);
        pause();
    }
    private void stopp() {
        left_front.setPower(0);
        right_front.setPower(0);
        left_back.setPower(0);
        right_back.setPower(0);
    }
}
