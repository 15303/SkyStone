package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="2: XingMeca", group="Linear Opmode")
//@Disabled
public class R2_MecaOpModeX extends LinearOpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor left_front = null;
  private DcMotor right_front = null;
  private DcMotor left_back = null;
  private DcMotor right_back = null;

  private DcMotor armH = null;
  private DcMotor armV = null;

  private CRServo foundation = null;
  private CRServo wrist = null;
  private CRServo grabber = null;

  double driveRht = 0;
  double driveFwd = 0;
  double driveC = 0;

  @Override
  public void runOpMode() {
    telemetry.addData("Status", "Initialized");
    telemetry.update();

    // Initialize the hardware variables. Note that the strings used here as parameters
    // to 'get' must correspond to the names assigned during the robot configuration
    // step (using the FTC Robot Controller app on the phone).
    left_front  = hardwareMap.get(DcMotor.class, "left_front");
    right_front = hardwareMap.get(DcMotor.class, "right_front");
    left_back = hardwareMap.get(DcMotor.class, "left_back");
    right_back = hardwareMap.get(DcMotor.class, "right_back");

    armH = hardwareMap.get(DcMotor.class, "armH");
    armV = hardwareMap.get(DcMotor.class, "armV");

    foundation = hardwareMap.get(CRServo.class, "foundation");
    wrist = hardwareMap.get(CRServo.class, "wrist");
    grabber = hardwareMap.get(CRServo.class, "grabber");

    // Most robots need the motor on one side to be reversed to drive forward
    // Reverse the motor that runs backwards when connected directly to the battery
    left_front.setDirection(DcMotor.Direction.FORWARD);
    right_front.setDirection(DcMotor.Direction.REVERSE);
    left_back.setDirection(DcMotor.Direction.FORWARD);
    right_back.setDirection(DcMotor.Direction.REVERSE);

//        armH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armH.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    //encoders becau
    left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    // Wait for the game to start (driver presses PLAY)

    waitForStart();
    runtime.reset();

    // run until the end of the match (driver presses STOP)
    while (opModeIsActive()) {

      driveRht = - Math.pow( ( gamepad1.left_stick_x + gamepad2.left_stick_x ) /2 , 3 ) - Math.pow( ( gamepad1.right_stick_x + gamepad2.right_stick_x ) / 2 , 3 );
      driveFwd = - Math.pow( ( gamepad1.left_stick_y + gamepad2.left_stick_y ) /2 , 3 ) - Math.pow( ( gamepad1.right_stick_y + gamepad2.right_stick_y ) / 2 , 3 );
      driveC   =   Math.pow( ( gamepad1.left_trigger - gamepad1.right_trigger ) , 3 );

      //mecanums
      
      left_front.setPower(  driveFwd + driveRht + driveC);
      left_back.setPower(   driveFwd - driveRht + driveC);
      right_front.setPower( driveFwd - driveRht - driveC);
      right_back.setPower(  driveFwd + driveRht - driveC);

      //foundation
//      gamepad1.dpad_down ? foundation.setPower(-0.5)
//      : gamepad1.dpad_up ? foundation.setPower(0.5)
//      : foundation.setPower(0);

      //arm movement
      armH.setPower(gamepad2.left_stick_x);
      armV.setPower(gamepad2.left_stick_y);

      //wrist
      wrist.setPower(gamepad2.right_stick_y);

      //grabber
//      gamepad2.dpad_down ? grabber.setPower(1)
//      : gamepad2.dpad_up ? grabber.setPower(-1)
//      : (gamepad2.left_trigger > 0.05 || gamepad2.right_trigger > 0.05) ? grabber.setPower(0);
    }
  }
}
