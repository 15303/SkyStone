package org.firstinspires.ftc.teamcode.omni;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp (
  name  = "OmniX"         ,
  group = "2"
)
//@Disabled
public class OmniX extends LinearOpMode {

  private DcMotor driveNW = null;
  private DcMotor driveNE = null;
  private DcMotor driveSE = null;
  private DcMotor driveSW = null;

  private DcMotor slider = null;
  private Servo grabber = null;
  private Servo dragger = null;
  private CRServo dropper = null;

  double left_stick_right = 0 ;
  double right_stick_right = 0 ;
  double left_stick_forward = 0 ;
  double right_stick_forward = 0 ;
  double trigger_left = 0 ;
  double trigger_right = 0 ;
  double bumper_left = 0 ;
  double bumper_right = 0 ;

  // minimum throttle for motors to have sufficient grip

  double drive_right  = 0 ;
  double drive_forward  = 0 ;
  double drive_clockwise    = 0 ;

  double slider_power = 0   ;
  double grabber_position  = 0.3 ;
  double dragger_position  = 0.2 ;
  double dropper_position  = 0 ;

  private int boolean_to_int( boolean bool ){
	return bool ? 1 : -1;
  }

	private int clamp ( int low , int var , int high ){
		return Math.max(low,Math.min(var,high));
	}
	private double clamp ( int low , double var , int high ){
		return Math.max(low,Math.min(var,high));
	}

  @Override
  public void runOpMode() {

	telemetry.addData ( "Status" , "OmniX Initialized" ) ;
	telemetry.update () ;

	driveNW = hardwareMap.get ( DcMotor.class , "driveNW" ) ;
	driveNE = hardwareMap.get ( DcMotor.class , "driveNE" ) ;
	driveSE = hardwareMap.get ( DcMotor.class , "driveSE" ) ;
	driveSW = hardwareMap.get ( DcMotor.class , "driveSW" ) ;

	slider  = hardwareMap.get ( DcMotor.class , "slider"  ) ;
	grabber = hardwareMap.get ( Servo.class   , "grabber" ) ;
  dragger = hardwareMap.get ( Servo.class   , "dragger" ) ;
  dropper = hardwareMap.get ( CRServo.class   , "dropper" ) ;

	waitForStart ( ) ;
	telemetry.addData ( "Status" , "OmniX Active" ) ;

	while ( opModeIsActive () ) {

	  //driving define

	  left_stick_right = gamepad1.left_stick_x  + gamepad2.left_stick_x  ;
	  right_stick_right = gamepad1.right_stick_x + gamepad2.right_stick_x ;

	  left_stick_forward = gamepad1.left_stick_y  + gamepad2.left_stick_y  ;
	  right_stick_forward = gamepad1.right_stick_y + gamepad2.right_stick_y ;

	  trigger_left = gamepad1.left_trigger  + gamepad2.left_trigger  ;
	  trigger_right = gamepad1.right_trigger + gamepad2.right_trigger ;
	  bumper_left = boolean_to_int( gamepad1.left_bumper  ) + boolean_to_int( gamepad2.left_bumper  );
	  bumper_right = boolean_to_int( gamepad1.right_bumper ) + boolean_to_int( gamepad2.right_bumper );

	  drive_forward = ( Math.pow ( left_stick_forward , 3 ) + Math.pow ( right_stick_forward , 3 ) ) / 2 ;
	  drive_right = - ( Math.pow ( left_stick_right , 3 ) + Math.pow ( right_stick_right , 3 ) ) / 2 ;
	  drive_clockwise = ( trigger_left - trigger_right ) + 0.1 * ( bumper_left - bumper_right ) ;

	  driveNW.setPower (   drive_right + drive_forward + drive_clockwise ) ;
	  driveNE.setPower (   drive_right - drive_forward + drive_clockwise ) ;
	  driveSE.setPower ( - drive_right - drive_forward + drive_clockwise ) ;
	  driveSW.setPower ( - drive_right + drive_forward + drive_clockwise ) ;

	  slider.setPower ( (gamepad1.dpad_left||gamepad2.dpad_up) ? 1 : (gamepad1.dpad_right||gamepad2.dpad_down) ? -1 : 0 ) ;
    if ( gamepad1.dpad_up || gamepad2.dpad_left ){
      grabber_position += 0.002;
    } else if (gamepad1.dpad_down || gamepad2.dpad_right) {
      grabber_position = 0;
    }
    grabber_position = Math.max(0.15,Math.min(grabber_position,0.9));
    if( gamepad1.y||gamepad2.y||gamepad1.a||gamepad2.a ){
      dragger_position = ( gamepad1.y || gamepad2.y ) ? 0.2 : 0.85 ;
    }
    if( gamepad1.x||gamepad2.x||gamepad1.b||gamepad2.b ) {
      dropper_position = ( gamepad1.x || gamepad2.x ) ? 0.5 : 0.6 ;
    }
    grabber.setPosition(grabber_position);
    dragger.setPosition(dragger_position);
    dropper.setPower( (gamepad1.x||gamepad2.x) ? 1 : (gamepad1.b||gamepad2.b) ? -1 : 0);

	  // telemetry

	  telemetry.addData ( "Status" , "OmniX Running" ) ;
	  telemetry.addData ( "drive right" , drive_right ) ;
	  telemetry.addData ( "drive forward" , drive_forward ) ;
	  telemetry.addData ( "drive clockwise" , drive_clockwise ) ;
	  telemetry.addData ( "slider power" , slider_power ) ;
	  telemetry.addData ( "grabber position" , grabber_position ) ;
	  telemetry.addData ( "dragger position" , dragger_position ) ;
	  telemetry.update () ;

	}
  }
}