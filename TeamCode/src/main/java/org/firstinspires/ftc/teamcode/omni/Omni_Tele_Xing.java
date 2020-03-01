package org.firstinspires.ftc.teamcode.omni ;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode ;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp ;
import com.qualcomm.robotcore.hardware.CRServo ;
import com.qualcomm.robotcore.hardware.DcMotor ;
import com.qualcomm.robotcore.hardware.Servo ;

@TeleOp ( name = "Omni Tele Xing" )

public class Omni_Tele_Xing extends LinearOpMode {
  
  private double diffbool ( boolean first , boolean second ) {
    return ( first ? 1 : 0 ) - ( second ? 1 : 0 ) ;
  }
  
  private double curve ( double left_stick , double right_stick ) {
    return ( Math.pow ( left_stick , 3 ) + Math.pow ( right_stick, 3 ) ) / 2 ;
  }
  
  private double clamp ( double min , double input, double max ) {
    return Math.max ( min , Math.min ( input , max ) ) ;
  }

  @Override
  public void runOpMode() {
  
		telemetry.addLine ( "OmniX INITIALIZING" ) ;
		telemetry.update () ;
  
		private DcMotor motor_nw = hardwareMap.get ( DcMotor.class , "driveNW" ) ;
		private DcMotor motor_ne = hardwareMap.get ( DcMotor.class , "driveNE" ) ;
		private DcMotor motor_se = hardwareMap.get ( DcMotor.class , "driveSE" ) ;
		private DcMotor motor_sw = hardwareMap.get ( DcMotor.class , "driveSW" ) ;
  
		private DcMotor slider = hardwareMap.get ( DcMotor.class , "slider" ) ;
		private Servo grabber = hardwareMap.get ( Servo.class , "grabber" ) ;
		private Servo dragger = hardwareMap.get ( Servo.class , "dragger" ) ;
		private CRServo dropper = hardwareMap.get ( CRServo.class , "dropper" ) ;
  
		double drive_right = 0 ;
		double drive_forward = 0 ;
		double drive_clockwise = 0 ;

    double grabber_state = 1 ;
    double dragger_state = 0 ;

    double dropper_power = 0 ;
    double slider_power = 0 ;
  
  	telemetry.addLine ( "OmniX INITIALIZED" ) ;
  	telemetry.update () ;
  	
  	waitForStart () ;
  	
  	telemetry.addLine ( "OmniX ACTIVE" ) ;
  	telemtry.update() ;
  
  	while ( opModeIsActive () ) {
  
      drive_forward = curve (
        gamepad1.left_stick_y + gamepad2.left_stick_y ,
        gamepad1.right_stick_y + gamepad2.right_stick_y
      );
  
      drive_right = - curve (
        gamepad1.left_stick_x + gamepad2.left_stick_x ,
        gamepad1.right_stick_x - gamepad2.right_stick_x
      ) ;
  
      drive_clockwise = (
          gamepad1.left_trigger + gamepad2.left_trigger
        - gamepad1.right_trigger - gamepad2.right_trigger
        + diffbool ( gamepad1.left_bumper || gamepad2.left_bumper , gamepad1.right_bumper || gamepad2.right_bumper ) / 9
      ) ;
  
      motor_nw.setPower ( drive_right + drive_forward + drive_clockwise ) ;
      motor_ne.setPower ( drive_right - drive_forward + drive_clockwise ) ;
      motor_se.setPower ( - drive_right - drive_forward + drive_clockwise ) ;
      motor_sw.setPower ( - drive_right + drive_forward + drive_clockwise ) ;
  
      if ( gamepad1.dpad_up || gamepad2.dpad_left ){
        grabber_state = clamp ( 0 , grabber_state - 0.003 , 0.9 ) ;
      } else if ( gamepad1.dpad_down || gamepad2.dpad_right ) {
        grabber_state = 1 ;
      }
  
      if ( gamepad1.y || gamepad2.y ) {
        dragger_state = 0 ;
      } else if ( gamepad1.x || gamepad2.x ) {
        dragger_state = 1 ;
      }
      
      grabber.setPosition ( 1 - grabber_state * 0.75 - 0.1 ) ;
      dragger.setPosition ( dragger_state * 0.35 + 0.5 ) ;
  
      dropper.setPower ( diffbool ( gamepad1.a || gamepad2.a , gamepad1.b || gamepad2.b ) ) ;
      slider.setPower ( diffbool ( gamepad1.dpad_left || gamepad2.dpad_up , gamepad1.dpad_right || gamepad2.dpad_down ) ) ;
  
      telemetry.addLine ( "OmniX RUNNING" ) ;
      telemetry.addLine ( "→" + drive_right ) ;
      telemetry.addLine ( "↑" + drive_forward ) ;
      telemetry.addLine ( "↻" + drive_clockwise ) ;
      telemetry.addLine ( "s" + slider_power ) ;
      telemetry.addLine ( "g" + grabber_state ) ;
      telemetry.addLine ( "d" + dragger_state ) ;
      telemetry.update () ;
  
    }
  }
}