package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp (

  name  = "OmniX"         ,
  group = "r2"

)

//@Disabled




























public class OmniX extends LinearOpMode {



















  private DcMotor driveNW = null;
  private DcMotor driveNE = null;
  private DcMotor driveSE = null;
  private DcMotor driveSW = null;

  private DcMotor slider  = null;
  private Servo   grabber = null;
  private Servo   dragger = null ;





  double stickXL = 0 ;
  double stickXR = 0 ;
  double stickYL = 0 ;
  double stickYR = 0 ;
  double trigL   = 0 ;
  double trigR   = 0 ;
  double bumpL   = 0 ;
  double bumpR   = 0 ;


  double grabberOpen    = 0.6;
  double grabberClosed  = 0.1;




  // minimum throttle for motors to have sufficient grip





  double driveRht  = 0 ;
  double driveFwd  = 0 ;
  double driveC    = 0 ;
  double normalize = 1 ;




  double sliderPower = 0   ;
  double grabberPos  = grabberOpen ;
  double draggerPos  = 0 ;




  boolean charmode = false;











  private double booleanToNum( boolean bool ){
    return bool ? 1 : -1;
  }





  @Override
  public void runOpMode() {



    telemetry.addData ( "Status    " , "OmniX Initialized" ) ;
    telemetry.update  (                                    ) ;



    driveNW = hardwareMap.get ( DcMotor.class , "driveNW" ) ;
    driveNE = hardwareMap.get ( DcMotor.class , "driveNE" ) ;
    driveSE = hardwareMap.get ( DcMotor.class , "driveSE" ) ;
    driveSW = hardwareMap.get ( DcMotor.class , "driveSW" ) ;

    slider  = hardwareMap.get ( DcMotor.class , "slider"  ) ;
    grabber = hardwareMap.get ( Servo.class   , "grabber" ) ;
    dragger = hardwareMap.get ( Servo.class   , "dragger" ) ;




    waitForStart      (                                    ) ;
    telemetry.addData ( "Status     " , "OmniX Active"     ) ;









    while ( opModeIsActive () ) {



















      //driving define

      stickXL = gamepad1.left_stick_x  + gamepad2.left_stick_x  ;
      stickXR = gamepad1.right_stick_x + gamepad2.right_stick_x ;

      stickYL = gamepad1.left_stick_y  + gamepad2.left_stick_y  ;
      stickYR = gamepad1.right_stick_y + gamepad2.right_stick_y ;

      trigL   = gamepad1.left_trigger  + gamepad2.left_trigger  ;
      trigR   = gamepad1.right_trigger + gamepad2.right_trigger ;
      bumpL   = booleanToNum( gamepad1.left_bumper  ) + booleanToNum( gamepad2.left_bumper  );
      bumpR   = booleanToNum( gamepad1.right_bumper ) + booleanToNum( gamepad2.right_bumper );




      driveFwd =   ( Math.pow ( stickYL , 3 ) + Math.pow ( stickYR , 3 ) ) / 2 ;

      driveRht = - ( Math.pow ( stickXL , 3 ) + Math.pow ( stickXR , 3 ) ) / 2 ;

      driveC   =  ( trigL - trigR ) + 0.1 * ( bumpL - bumpR ) ;


      normalize = Math.max ( Math.abs ( driveRht ) + Math.abs ( driveFwd ) + Math.abs ( driveC ) , 1 ) ;






      //driving do

      driveNW.setPower ( (   driveRht + driveFwd + driveC ) / normalize ) ;
      driveNE.setPower ( (   driveRht - driveFwd + driveC ) / normalize ) ;
      driveSE.setPower ( ( - driveRht - driveFwd + driveC ) / normalize ) ;
      driveSW.setPower ( ( - driveRht + driveFwd + driveC ) / normalize ) ;














      // arm define

      if ( !charmode && gamepad2.x && gamepad2.start ) {
        charmode = true;
      }

      if ( charmode ) {

        grabberPos  = ( gamepad1.dpad_up    || gamepad2.dpad_left  ) ?  grabberOpen
                    : ( gamepad1.dpad_down  || gamepad2.dpad_right ) ?  grabberClosed
                    :                                                   grabberPos ;

        sliderPower = ( gamepad1.dpad_left  || gamepad2.dpad_down  ) ?  1
                    : ( gamepad1.dpad_right || gamepad2.dpad_up    ) ? -1
                    :                                                    0          ;

      } else {


        sliderPower = ( gamepad1.dpad_left  || gamepad2.dpad_left  ) ?  1
                    : ( gamepad1.dpad_right || gamepad2.dpad_right ) ? -1
                    :                                                   0          ;


        grabberPos  = ( gamepad1.dpad_up    || gamepad2.dpad_up    ) ?  grabberOpen
                    : ( gamepad1.dpad_down  || gamepad2.dpad_down  ) ?  grabberClosed
                    :                                                   grabberPos ;

      }

      draggerPos  = ( gamepad1.y          || gamepad2.y            ) ?  0
                  : ( gamepad1.a          || gamepad2.a            ) ?  1
                  :                                                   draggerPos ;






      // arm do

      slider.setPower     ( sliderPower ) ;
      grabber.setPosition ( grabberPos  ) ;
      dragger.setPosition ( draggerPos  ) ;












      // telemetry

      telemetry.addData ( "Status     " , "OmniX Running"    ) ;
      telemetry.addData ( "DriveRht   " ,  driveRht          ) ;
      telemetry.addData ( "DriveFwd   " ,  driveFwd          ) ;
      telemetry.addData ( "DriveC     " ,  driveC            ) ;
      telemetry.addData ( "SliderPower" ,  sliderPower       ) ;
      telemetry.addData ( "GrabberPos " ,  grabberPos        ) ;
      telemetry.addData ( "DraggerPos " ,  draggerPos        ) ;
      telemetry.addData ( "Normalize  " ,  normalize         ) ;
      telemetry.update  (                                            ) ;








    }
  }
}