package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="OmniAutoR1", group="Linear Opmode")
//@Disabled

public class OmniOpModeR1 extends LinearOpMode {

  boolean isRed = true; //IMPORTANT

  private ElapsedTime runtime = new ElapsedTime();
  private DcMotor driveNW = null;
  private DcMotor driveNE = null;
  private DcMotor driveSE = null;
  private DcMotor driveSW = null;
  private DcMotor slider = null;
  private Servo grabber = null;

  double driveTwd = 0;
  double driveFwd = 0;

  boolean isGrabbing = false;

  private void driveTwd(double force,int time){

    // drive toward bridge

    if(isRed){
      force = -force;
    }

    driveNW.setPower( force);
    driveNE.setPower( force);
    driveSE.setPower(-force);
    driveSW.setPower(-force);

    sleep(time);

  }

  private void driveFwd(double force,int time){

    // drive toward stones

    driveNW.setPower(-force);
    driveNE.setPower( force);
    driveSE.setPower( force);
    driveSW.setPower(-force);

    sleep(time);

  }

  private void driveSpn(double force,int time){

    // spin clockwise red, cc blue

    if(isRed){
      force = -force;
    }

    driveNW.setPower( force);
    driveNE.setPower( force);
    driveSE.setPower( force);
    driveSW.setPower( force);

    sleep(time);

  }

  private void driveRst(){

    driveFwd(0,0);

  }

  private void grab(boolean shouldGrab){

    if(shouldGrab){
      grabber.setPosition(0.7);
    }else{
      grabber.setPosition(0);
    }

    sleep(1000);

  }

  @Override
  public void runOpMode() {

    driveNW = hardwareMap.get( DcMotor.class, "driveNW" );
    driveNE = hardwareMap.get( DcMotor.class, "driveNE" );
    driveSE = hardwareMap.get( DcMotor.class, "driveSE" );
    driveSW = hardwareMap.get( DcMotor.class, "driveSW" );
    slider  = hardwareMap.get( DcMotor.class, "slider"  );
    grabber = hardwareMap.get( Servo.class  , "grabber" );

    waitForStart();
    runtime.reset();

    driveFwd(1,1100);
    
    driveRst();

  }
}
