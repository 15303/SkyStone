package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="OmniAuto-R-0", group="Linear Opmode")
//@Disabled

public class OmniOpModeR0 extends LinearOpMode {

  boolean isRed = true; //IMPORTANT

  private ElapsedTime runtime = new ElapsedTime();

  private DcMotor         driveNW = null;
  private DcMotor         driveNE = null;
  private DcMotor         driveSE = null;
  private DcMotor         driveSW = null;

  private DcMotor         slider  = null;
  private Servo           grabber = null;

  private ColorSensor     color   = null;

  private DistanceSensor  distN   = null;
  private DistanceSensor  distE   = null;
  private DistanceSensor  distS   = null;
  private DistanceSensor  distW   = null;

  double distN = 30;
  double distX = 0;
  double distS = 100;

  double lumin = 1;

  double time = 0;
  double timeCache = 0;

  boolean shouldGrab = false;

  private void driveX(double force){

    // drive away from drivers

    if(isRed){
      force = -force;
    }

    driveNW.setPower( force);
    driveNE.setPower( force);
    driveSE.setPower(-force);
    driveSW.setPower(-force);

  }

  private void driveY(double force){

    // drive toward stones

    driveNW.setPower(-force);
    driveNE.setPower( force);
    driveSE.setPower( force);
    driveSW.setPower(-force);

  }

  private void driveSpn(double force){

    // spin clockwise red, cc blue

    if(isRed){
      force = -force;
    }

    driveNW.setPower( force);
    driveNE.setPower( force);
    driveSE.setPower( force);
    driveSW.setPower( force);

  }

  private void raiseSlider(int level){

    if(level>0){
      slider.setPower(1);
    }else{
      slider.setPower(-1);
    }

    sleep(700);

    slider.setPower(0);

  }

  private void grab(){

    if(shouldGrab){
      grabber.setPosition(0.7);
    }else{
      grabber.setPosition(0);
    }

  }

  private void update(){
    grab();
    time = getRuntime();
    distN = ( distN + senseDistN.getDistance(DistanceUnit.CM) ) / 2;
    distX = ( distX + senseDistX.getDistance(DistanceUnit.CM) ) / 2;
    distS = ( distS + senseDistS.getDistance(DistanceUnit.CM) ) / 2;
    lumin = ( lumin + (senseColor.red()+senseColor.green()+senseColor.blue()) / 3 ) / 2;
  }

  @Override
  public void runOpMode() {

    driveNW    = hardwareMap.get( DcMotor.class,       "driveNW" );
    driveNE    = hardwareMap.get( DcMotor.class,       "driveNE" );
    driveSE    = hardwareMap.get( DcMotor.class,       "driveSE" );
    driveSW    = hardwareMap.get( DcMotor.class,       "driveSW" );
    slider     = hardwareMap.get( DcMotor.class,       "slider"  );
    grabber    = hardwareMap.get( Servo.class  ,       "grabber" );
    senseColor = hardwareMap.get(ColorSensor.class,    "color"   );
    senseDistN = hardwareMap.get(DistanceSensor.class, "distN"   );
    senseDistX = hardwareMap.get(DistanceSensor.class, "distX"   );
    senseDistS = hardwareMap.get(DistanceSensor.class, "distS"   );

    waitForStart();
    runtime.reset();

    update();


    while( //drive forward until color sensor reads black

      opModeIsActive() &&
      distN > 5 &&
      lumin < 0.3

    ){

      driveY(0.5);
      update();

    }


    timeCache = time;

    while( //drive back for .2s

      opModeIsActive() &&
      time - timeCache < 200

    ){

      driveY(-0.5);
      update();

    }


    while( //drive sideways to stone depot

      opModeIsActive() &&
      distX < 80

    ){

      driveX(0.5);
      update();

    }


    timeCache = time;

    while( //drive forward for .2s

      opModeIsActive() &&
      time - timeCache < 200

    ){

      driveY(0.5);
      update();

    }


    shouldGrab = true;
    timeCache = time;


    while( //wait 1s for grabber to close

      opModeIsActive() &&
      time - timeCache < 1000

    ){

      update();

    }


    while( //drive sideways to inner track

      opModeIsActive() &&
      distX > 50

    ){

      driveX(-0.5);
      update();

    }


    while( //drive back until centered on foundation's long side

      opModeIsActive() &&
      distS > 20

    ){

      driveY(-0.5);
      update();

    }


    //deploy foundation grabber

    while( //drive sideways until touching wall

      opModeIsActive() &&
      distX > 5

    ){

      driveX(-0.5);
      update();

    }

    //release foundation grabber


    while( //drive forward until clear of foundation

      opModeIsActive() &&
      distS < 100

    ){

      driveY(0.5);
      update();

    }


    while( //drive sideways until centered on foundation's short side

      opModeIsActive() &&
      distX < 100

    ){

      driveX(0.5);
      update();

    }

    //spin

    //drive "forward" until touching foundation's short side

    raiseSlider(1);

    shouldGrab = false;

    //drive "backward"

    raiseSlider(-1);

    //spin

    //drive sideways to inner track

    //drive forward to under bridge


    driveX(0);

  }
}