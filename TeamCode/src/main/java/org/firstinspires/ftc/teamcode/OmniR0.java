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

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="OmniR0", group="Linear Opmode")
//@Disabled

public class OmniR0 extends LinearOpMode {

  boolean isRed = true; //IMPORTANT

  private ElapsedTime runtime = new ElapsedTime();

  private DcMotor         driveNW = null;
  private DcMotor         driveNE = null;
  private DcMotor         driveSE = null;
  private DcMotor         driveSW = null;

  private DcMotor         slider  = null;
  private Servo           grabber = null;

  private ColorSensor     color   = null;

  private DistanceSensor  senseDistN   = null;
  private DistanceSensor  sensedistX   = null;
  private DistanceSensor  senseDistS   = null;

  double distN[] = {30,    30, 30, 30, 30, 30, 30, 30, 30 };
  double distX[] = {0,     0,  0,  0,  0,  0,  0,  0,  0  };
  double distS[] = {100,   100,100,100,100,100,100,100,100};

  static double DIST_WALL                 = 5;
  static double DIST_Y_SKYBRIDGE          = 60;
  static double DIST_X_INTRACK_OUTER      = 25;
  static double DIST_X_DEPOT_CENTER       = 40;
  static double DIST_X_DEPOT_OUTER        = 20;
  static double DIST_X_FOUNDATION_CENTER  = 35;
  static double DIST_Y_FOUNDATION_CENTER  = 15;
  static double DIST_Y_FOUNDATION_OUTER   = 40;

  static int optLuminG =  0;
  static int optDistNL = -1;
  static int optDistNG =  1;
  static int optdistXL = -2;
  static int optdistXG =  2;
  static int optDistSL = -3;
  static int optDistSG =  3;
  static int optTimeL  = -4;

  double lumin[] = {1,     1,1,1,1,1,1,1,1};
  static double LUMIN_THRESHOLD = 0.3;

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

  private void sliderSpn(double force){

    slider.setPower(force);

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

    distN[0] = 0;

    for(int i = 1; i < 7; i++){
      distN[i] = distN[i+1];
      distN[0]+= distN[i];

      distX[i] = distX[i+1];
      distX[0]+= distX[i];

      distS[i] = distS[i+1];
      distS[0]+= distS[i];

      lumin[i] = lumin[i+1];
      lumin[0]+= lumin[i];
    }

    distN[8] = senseDistN.getDistance(DistanceUnit.CM);
    distN[0]+= distN[8];
    distN[0]/= 8;

    distX[8] = senseDistX.getDistance(DistanceUnit.CM);
    distX[0]+= distN[8];
    distX[0]/= 8;

    distS[8] = senseDistS.getDistance(DistanceUnit.CM);
    distS[0]+= distN[8];
    distS[0]/= 8;

    lumin[8] = senseColor.red()+senseColor.green()+senseColor.blue();
    lumin[0]+= lumin[8];
    lumin[0]/= 8;

    telemetry.addData("lumin", lumin[0]);
    telemetry.addData("distN", distN[0]);
    telemetry.addData("distX", distX[0]);
    telemetry.addData("distS", distS[0]);
    telemetry.update();

  }

  private void runWhile(int option, double comparator){

    switch(option){

      case 0:
        while(opModeIsActive() && lumin[0] > comparator){
          update();
        }
      break;

      case -1:
        while(opModeIsActive() && distN[0] < comparator){
          update();
        }
      break;

      case 1:
        while(opModeIsActive() && distN[0] > comparator){
          update();
        }
      break;

      case -2:
        while(opModeIsActive() && distX[0] < comparator){
          update();
        }
      break;

      case 2:
        while(opModeIsActive() && distX[0] > comparator){
          update();
        }
      break;


      case -3:
        while(opModeIsActive() && distS[0] < comparator){
          update();
        }
      break;

      case 3:
        while(opModeIsActive() && distS[0] > comparator){
          update();
        }
      break;

      case -4:
        while(opModeIsActive() && getRuntime() < comparator){
          update();
        }
      break;

    }

  }

  private void runFor(double duration){

    runtime.reset();
    runWhile(optTimeL, duration);

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
    sensedistS = hardwareMap.get(DistanceSensor.class, "distX"   );
    senseDistS = hardwareMap.get(DistanceSensor.class, "distS"   );

    waitForStart();
    runtime.reset();

    driveX(0.5);
    runWhile(optdistSL, DIST_X_DEPOT_OUTER);

    driveY(0.5);
    runWhile(optLuminG, LUMIN_THRESHOLD);


    //drive back for .2s
    driveY(-0.5);
    runFor(200);

    //drive sideways to stone depot
    driveX(0.5);
    runWhile(optdistSL, DIST_X_DEPOT_CENTER);

    //drive forward for .2s
    driveY(0.5);
    runFor(200);
    driveY(0);

    //grab
    shouldGrab = true;
    //wait 1s for grabber to close
    runFor(1000);

    //drive sideways to inner track
    driveX(-0.5);
    runWhile(optdistSG, DIST_X_INTRACK_OUTER);

    //drive back until centered on foundation's long side
    driveY(-0.5);
    runWhile(optDistSG, DIST_Y_FOUNDATION_CENTER);
    driveY(0);


    //deploy foundation grabber
    //

    //drive sideways until touching wall
    driveX(-0.5);
    runWhile(optdistSG, DIST_WALL);
    driveX(0);

    //release foundation grabber
    //

    //drive forward until clear of foundation
    driveY(0.5);
    runWhile(optDistSL, DIST_Y_FOUNDATION_OUTER);

    //drive sideways until centered on foundation's short side
    driveX(0.5);
    runWhile(optdistSL, DIST_X_FOUNDATION_CENTER);

    //spin 180deg
    driveSpn(1);
    runFor(500);

    //raise slider
    sliderSpn(1);
    runFor(200);
    sliderSpn(0);
    driveSpn(0);

    //drive "forward" until touching foundation's short side
    driveY(0.5);
    runFor(500);

    //release
    shouldGrab = false;

    //drive "backward"
    driveY(-0.5);
    runFor(500);

    shouldGrab = true;
    sliderSpn(-1);
    driveSpn(-1);
    runFor(200);
    sliderSpn(0);
    runFor(500);
    driveSpn(0);


    //drive sideways to inner track

    driveX(-0.5);
    runWhile(optdistSG, DIST_X_INTRACK_OUTER);

    //drive forward to under bridge

    driveY(0.5);
    runWhile(optDistSL, DIST_Y_SKYBRIDGE);

    driveX(0);

  }
}