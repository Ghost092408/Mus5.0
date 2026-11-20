package org.firstinspires.ftc.teamcode.teleOps.debug;

import static org.firstinspires.ftc.teamcode.pedro.Drawing.ROBOT_RADIUS;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.pedroCommand.TurnToCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.Drawing;

import java.util.List;

@TeleOp
@Configurable
public class Test extends LinearOpMode {
   public static double turretSpeed = 0;
   public static double rampPos = 0.04;
   
   //test localizare
   public static double x2 = 60 - ROBOT_RADIUS;
   public static double y2 = 59 - ROBOT_RADIUS;
   private static double angle = -1;
   
   //valori PIDF tureta
   public static double kP = 0.0032;
   public static double kI = 0;
   public static double kD = 0;
   public static double kF = 0.00049;
   
   public void runOpMode() throws InterruptedException {
      TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
      
      Command turnCommand = null;
      
      DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "stangaFata");
      DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "dreaptaFata");
      DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "stangaSpate");
      DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "dreaptaSpate");
      DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
      DcMotorEx transfer = hardwareMap.get(DcMotorEx.class, "transfer");
      DcMotorEx turret = hardwareMap.get(DcMotorEx.class, "turret");
      
      ServoImplEx rampaAng = hardwareMap.get(ServoImplEx.class, "rampAng");
      
      VoltageSensor battery = hardwareMap.voltageSensor.iterator().next();
      
      Follower follower;
      Drawing.init();
      
      follower = Constants.create(hardwareMap);
      follower.setPose(new Pose(0, 0));
      follower.update();
      
      PIDFController controller = new PIDFController(kP, kI, kD, kF);
      ElapsedTime turretTimer = new ElapsedTime();
      
      frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      
      frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
      backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
      backRight.setDirection(DcMotorSimple.Direction.REVERSE);
      turret.setDirection(DcMotorSimple.Direction.REVERSE);
      
      turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
      
      boolean intakeState = false;
      boolean intakeStateR = false;
      boolean transferState = false;
      boolean transferStateR = false;
      boolean turretState = false;
      boolean turretAtTarget = false;
      double turretRampTime = 0;
      boolean drivingState = true;
      boolean lookState = false;
      
      Gamepad currentGamepad1 = new Gamepad();
      Gamepad currentGamepad2 = new Gamepad();
      
      Gamepad previousGamepad1 = new Gamepad();
      Gamepad previousGamepad2 = new Gamepad();
      
      List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
      
      for (LynxModule hub : allHubs) {
         hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
      }
      
      waitForStart();
      
      while(opModeIsActive() && !isStopRequested()) {
         for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
         }
         
         //odometrie
         follower.update();
         Pose pose = follower.pose();
         Drawing.draw(follower.pose());
         
         // Store gamepad values from the previous loop iteration
         previousGamepad1.copy(currentGamepad1);
         previousGamepad2.copy(currentGamepad2);
         
         // Store gamepad values from this loop iteration
         currentGamepad1.copy(gamepad1);
         currentGamepad2.copy(gamepad2);
         
         //sistem de miscare
         double y = -gamepad1.left_stick_y;
         double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
         double rx = gamepad1.right_stick_x;
         
         double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
         double frontLeftPower = (y + x + rx) / denominator;
         double backLeftPower = (y - x + rx) / denominator;
         double frontRightPower = (y - x - rx) / denominator;
         double backRightPower = (y + x - rx) / denominator;
         
         //miscare
         if(previousGamepad1.touchpad && !currentGamepad1.touchpad)
            drivingState = !drivingState;
         if (drivingState && !lookState) {
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
         }
         
         rampaAng.setPosition(rampPos);
         
         //intake
         if (previousGamepad1.a && !currentGamepad1.a) {
            intakeStateR = false;
            intakeState = !intakeState;
         }
         if(intakeState)
            intake.setPower(1);
         else if(!intakeStateR)
            intake.setPower(0);
         
         if (previousGamepad1.b && !currentGamepad1.b) {
            intakeState = false;
            intakeStateR = !intakeStateR;
         }
         if(intakeStateR)
            intake.setPower(-1);
         else if(!intakeState)
            intake.setPower(0);
         
         //transfer
         if (previousGamepad1.x && !currentGamepad1.x) {
            transferStateR = false;
            transferState = !transferState;
         }
         if(transferState)
            transfer.setPower(1);
         else if(!transferStateR)
            transfer.setPower(0);
         
         if (previousGamepad1.y && !currentGamepad1.y) {
            transferState = false;
            transferStateR = !transferStateR;
         }
         if(transferStateR)
            transfer.setPower(-1);
         else if(!transferState)
            transfer.setPower(0);
         
         //turret PIDF
         if (previousGamepad1.right_bumper && !currentGamepad1.right_bumper) {
            double turretTolerance = 25;
            controller.setTolerance(turretTolerance);
            
            turretState = !turretState;
            controller.reset();
            if (turretState) {
               turretTimer.reset();
               turretAtTarget = false;
            }
         }
         if (turretState) {
            if (!controller.atSetPoint()) controller.setPIDF(kP, kI, kD, kF);
            else controller.setPIDF(0, 0, 0, kF);
            turret.setPower(controller.calculate(turret.getVelocity(), turretSpeed) * 12.0 / battery.getVoltage());
            
            //ramp-up timer: marks the moment the flywheel first reaches target speed
            if (!turretAtTarget && controller.atSetPoint()) {
               turretRampTime = turretTimer.seconds();
               turretAtTarget = true;
            }
         }
         else
            turret.setPower(0);
         
         //turret speed
         if(currentGamepad1.dpad_right && !previousGamepad1.dpad_right)
            turretSpeed += 100;
         if(currentGamepad1.dpad_left && !previousGamepad1.dpad_left)
            if(turretSpeed > 0) turretSpeed -= 100;
         
         //ramp position
         if(currentGamepad1.dpad_up && !previousGamepad1.dpad_up)
            if(rampPos < 1) rampPos += 0.01;
         if(currentGamepad1.dpad_down && !previousGamepad1.dpad_down)
            if(rampPos > 0) rampPos -= 0.01;
         
         //turn to hive
         double x1 = pose.x();
         double y1 = pose.y();
         
         if (currentGamepad1.touchpad && !previousGamepad1.touchpad) {
            lookState = !lookState;
            
            if (!lookState) {
               if (turnCommand != null) {
                  turnCommand.cancel();
                  turnCommand = null;
               }
               follower.stop();
            }
         }
         
         if (lookState) {
            angle = Math.toDegrees(Math.atan2(y2 - y1, x2 - x1));
            
            if (turnCommand == null) {
               turnCommand = new TurnToCommand(follower, angle, AngleUnit.DEGREES);
               CommandScheduler.getInstance().schedule(turnCommand);
            }
            
            Drawing.drawPoint(x2, y2);
         }
         
         //distanta
         double dist = Math.sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
         
         //telemetry
         panelsTelemetry.addData("x: ", pose.x());
         panelsTelemetry.addData("y: ", pose.y());
         panelsTelemetry.addData("heading: ", Math.toDegrees(pose.heading()));
         panelsTelemetry.addData("x2: ", x2);
         panelsTelemetry.addData("y2: ", y2);
         panelsTelemetry.addData("angle: ", angle);
         panelsTelemetry.addData("turret ramp time (s): ", turretAtTarget ? turretRampTime : "ramping...");
         panelsTelemetry.addData("ramp angle", rampPos);
         panelsTelemetry.addData("viteza tureta: ", turret.getVelocity());
         panelsTelemetry.addData("dist: ", dist);
         panelsTelemetry.update(telemetry);
      }
   }
   
}
