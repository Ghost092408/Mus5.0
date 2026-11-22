package org.firstinspires.ftc.teamcode.teleOps.teleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import org.firstinspires.ftc.teamcode.commands.Launch;
import org.firstinspires.ftc.teamcode.commands.Spit;
import org.firstinspires.ftc.teamcode.commands.Suck;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

@TeleOp (name = "TeleOpOf")
@Configurable
public class TeleOpOf extends OpMode {
   private TelemetryManager telemetryM;
   
   private Follower follower;
   
   private GamepadEx gamepad;
   
   private IntakeSubsystem intake;
   private TurretSubsystem turret;
   private CameraSubsystem vedere;
   
   @Override
   public void init() {
      telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
      
      follower = Constants.create(hardwareMap);
      follower.update();
      
      gamepad = new GamepadEx(gamepad1);
      
      //subsysteme
      {
         intake = new IntakeSubsystem(hardwareMap);
         turret = new TurretSubsystem(hardwareMap);
         vedere = new CameraSubsystem(hardwareMap); //camera
      }
      
      //buttoane
     new GamepadButton(gamepad, GamepadKeys.Button.CROSS)
        .toggleWhenPressed( new Suck(intake) );
      new GamepadButton(gamepad, GamepadKeys.Button.CIRCLE)
         .toggleWhenPressed( new Spit(intake) );
     new GamepadButton(gamepad, GamepadKeys.Button.DPAD_UP)
        .toggleWhenPressed( new Launch(turret, intake) );

      // camera
      vedere.alegcebilevreau(0);
      vedere.start();
   }

   @Override
   public void loop() {
      vedere.update(); // camera
      follower.update();
      telemetryM.update();
      // camera
      if (vedere.BileRobot() == 4)
      {
         gamepad1.setLedColor(0.0, 0.0, 1.0, Gamepad.LED_DURATION_CONTINUOUS);
      }

      else
      {
         gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
      }
      if (vedere.VedeBile()  && vedere.BileRobot() <4 )
      {
         double y = 0;
         double x = 0;
         double rx = 0;
         double directie = vedere.Directie();

            if (Double.isFinite(directie)) {
               x = Math.max(-0.25, Math.min(0.25, directie * 0.25));

               if (Math.abs(directie) < 0.2) {
                  y = 0.2;
               }
            }
         }
      else if (vedere.BileRobot()==4)
      {
         /*if (poz robot == zona in care puteam da shoot )
          {  transfer.setPower(1);
            copiat tot shootingul la turret odata ce gata ;
           }
         else
         {
         cu pedro pahting ceva sa mergem pe partea buna a terenului;
         transfer.setPower(1);
         alea shooter din nou;

         }
         vedere.BileRobot()=0;
         gamepad1.setLedColor(1.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);

         */
      }


      
      CommandScheduler.getInstance().run();
      
      //miscare - robot centric
      follower.manual(
         -gamepad1.left_stick_y,
         -gamepad1.left_stick_x,
         -gamepad1.right_stick_x
      );
      
      Pose pose = follower.pose();
      
      //telemetrie
      telemetryM.debug("x: ", pose.x());
      telemetryM.debug("y: ", pose.y());
      telemetryM.debug("heading: ", Math.toDegrees(pose.heading()));
      telemetryM.update(telemetry);
   }
}
