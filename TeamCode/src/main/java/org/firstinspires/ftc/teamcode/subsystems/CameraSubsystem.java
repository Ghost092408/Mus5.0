package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.apriltag.AprilTagDetection;
import com.qualcomm.hardware.limelightvision.LLResult;
public class CameraSubsystem {
    private final Limelight3A camera;

    boolean vedebile = false;
    double directie = 0;
    int nrbile = 0 ;
    int bilerobot = 0 ;
    public CameraSubsystem(HardwareMap hardwareMap) {
         camera = hardwareMap.get(Limelight3A.class,"LimeLight");
    }

    public void alegcebilevreau (int x) { // 0 este pentru cele albastre 1 pentru cele rosii 2 este pentru apritaguri

        camera.pipelineSwitch(x);

    }
    public void porneste() {
        camera.start();
    }
    public void update()
    {
        vedebile = false;
        directie = 0;
        nrbile = 0;
        bilerobot = 0;

        LLResult rezultat  = camera.getLatestResult();
        if ( rezultat == null) {
            return ;
        }
        double [] date = rezultat.getPythonOutput ();
        vedebile = date[0] == 1;
        directie = date[1];
        nrbile = (int) date[2];
        bilerobot = (int) date [8];

    }

    public void start() {
        camera.start();
    }
    public boolean VedeBile() {
        return vedebile;
    }
    public double Directie() {
        return directie;
    }
    public int NrBile() {
        return nrbile;
    }
    public int BileRobot() {
        return bilerobot;
    }
    public void reseteazaBileRobot() { bilerobot = 0; }
}

