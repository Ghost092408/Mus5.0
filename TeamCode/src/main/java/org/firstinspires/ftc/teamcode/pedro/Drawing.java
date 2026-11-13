package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.math.Pose;

import java.util.ArrayList;
import java.util.List;

public class Drawing {
   private static final double ROBOT_RADIUS = 7.87;
   private static final double FIELD_CENTER = 7.87;
   private static final double TRAIL_STEP = 0.25;
   private static final int MAX_TRAIL = 400;
   
   private static final FieldManager field = PanelsField.INSTANCE.getField();
   private static final Style robotLook = new Style("", "#3F51B5", 0.75);
   private static final Style trailLook = new Style("", "#4CAF50", 0.75);
   private static final List<Pose> trail = new ArrayList<>();
   
   public static void init() {
      field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
      trail.clear();
   }
   
   public static void draw(Pose pose) {
      Pose p = new Pose(pose.x() + FIELD_CENTER, pose.y() + FIELD_CENTER, pose.heading());
      
      if (trail.isEmpty() || trail.get(trail.size() - 1).distance(p) > TRAIL_STEP) {
         trail.add(p);
         if (trail.size() > MAX_TRAIL) trail.remove(0);
      }
      
      field.setStyle(trailLook);
      for (int i = 0; i < trail.size() - 1; i++) {
         field.moveCursor(trail.get(i).x(), trail.get(i).y());
         field.line(trail.get(i + 1).x(), trail.get(i + 1).y());
      }
      
      field.setStyle(robotLook);
      field.moveCursor(p.x(), p.y());
      field.circle(ROBOT_RADIUS);
      field.line(p.x() + ROBOT_RADIUS * Math.cos(p.heading()), p.y() + ROBOT_RADIUS * Math.sin(p.heading()));
      
      field.update();
   }
   
   public static void drawPoint(double x, double y) {
      Pose p = new Pose(x + FIELD_CENTER, y + FIELD_CENTER, 0);
      
      if (trail.isEmpty() || trail.get(trail.size() - 1).distance(p) > TRAIL_STEP) {
         trail.add(p);
         if (trail.size() > MAX_TRAIL) trail.remove(0);
      }
      
      field.setStyle(trailLook);
      for (int i = 0; i < trail.size() - 1; i++) {
         field.moveCursor(trail.get(i).x(), trail.get(i).y());
         field.line(trail.get(i + 1).x(), trail.get(i + 1).y());
      }
      
      field.setStyle(robotLook);
      field.moveCursor(p.x(), p.y());
      field.circle(1);
      
      field.update();
   }
}