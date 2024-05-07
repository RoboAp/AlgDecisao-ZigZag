#include <QDebug>
#include <QtNetwork>
#include "ssl-client/ssl-client.h"

void printRobotInfo(const SSL_DetectionRobot& robot) {
  qDebug("CONF=%4.2f ", robot.confidence());
  if (robot.has_robot_id()) {
    qDebug("ID=%3d ", robot.robot_id());
  } else {
    qDebug("ID=N/A ");
  }
  qDebug(" HEIGHT=%6.2f POS=<%9.2f,%9.2f> ",
         robot.height(),
         robot.x(),
         robot.y());
  if (robot.has_orientation()) {
    qDebug("ANGLE=%6.3f ", robot.orientation());
  } else {
    qDebug("ANGLE=N/A    ");
  }
  qDebug("RAW=<%8.2f,%8.2f>\n", robot.pixel_x(), robot.pixel_y());
}

// Função do gradiente descendente da atração
float gradUAtt(float Katt, float q, float q_goal){
    return Katt*(q-q_goal);
}

// Função do gradiente descendente da repulsão
float gradURep(float Krep, float q, float q_obs, float dist, float ro_zero){
    if(dist > ro_zero){
        return 0;
    } else {
        return Krep*(1/dist - 1/ro_zero)*(1/pow(dist,2))*(q-q_obs)/(dist);
    }
}

// Função para definir a velocidade do robô
float getVelocity(float force, float mass, float time){
    return (force*time/mass);
}




int main(int argc, char* argv[]) {
  (void) argc;
  (void) argv;

  float pos_yellow_y = 0;
  float pos_yellow_x = 0;
  float pos_blue_y = 0;
  float pos_blue_x = 0;
  float euc_dist = 0;
  float dots_ref[6][2] = {{0,0},{100,0},{100,200},{300,200},{500,500},{800,900}};
  float goal_dist;
  int state = 0;
  int pos_x_final = dots_ref[state][0];
  int pos_y_final = dots_ref[state][1];


  RoboCupSSLClient client(10020, "224.5.23.2");

  client.open(true);
  SSL_WrapperPacket packet;

  GrSim_Client_Example grSim_client;
  grSim_client.setPortAndAddress(20011, "127.0.0.1");

  while (true) {
    // grSim_client.sendCommand(1.0, 0);
    if (client.receive(packet)) {
      // qDebug(
      //     "-----Received Wrapper Packet---------------------------------------------\n");
      // see if the packet contains a robot detection frame:
      if (packet.has_detection()) {
        SSL_DetectionFrame detection = packet.detection();
        // Display the contents of the robot detection results:
        double t_now = GetTimeSec();

        int balls_n = detection.balls_size();
        int robots_blue_n = detection.robots_blue_size();
        int robots_yellow_n = detection.robots_yellow_size();
        // qDebug("Number blue: %2d", robots_blue_n);
        // qDebug("Number yellow: %2d", robots_yellow_n);

        // // Ball info:
        // for (int i = 0; i < balls_n; i++) {
        //   SSL_DetectionBall ball = detection.balls(i);
        //   qDebug("-Ball (%2d/%2d): CONF=%4.2f POS=<%9.2f,%9.2f> ",
        //          i + 1,
        //          balls_n,
        //          ball.confidence(),
        //          ball.x(),
        //          ball.y());
        //   if (ball.has_z()) {
        //     qDebug("Z=%7.2f ", ball.z());
        //   } else {
        //     qDebug("Z=N/A   ");
        //   }
        //   qDebug("RAW=<%8.2f,%8.2f>\n", ball.pixel_x(), ball.pixel_y());
        // }

        // Blue robot info:
        for (int i = 0; i < robots_blue_n; i++) {
          SSL_DetectionRobot robot = detection.robots_blue(i);

          // conseguir o mínimo global
          pos_blue_x = robot.x();
          pos_blue_y = robot.y();

          euc_dist = abs(sqrt(pow(pos_blue_x - pos_yellow_x,2)+pow(pos_blue_y - pos_yellow_y,2)));
          goal_dist = abs(sqrt(pow(pos_blue_x - pos_x_final,2)+pow(pos_blue_y - pos_y_final,2)));
          qDebug("Goal dist: %.2f",goal_dist);
          if(goal_dist <= 5){
              state++;
              if(state == 6){
                  state = 0;
              }
          }

          qDebug("State: %d",state);

          int pos_x_final = dots_ref[state][0];
          int pos_y_final = dots_ref[state][1];

          float min_glob_x = -gradUAtt(0.2,pos_blue_x,pos_x_final) +gradURep(0.5,pos_blue_x,pos_yellow_x,euc_dist,100);
          float min_glob_y = -gradUAtt(0.2,pos_blue_y,pos_y_final) +gradURep(0.5,pos_blue_y,pos_yellow_y,euc_dist,100);

          //qDebug("Minimo Global: X=%.2f Y=%.2f",min_glob_x,min_glob_y);
          float vel_x = getVelocity(min_glob_x,5,0.01);
          float vel_y = getVelocity(min_glob_y,5,0.01);
          //qDebug("Velocidades: X=%.2f Y=%.2f",vel_x,vel_y);
          grSim_client.sendCommand(vel_x,vel_y, i);

        }



        // Yellow robot info:
        for (int i = 0; i < robots_yellow_n; i++) {
          SSL_DetectionRobot robot = detection.robots_yellow(i);
          // qDebug("-Robot(Y) (%2d/%2d): ", i + 1, robots_yellow_n);
          // printRobotInfo(robot);
          pos_yellow_x = robot.x();
          pos_yellow_y = robot.y();
          euc_dist = abs(sqrt(pow(pos_blue_x - pos_yellow_x,2)+pow(pos_blue_y - pos_yellow_y,2)));
          qDebug("Pos Yellow: X=%.2f Y=%.2f", pos_yellow_x,pos_yellow_y);
        }
      }

      // see if packet contains geometry data:
      if (packet.has_geometry()) {
        const SSL_GeometryData& geom = packet.geometry();
        qDebug("-[Geometry Data]-------\n");

        const SSL_GeometryFieldSize& field = geom.field();
        qDebug("Field Dimensions:\n");
        qDebug("  -field_length=%d (mm)\n", field.field_length());
        qDebug("  -field_width=%d (mm)\n", field.field_width());
        qDebug("  -boundary_width=%d (mm)\n", field.boundary_width());
        qDebug("  -goal_width=%d (mm)\n", field.goal_width());
        qDebug("  -goal_depth=%d (mm)\n", field.goal_depth());
        qDebug("  -field_lines_size=%d\n", field.field_lines_size());
        qDebug("  -field_arcs_size=%d\n", field.field_arcs_size());

        int calib_n = geom.calib_size();
        for (int i = 0; i < calib_n; i++) {
          const SSL_GeometryCameraCalibration& calib = geom.calib(i);
          qDebug("Camera Geometry for Camera ID %d:\n", calib.camera_id());
          qDebug("  -focal_length=%.2f\n", calib.focal_length());
          qDebug("  -principal_point_x=%.2f\n", calib.principal_point_x());
          qDebug("  -principal_point_y=%.2f\n", calib.principal_point_y());
          qDebug("  -distortion=%.2f\n", calib.distortion());
          qDebug("  -q0=%.2f\n", calib.q0());
          qDebug("  -q1=%.2f\n", calib.q1());
          qDebug("  -q2=%.2f\n", calib.q2());
          qDebug("  -q3=%.2f\n", calib.q3());
          qDebug("  -tx=%.2f\n", calib.tx());
          qDebug("  -ty=%.2f\n", calib.ty());
          qDebug("  -tz=%.2f\n", calib.tz());

          if (calib.has_derived_camera_world_tx() &&
              calib.has_derived_camera_world_ty() &&
              calib.has_derived_camera_world_tz()) {
            qDebug("  -derived_camera_world_tx=%.f\n",
                   calib.derived_camera_world_tx());
            qDebug("  -derived_camera_world_ty=%.f\n",
                   calib.derived_camera_world_ty());
            qDebug("  -derived_camera_world_tz=%.f\n",
                   calib.derived_camera_world_tz());
          }
        }
      }
    }
  }

  return 0;
}
