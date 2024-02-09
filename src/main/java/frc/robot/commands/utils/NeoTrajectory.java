
package frc.robot.commands.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.*;
import frc.robot.subsystems.swerve.SwerveDrive;
import java.util.Optional;
import java.util.ArrayList;
import java.util.List;

public class NeoTrajectory {

    
    private static Pose2d lastEnd = new Pose2d(0, 0, new Rotation2d(0));
    private List<List<Pose2d>> waypointGroups = new ArrayList<>();
    private List<TrajectoryConfig> configs = new ArrayList<>();
    private List<Pose2d> currentWaypoints = new ArrayList<>();
    private boolean isReversed;

    public NeoTrajectory(){
        addNewConfig();
        addWaypoint(0, 0, 0);
    }

    public NeoTrajectory(double x, double y, double rot) {
        addNewConfig();
        addWaypoint(x, y, rot);
    }
    public NeoTrajectory(double x, double y, double rot, TrajectoryConfig config) {
        configs.add(config);
        addWaypoint(x, y, rot);
    }

    private void addNewConfig() {
        TrajectoryConfig config = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(DriveConstants.kDriveKinematics)
            .setReversed(isReversed);
        configs.add(config);
    }

    public void addWaypoint(double x, double y, double rot) {
        Rotation2d rotation = new Rotation2d(Math.toRadians(rot));
        addWaypoint(x, y, rotation);
    }

    public void addWaypoint(double x, double y) {
        Rotation2d rotation;
        if (!currentWaypoints.isEmpty()) {
            Pose2d lastWaypoint = currentWaypoints.get(currentWaypoints.size() - 1);
            rotation = new Rotation2d(x - lastWaypoint.getTranslation().getX(), y - lastWaypoint.getTranslation().getY());
            boolean newIsReversed = x < lastWaypoint.getTranslation().getX();
            if (newIsReversed != isReversed) {
                waypointGroups.add(currentWaypoints);
                currentWaypoints = new ArrayList<>();
                isReversed = newIsReversed;
                addNewConfig();
            }
        } else {
            rotation = new Rotation2d(0);
        }
        addWaypoint(x, y, rotation);
    }

    private void addWaypoint(double x, double y, Rotation2d rotation) {
        Pose2d waypoint = new Pose2d(x, y, rotation);
        currentWaypoints.add(waypoint);
        lastEnd = waypoint;
    }

    public List<Trajectory> generateTrajectories() {
        waypointGroups.add(currentWaypoints);
        List<Trajectory> trajectories = new ArrayList<>();
        if (waypointGroups.size() != configs.size()) {
            // Handle the size mismatch appropriately
            System.err.println("Size of waypointGroups and configs do not match");
            return trajectories;
        }
        for (int i = 0; i < waypointGroups.size(); i++) {
            trajectories.add(TrajectoryGenerator.generateTrajectory(waypointGroups.get(i), configs.get(i)));
        }
        return trajectories;
    }

    public Pose2d getStartingWaypointInitialPose() {
        if (waypointGroups.isEmpty() || waypointGroups.get(0).isEmpty()) {
            // Handle the empty list(s) appropriately
            System.err.println("WaypointGroups is empty or the first list in waypointGroups is empty");
            return new Pose2d(0, 0, new Rotation2d(0)); // Return a default pose
        }
        return waypointGroups.get(0).get(0);
    }

    public SequentialCommandGroup generateCommand(SwerveDrive m_robotDrive) {
        List<Trajectory> trajectories = this.generateTrajectories();
        SequentialCommandGroup commandGroup = new SequentialCommandGroup();
        for (Trajectory trajectory : trajectories) {
            SwerveControllerCommand command = new SwerveControllerCommand(
                trajectory,
                m_robotDrive::getPose,
                DriveConstants.kDriveKinematics,
                // Position controllers
                new PIDController(AutoConstants.kPXController, AutoConstants.kIXController, AutoConstants.kDXController),
                new PIDController(AutoConstants.kPYController, AutoConstants.kIYController, AutoConstants.kDYController),
                new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController,
                    AutoConstants.kThetaControllerConstraints),
                () -> lastEnd.getRotation(),
                m_robotDrive::setModuleStates,
                m_robotDrive
            );
            commandGroup.addCommands(command);
        }
        return commandGroup;
    }
}