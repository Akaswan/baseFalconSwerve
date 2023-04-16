package frc.robot.commands.SwerveDrivebase;

import frc.robot.subsystems.SwerveDrive;
import frc.robot.utilities.CreateEventMap;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*
 * <h3>CreatePath<h3>
 * 
 */
public class CreatePath extends SequentialCommandGroup {

    public static HashMap<String, Command> eventMap = new HashMap<>();
    
    /**
     * <h3>CreatePath</h3>
     * 
     * adding path constraints and builds auto command
     * 
     * @param preCommand a command that runs before the path, use command groups to run multiple commands
     * @param m_drivebase the required subsystem
     * @param pathName name of path (pathPlanner's path)
     * @param maxVelocity max velocity of path
     * @param maxAcceleration max acceleration of path
     * @param postCommand a command that is run after the path completes, use command groups to run multiple commands
     */
    public CreatePath(Command preCommand, SwerveDrive m_drivebase, String pathName, double maxVelocity, double maxAcceleration, Command postCommand) {
        addRequirements(m_drivebase);

        CreateEventMap createMap = new CreateEventMap(m_drivebase);

        eventMap = createMap.createMap();

        List<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(pathName, 
             false, new PathConstraints(maxVelocity, maxAcceleration));

             SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
                m_drivebase::getPose, 
                m_drivebase::resetOdometry, 
                SwerveDrive.kDriveKinematics, 
                new PIDConstants(3.5, 0.25, 0), // Translation
                new PIDConstants(3.2, 0, 0), // Rotation
                m_drivebase::setSwerveModuleStates, 
                eventMap, 
                true, 
                m_drivebase
              );

        // creates a command based on the path with post and pre commands added
        Command pathCommand = autoBuilder.fullAuto(path);
        if (preCommand != null) {
            addCommands(preCommand);
        }
        addCommands(
            pathCommand
        );
        if (postCommand != null) {
            addCommands(postCommand);
        }
    }
}