package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.music.Orchestra;
import frc.robot.Constants.RobotMap.CAN;
import frc.thunder.shuffleboard.LightningShuffleboard;

public class Music extends SubsystemBase {

	Orchestra orchestra;

	TalonFX motor1 = new TalonFX(CAN.LEFT_WRIST_MOTOR);
	TalonFX motor2 = new TalonFX(CAN.RIGHT_WRIST_MOTOR);
	TalonFX motor3 = new TalonFX(CAN.INSIDE_COLLECTOR_MOTOR);
	TalonFX motor4 = new TalonFX(CAN.OUTSIDE_COLLECTOR_MOTOR);

	private int currentSong = 0;
	private int songRequest = 0;

	private int timeToPlayLoops = -1;

	// An array of songs that are available to be played, can you guess the song/artists?
	String[] songs = new String[] {
			"song1.chrp", "song2.chrp", "song3.chrp", "song4.chrp", "song5.chrp", "song6.chrp",
			"song7.chrp", "song8.chrp", "song9.chrp", /*
														 * the remaining songs play better with
														 * three or more FXs
														 */
			"song10.chrp", "song11.chrp", "Megolavania.chrp"};

	/** Creates a new Music. */
	public Music() {
		ArrayList<TalonFX> instruments = new ArrayList<TalonFX>();
		instruments.add(motor1);
		instruments.add(motor2);
		instruments.add(motor3);
		instruments.add(motor4);

		orchestra = new Orchestra(instruments);

		orchestra.loadMusic(songs[currentSong]);
	}

	@Override
	public void periodic() {

		if (songRequest != currentSong) {
			// orchestra.pause();
			orchestra.loadMusic(songs[songRequest]);
			// try {
			// 	orchestra.wait(40);
			// } catch (InterruptedException e) {
			// 	// TODO Auto-generated catch block
			// 	e.printStackTrace();
			// }
			currentSong = songRequest;
			// orchestra.play();
			System.out.println("MADE IT TO NEW SONG");
		}

		// if (timeToPlayLoops > 0) {
		// --timeToPlayLoops;
		// if (timeToPlayLoops == 0) {
		// toggle();
		// }
		// }
		LightningShuffleboard.setString("Music", "Current Song", songs[currentSong]);
		LightningShuffleboard.setBool("Music", "Is playing", orchestra.isPlaying());
		LightningShuffleboard.setDouble("Music", "Current song", currentSong);
		LightningShuffleboard.setDouble("Music", "Requested song", songRequest);
		// LightningShuffleboard
	}

	public void toggle() {
		if (orchestra.isPlaying()) {
			orchestra.pause();
		} else {
			orchestra.play();
		}
	}

	public void songRequest(int request) {
		songRequest = MathUtil.clamp(request, 0, songs.length);
		timeToPlayLoops = 10;
	}

	public void nextTrack() {
		songRequest = currentSong + 1;
		if (songRequest >= songs.length) {
			songRequest = 0;
		}

		timeToPlayLoops = 10;
	}

	public void previousTrack() {
		songRequest = currentSong - 1;
		if (songRequest < 0) {
			songRequest = songs.length - 1;
		}

		timeToPlayLoops = 10;
	}



}
