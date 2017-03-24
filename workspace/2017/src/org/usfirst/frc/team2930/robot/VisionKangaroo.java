package org.usfirst.frc.team2930.robot;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.DriverStation;

public class VisionKangaroo {
	public boolean ComputedPosition;
	public boolean DetectedTwoCentroids;
	public double[] Translation;
	private SerialPort RS232Kangaroo;
	
	public VisionKangaroo() {
		RS232Kangaroo = new SerialPort(9600, SerialPort.Port.kOnboard);
		RS232Kangaroo.enableTermination();
		ComputedPosition = false;
		DetectedTwoCentroids = false;
		Translation = new double[3];
		Translation[0] = 0;
		Translation[1] = 0;
		Translation[2] = 0;
	}
	
	public void Update() {
		String receivedString = null;
		while (RS232Kangaroo.getBytesReceived() > 0) {
			receivedString = RS232Kangaroo.readString();
		}
		if (receivedString != null) {
			String[] splitReceivedString = receivedString.split(";");
			if (splitReceivedString.length == 5) {
				try {
					ComputedPosition = Boolean.parseBoolean(splitReceivedString[0]);
					DetectedTwoCentroids = Boolean.parseBoolean(splitReceivedString[1]);
					Translation[0] = Double.parseDouble(splitReceivedString[2]);
					Translation[1] = Double.parseDouble(splitReceivedString[3]);
					Translation[2] = Double.parseDouble(splitReceivedString[4]);
				}
				catch (Exception e)
				{
					DriverStation.reportError(
							String.format("Problem parsing serial received string: %s", e.getMessage()),
							true
							);
				}
			}
			else {
				DriverStation.reportError("Didn't receive correct info from VisionKangaroo", false);
			}
		}
		else {
			ComputedPosition = false;
			DetectedTwoCentroids = false;
			Translation[0] = 0;
			Translation[1] = 0;
			Translation[2] = 0;
		}
	}
}
