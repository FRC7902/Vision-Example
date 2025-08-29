import edu.wpi.first.hal.SerialPortJNI;

public class LED {
    private final int port;
    SerialPortJNI serial;

    public LED(int port) {
        this.port = port;
    }

    public void sendStripData(int stripNumber, int brightness, int mode,
                              int r, int g, int b) {

        byte[] packet = new byte[6];
        packet[0] = (byte) stripNumber;
        packet[1] = (byte) brightness;
        packet[2] = (byte) mode;
        packet[3] = (byte) r; 
        packet[4] = (byte) g;
        packet[5] = (byte) b;

        SerialPortJNI.serialWrite(port, packet, packet.length);
    }
}