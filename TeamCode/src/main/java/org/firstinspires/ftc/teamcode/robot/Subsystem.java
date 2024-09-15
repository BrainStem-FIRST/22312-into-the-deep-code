public class Subsystem {

    private HardwareMap hwMap;
    private Telemetry telemetry;

    public Subsystem(HardwareMap hwMap, Telemetry telemetry) {
        this.hwMap = hwMap;
        this.telemetry = telemetry;
    }

}