public class SubsystemTele {
    protected Telemetry telemetry;
    protected HardwareMap hwMap;
    public SubsystemTele(Telemetry telemetry, HardwareMap hwMap) {
        this.telemetry = telemetry;
        this.hwMap = hwMap;
    }
    public void checkState();
}