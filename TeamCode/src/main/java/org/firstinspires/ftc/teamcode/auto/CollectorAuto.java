public CollectorAuto {

    public CollectorAuto(HardwareMap hwMap, Telemetry telemetry, Opencv stupidCam){
        super(hwMap, telemetry, stupidCam);
    }

    public Action takeInBlock(){
        return new Action(){
            @Override
            public boolean run(){

            }
        }
    }
}