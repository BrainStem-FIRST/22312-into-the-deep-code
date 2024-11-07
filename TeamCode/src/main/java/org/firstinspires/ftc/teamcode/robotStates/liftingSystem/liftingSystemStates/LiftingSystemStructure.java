public class LiftingSystemStructure {
    private Lift.StateType liftStateType;
    private Arm.StateType armStateType;
    private Grabber.StateType grabberStateType;

    public LiftingSystemStructure(Lift.StateType liftStateType, Arm.StateType armStateType, Grabber.StateType grabberStateType) {
        this.liftStateType = liftStateType;
        this.armStateType = armStateType;
        this.grabberStateType = grabberStateType;
    }
    
    // getters
    public Lift.StateType getLiftStateType() {
        return liftStateType;
    }
    public Arm.StateType getArmStateType() {
        return armStateType;
    }
    public Grabber.StateType getGrabberStateType() {
        return grabberStateType;
    }

}
