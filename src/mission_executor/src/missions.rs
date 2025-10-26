use crate::{Mission, MissionExecutor, ObjectCls};

// this could hold some state if necessary
// like the some sort of queue if a sequence of reactions is necessary
pub(crate) struct PrecualifyMission {}

impl PrecualifyMission {
    pub(crate) fn new() -> Self {
        PrecualifyMission {}
    }
    // TODO: these function are gonna need fuctions like: [move_to, look_at]
    //  these functions are expected to take controll and block the
    //  thread until they're done
    fn go_around(td: &MissionExecutor) {
        todo!()
    }
    fn go_through(td: &MissionExecutor) {
        todo!()
    }
}

impl Mission for PrecualifyMission {
    fn object_reactions(&self, cls: ObjectCls) -> Option<fn(&MissionExecutor) -> ()> {
        match cls {
            ObjectCls::Rectangle | ObjectCls::Cube => Some(Self::go_around),
            ObjectCls::Gate => Some(Self::go_through),
            ObjectCls::Shark => None,
        }
    }
}
