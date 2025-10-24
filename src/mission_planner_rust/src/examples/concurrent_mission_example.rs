use std::sync::atomic::Ordering;

use crate::mission::{CommonMission, Mission, MissionData, MissionResult, RustTask, Task};

#[allow(unused)]
fn conc_example(data: &MissionData) -> MissionResult {
    if data.example_flag_request.load(Ordering::Relaxed) {
        data.example_flag.store(true, Ordering::Relaxed);
        println!("Wrote flag!");
    }
    MissionResult::Ok
}

#[allow(unused)]
pub fn new() -> impl Mission {
    let name = "concurrent-mission-example".to_string();
    let task = RustTask::new("conc-example-task".to_string(), Some(conc_example), None);
    let task_list: Vec<Box<dyn Task>> =  vec![
        Box::new(task)
    ];
    CommonMission { name, task_list }
}

