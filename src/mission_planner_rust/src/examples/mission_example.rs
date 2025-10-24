use std::thread::sleep;
use crate::mission::{CommonMission, Mission, MissionData, MissionResult, RustTask, Task};
use std::sync::atomic::Ordering;

#[allow(unused)]
fn example(_data: &MissionData) -> MissionResult {
    println!("Hello from Rust!");
    MissionResult::Err("Example Error".to_owned())
}

#[allow(unused)]
fn repair_example(data: &MissionData) -> MissionResult {
    println!("Requested flag");
    data.example_flag_request.store(true, Ordering::Relaxed);

    while ! data.example_flag.load(Ordering::Relaxed) {
        sleep(std::time::Duration::from_millis(100));
    }
    println!("Got flag!");

    MissionResult::Ok
}

#[allow(unused)]
pub fn new() -> impl Mission {
    let name = "example-mission".to_string();
    let task = RustTask::new("example-task".to_string(), Some(example), Some(repair_example));
    let task_list: Vec<Box<dyn Task>> = vec![
        Box::new(task)
    ];

    CommonMission { name, task_list }
}
