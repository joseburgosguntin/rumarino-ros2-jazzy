use std::sync::{Mutex, atomic::{AtomicBool, AtomicI32, AtomicUsize}};
use r2r::interfaces::msg::*;
use r2r::geometry_msgs::msg::Pose;
pub type MissionResult = Result<(), bool>;

#[derive(Debug)]
pub struct MissionData {
    pub example_flag : AtomicBool,
    pub example_flag_request : AtomicBool,
    pub cached_map : Mutex<Map>,
    pub scouting : AtomicBool,
    pub map_objects_count : AtomicUsize, //It is possible this being atomic could cause problems
    pub pose : Mutex<Pose>,
}

impl MissionData {
    pub fn new() -> Self {
        MissionData {
            example_flag: AtomicBool::new(false),
            example_flag_request: AtomicBool::new(false),
            cached_map: Mutex::new(Map::default()),
            scouting: AtomicBool::new(true),
            map_objects_count: AtomicUsize::new(0),
            pose : Mutex::new(Pose::default())
        }
    }
}

pub trait Task : Send + Sync {
    fn run(&self, data: &MissionData) -> MissionResult;
    fn repair_run(&self, data: &MissionData) -> MissionResult;
    fn name(&self) -> &String;
}

pub struct RustTask {
    pub name: String,
    func: Option<fn(&MissionData) -> MissionResult>,
    repair_func: Option<fn(&MissionData) -> MissionResult>
}

fn run_with(func: Option<fn(&MissionData) -> MissionResult>, data: &MissionData) -> MissionResult {
    let Some(func) = func else {
        return Err(false);
    };
    func(data)
}

impl RustTask {
    pub fn new(name: String, func: Option< fn(&MissionData) -> MissionResult>,
    repair_func: Option<fn(&MissionData)-> MissionResult>) -> RustTask {
        RustTask {
            name,
            func,
            repair_func,
        }
    }
}

impl Task for RustTask {
    fn run(&self, data: &MissionData) -> MissionResult {
        run_with(self.func, data)
    }
    fn repair_run(&self, data: &MissionData) -> MissionResult {
        run_with(self.repair_func, data)
    }
    fn name(&self) -> &String {
        &self.name
    }
}

pub struct CommonMission {
    pub name: String,
    pub task_list: Vec<Box<dyn Task>>,
}

pub trait Mission : Send + Sync {
    fn run(&self, data: &MissionData) -> MissionResult;
    fn name(&self) -> &String;
}

impl CommonMission {
    
}
impl Mission for CommonMission {
    fn run(&self, data: &MissionData) -> MissionResult {
        if self.task_list.is_empty() {
            return Ok(())
        }

        let mut res = Ok(());
        for task in &self.task_list {
            let task_res = task.run(data);
            res = match task_res {
                Ok(_) => task_res,
                Err(skip) => {
                    if skip {
                        task_res
                    } else {
                        task.repair_run(data)
                    }
                },

            };
            if let Err(skip) = res {
                if ! skip {
                    break
                }
            }
        }
        res
    }

    fn name(&self) -> &String {
        &self.name
    }
}
