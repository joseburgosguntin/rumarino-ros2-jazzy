use futures::executor::ThreadPool;
use futures::future::BoxFuture;
use crate::mission::{Mission, MissionData};
use std::collections::VecDeque;
use std::sync::{Arc, Mutex};
use std::sync::atomic::{AtomicBool, Ordering};
use std::thread::{self, JoinHandle, sleep};

pub type MissionBox = Box<dyn Mission>;
pub type MissionVec = VecDeque<MissionBox>;
pub struct MissionThreadData {
    mission_list: Mutex<MissionVec>,
    conc_mission_list: Mutex<MissionVec>,
    pub mission_data: MissionData,
    run: AtomicBool,
    pub stop: AtomicBool,
    waiting: AtomicBool,
}

impl MissionThreadData {
    fn new() -> Self {
        Self {
            mission_list: Mutex::new(VecDeque::new()),
            conc_mission_list: Mutex::new(VecDeque::new()),
            mission_data: MissionData::new(),
            run: AtomicBool::new(false),
            stop: AtomicBool::new(false),
            waiting: AtomicBool::new(false),
        }
    }

    fn with_mission_list(&self, func: impl FnOnce(&mut MissionVec) -> Option<MissionBox>, is_concurrent: bool) -> Option<MissionBox> {
        let mut guard = if is_concurrent {
            self.conc_mission_list.try_lock().expect("Concurrent mission lock is poisoned!")
        } else {
            self.mission_list.try_lock().expect("Mission lock is poisoned!")
        };
        func(&mut guard)
    }

    pub fn pop_front(&self) -> Option<Box<dyn Mission>> {
        let func = move |mission_list: &mut MissionVec| {
            mission_list.pop_front()
        };
        self.with_mission_list(func, false)
    }

    pub fn push_back(&self, mission : impl Mission + 'static) -> Option<MissionBox> {
        let func = move |mission_list: &mut MissionVec| {
            mission_list.push_back(Box::new(mission));
            None
        };
        self.with_mission_list(func, false)
    }
}


pub struct MissionScheduler {
    normal_handle: Option<thread::JoinHandle<()>>,
    concurrent_handle: Option<thread::JoinHandle<()>>,
    pool: ThreadPool,
    scheduler_data : Arc<MissionThreadData>,
}

impl MissionScheduler {
    pub fn new() -> Self {
        let scheduler_data = Arc::new(MissionThreadData::new());
        Self {
            normal_handle: None,
            concurrent_handle: None,
            scheduler_data,
            pool: ThreadPool::new().expect("Failed to create ThreadPool"),
        }
    }

    #[allow(unused)]
    pub fn push_back(&self, mission: impl Mission + 'static) {
        let func = move |mission_list: &mut VecDeque<MissionBox>| {
            mission_list.push_back(Box::new(mission));
            None
        };
        self.scheduler_data.with_mission_list(func, false);
    }

    #[allow(unused)]
    pub fn conc_push_back(&self, mission: impl Mission + 'static) {
        let func = move |mission_list: &mut MissionVec| {
            mission_list.push_back(Box::new(mission));
            None
        };
        self.scheduler_data.with_mission_list(func, true);
    }

    #[allow(unused)]
    pub fn append(&self, mut mission_vec: MissionVec) {
        let func = move |mission_list: &mut MissionVec| {
            mission_list.append(&mut mission_vec);
            None
        };
        self.scheduler_data.with_mission_list(func, false);
    }

    #[allow(unused)]
    pub fn conc_append(&self, mut mission_vec: MissionVec) {
        let func = move |mission_list: &mut MissionVec| {
            mission_list.append(&mut mission_vec);
            None
        };
        self.scheduler_data.with_mission_list(func, true);
    }

    #[allow(unused)]
    pub fn get_data(&self) -> &MissionData {
        &self.scheduler_data.mission_data
    }

    #[allow(unused)]
    pub fn is_waiting(&self) -> bool {
        self.scheduler_data.waiting.load(Ordering::Relaxed)
    }

    // pub fn concurrent_append(&mut self, mission: &mut Vec<Box<dyn Mission<'static> + Send >>) {
    //     self.concurrent_mission_list.append(mission);
    // }
    pub fn add_async_thread<F>(&self, func: F)
    where F : FnOnce(Arc<MissionThreadData>) -> BoxFuture<'static, ()>
    {
        let future = func(self.scheduler_data.clone());
        self.pool.spawn_ok(future);

    }
   
    pub fn start(&mut self) {
        let scheduler_data = self.scheduler_data.clone();
        let normal_func = move || {
            let mut stop = false;
            let data = &scheduler_data.mission_data;

            while ! stop {
                stop = scheduler_data.stop.load(Ordering::Relaxed);

                let mission = scheduler_data.pop_front();
                let Some(mission) = mission else {
                    println!("Waiting for missions...");
                    scheduler_data.waiting.store(true, Ordering::Relaxed);
                    sleep(std::time::Duration::from_secs(3));
                    continue;
                };
                let res = mission.run(data);
                match res {
                    Ok(_) => (),
                    Err(skip) => {
                        if skip {
                            println!("{} mission skipped!", mission.name());
                        }
                        else {
                            println!("{} mission failed!", mission.name());
                            scheduler_data.stop.store(true, Ordering::Relaxed);
                            stop = true;
                        }
                    }
                };


                sleep(std::time::Duration::from_millis(1));
            }
        };

        let scheduler_data = self.scheduler_data.clone();
        let concurrent_func = move || {
            let mut stop = false;
            let data = &scheduler_data.mission_data;
            while ! stop {
                stop = scheduler_data.stop.load(Ordering::Relaxed);
                let getter = scheduler_data.conc_mission_list
                    .try_lock()
                    .expect("Concurrent mission lock is poisoned!");
                for mission in &*getter {
                    let res = mission.run(data);
                    match res {
                        Ok(_) => (),
                        Err(skip) => {
                            if skip {
                                println!("{} mission skipped!", mission.name());
                            }
                            else {
                                println!("{} mission failed!", mission.name());
                                scheduler_data.stop.store(true, Ordering::Relaxed);
                                stop = true;
                            }
                        }
                    };
                };

                sleep(std::time::Duration::from_millis(100));
            }
        };

        
        let normal_handle = thread::spawn(normal_func);
        let conc_handle = thread::spawn(concurrent_func);
        self.normal_handle = Some(normal_handle);
        self.concurrent_handle = Some(conc_handle);
    }

    pub fn stop(&mut self) {
        self.scheduler_data.stop.store(true, Ordering::Relaxed);
        let Some(normal_handle) = self.normal_handle.take() else {
            return;
        };
        let Some(conc_handle) = self.concurrent_handle.take() else {
            return;
        };
        normal_handle.join().expect("Failed to join mission handle!");
        conc_handle.join().expect("Failed to join concurrent mission handle!");
    }
}
