mod examples;
mod mission;
mod mission_scheduler;

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::Ordering;
use std::thread::sleep;
use std::time::{Duration, Instant};

use futures::future::BoxFuture;
use futures::prelude::*;
use interfaces::msg::*;
use r2r::{Node, QosProfile, interfaces, nav_msgs, std_msgs};

use crate::mission::MissionData;
use crate::mission_scheduler::{MissionBox, MissionThreadData};
use crate::{mission::CommonMission, mission_scheduler::MissionScheduler};

type OdometrySub = nav_msgs::msg::Odometry;
type Float64MultiArray = std_msgs::msg::Float64MultiArray;

fn handle_map(data: &MissionData, map: Map) {
    let mut cached_map = data
        .cached_map
        .try_lock()
        .expect("Failed to lock cached map");
    *cached_map = map;
    let map = &*cached_map;

    if !data.scouting.load(Ordering::Relaxed) {
        return;
    }
    let map_objects_count = data.map_objects_count.load(Ordering::Relaxed);
    let new_objects_count = map.objects.len() - map_objects_count;
    for i in map_objects_count..new_objects_count {
        let new_object = &map.objects[i];
        let pos = &new_object.bbox.center.position;
        r2r::log_info!(
            "mission_planner",
            "MapObject {} {{ {} {} {} }}",
            &new_object.cls,
            &pos.x,
            &pos.y,
            &pos.z
        );
        //Dunno how to implement object detection yet
    }
    data.map_objects_count
        .store(map_objects_count + 1, Ordering::Relaxed);
}

fn handle_odometry(data: &MissionData, odometry: OdometrySub) {
    let p = odometry.pose.pose.position;
    let o = odometry.pose.pose.orientation;

    let mut cur_pos = data.pose.try_lock().expect("Failed to lock pose");
    cur_pos.position = p;
    cur_pos.orientation = o;

    r2r::log_info!(
        "mission_planner",
        "Odometry {:#?} {:#?}",
        cur_pos.position,
        cur_pos.orientation
    );

    //Skipped yaw stuff
}

fn add_ros_topics(scheduler: &MissionScheduler) -> Node {
    let ctx = r2r::Context::create().expect("Failed to create r2r context!");
    let mut node =
        r2r::Node::create(ctx, "mission_planner", "namespace").expect("Failed to get Node!");
    //What QoS should we use?
    let mut map_sub = node
        .subscribe::<Map>("/hydrus/map", QosProfile::default())
        .expect("Failed to subscribe to map");
    let mut odometry_sub = node
        .subscribe::<OdometrySub>("/hydrus/odometry", QosProfile::default())
        .expect("Failed to subscribe to odometry");
    let thrusters_pub = node
        .create_publisher::<Float64MultiArray>("/hydrus/trusters", QosProfile::default())
        .expect("Failed to setup thruster publisher");

    let map_sub_func = |thread_data: Arc<MissionThreadData>| {
        let thread_data = thread_data.clone();
        let pin: BoxFuture<'static, ()> = Box::pin(async move {
            let data = &thread_data.mission_data;
            while !thread_data.stop.load(Ordering::Relaxed) {
                match map_sub.next().await {
                    Some(map) => {
                        handle_map(data, map);
                    }
                    None => break,
                }
            }
        });
        pin
    };

    let odometry_sub_func = |thread_data: Arc<MissionThreadData>| {
        let thread_data = thread_data.clone();
        let pin: BoxFuture<'static, ()> = Box::pin(async move {
            let data = &thread_data.mission_data;
            while !thread_data.stop.load(Ordering::Relaxed) {
                match odometry_sub.next().await {
                    Some(odometry) => {
                        handle_odometry(data, odometry);
                    }
                    None => break,
                }
            }
        });
        pin
    };

    scheduler.add_async_thread(map_sub_func);
    scheduler.add_async_thread(odometry_sub_func);

    node
}

fn main() {
    let mut scheduler = MissionScheduler::new();
    let mut node = add_ros_topics(&scheduler);

    scheduler.start();
    let start = Instant::now();
    //This is to give time to start oneshot_map
    while start.elapsed() < Duration::from_secs(15) {
        node.spin_once(Duration::from_millis(100));
    }
    scheduler.stop();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::examples::*;

    fn add_example_ros_topics(scheduler: &MissionScheduler) -> Node {
        let ctx = r2r::Context::create().expect("Failed to create r2r context!");
        let mut node =
            r2r::Node::create(ctx, "mission_planner", "namespace").expect("Failed to get Node!");
        /*let mut example_sub = node
        .subscribe::<sensor_msgs::msg::Image>("/camera/image", QosProfile::default())
        .expect("Failed to create example subscriber!");*/
        let example_pub = node
            .create_publisher::<std_msgs::msg::String>("/example", QosProfile::default())
            .expect("Failed to create example publisher!");

        /*let example_subscriber_func =
        |thread_data : Arc<MissionThreadData>| {
            let scheduler_data = thread_data.clone();
            let pin: BoxFuture<'static, ()> = Box::pin(async move {
                while ! scheduler_data.stop.load(Ordering::Relaxed) {
                    match example_sub.next().await {
                        Some(msg) => {
                            let mission = ros_mission::new(msg);
                            scheduler_data.push_back(mission);
                        }
                        None => break,
                    }
                }
            });
            pin
        };*/

        let example_publisher_func = |thread_data: Arc<MissionThreadData>| {
            let scheduler_data = thread_data.clone();
            let pin: BoxFuture<'static, ()> = Box::pin(async move {
                let mut counter = 0;
                let mut stop = false;
                while !stop {
                    let msg = std_msgs::msg::String {
                        data: format!("{}", counter),
                    };
                    example_pub
                        .publish(&msg)
                        .expect("Failed to publish example!");
                    counter += 1;
                    stop = scheduler_data.stop.load(Ordering::Relaxed);
                    //Should we use a ros timer instead?
                    sleep(Duration::from_secs(1));
                    //This should probably go on another thread
                }
            });
            pin
        };

        scheduler.add_async_thread(example_publisher_func);
        node
    }

    #[test]
    fn main_test() -> Result<(), String> {
        let foo = mission_example::new();
        let bar = concurrent_mission_example::new();

        let mission_list: [MissionBox; 1] = [Box::new(foo)];
        let mission_list = VecDeque::from(mission_list);
        let conc_mission_list: [MissionBox; 1] = [Box::new(bar)];
        let conc_mission_list = VecDeque::from(conc_mission_list);

        let mut scheduler = MissionScheduler::new();
        scheduler.append(mission_list);
        scheduler.conc_append(conc_mission_list);
        let _data = scheduler.get_data();

        scheduler.start();
        let mut node = add_example_ros_topics(&scheduler);
        while !scheduler.is_waiting() {
            node.spin_once(Duration::from_millis(100));
        }
        scheduler.stop();
        Ok(())
    }
}
