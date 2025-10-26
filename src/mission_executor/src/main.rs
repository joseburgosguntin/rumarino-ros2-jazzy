#![allow(unused)]
//! This process should just realize a `Mission` passed as a ROS arg (somehow) and terminate.
//! A `Mission` is a scenario where the submarine must react to diferent `ObjectCls` with their
//! respective sequences of actions.

mod missions;

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::Duration;

use futures::StreamExt;
use nalgebra::{Vector3, Quaternion};
use r2r::{Node, ParameterValue, QosProfile};
use tokio::sync::Mutex;
use tokio::sync::mpsc::{Sender, channel};

struct Pose {
    pos: Vector3<f64>,
    rot: Quaternion<f64>,
}

struct MissionExecutor {
    pub cached_map: Mutex<MapMsg>,
    pub map_objects_count: AtomicUsize,
    pub pose: Mutex<Pose>,
    pub mission: Box<dyn Mission>,
}

impl MissionExecutor {
    pub fn new(mission: Box<dyn Mission>) -> Self {
        let origin = Pose { pos: Vector3::zeros(), rot: Quaternion::new(1.0, 0.0, 0.0, 0.0) };
        Self {
            cached_map: Mutex::new(MapMsg::default()),
            map_objects_count: AtomicUsize::new(0),
            pose: Mutex::new(origin),
            mission,
        }
    }
}

pub enum ObjectCls {
    Cube = 0,
    Rectangle = 1,
    Gate = 2,
    Shark = 3,
}

trait Mission: Send + Sync {
    fn object_reactions(&self, cls: ObjectCls) -> Option<fn(&MissionExecutor) -> ()>;
}

type MapMsg = r2r::interfaces::msg::Map;
type OdometryMsg = r2r::nav_msgs::msg::Odometry;
type Float64MultiArray = r2r::std_msgs::msg::Float64MultiArray;

async fn handle_map(
    td: &MissionExecutor,
    map: MapMsg,
    reactions_tx: &mut Sender<fn(&MissionExecutor) -> ()>,
) {
    let mut cached_map = td.cached_map.lock().await;
    *cached_map = map;
    let map_objects_count = td.map_objects_count.load(Ordering::Relaxed);
    let new_objects_count = cached_map.objects.len() - map_objects_count;

    for new_object in &cached_map.objects[map_objects_count..new_objects_count] {
        let pos = &new_object.bbox.center.position;
        r2r::log_info!(
            "mission_executor",
            "MapObject {} {{ {} {} {} }}",
            &new_object.cls,
            &pos.x,
            &pos.y,
            &pos.z
        );
        let cls = unsafe { std::mem::transmute::<u8, ObjectCls>(new_object.cls as u8) };
        if let Some(reaction) = td.mission.object_reactions(cls) {
            reactions_tx.send(reaction).await.unwrap();
        }
    }
    td.map_objects_count
        .store(map_objects_count + 1, Ordering::Relaxed);
}

async fn handle_odometry(data: &MissionExecutor, odometry: OdometryMsg) {
    let p = odometry.pose.pose.position;
    let o = odometry.pose.pose.orientation;

    let mut pose = data.pose.lock().await;
    pose.pos.x = p.x;
    pose.pos.y = p.y;
    pose.pos.z = p.z;
    pose.rot.w = o.w;
    pose.rot.i = o.x;
    pose.rot.j = o.y;
    pose.rot.k = o.z;

    r2r::log_info!(
        "mission_planner",
        "Odometry {:#?} {:#?}",
        pose.pos,
        pose.rot
    );
}

#[tokio::main]
async fn main() {
    let ctx = r2r::Context::create().expect("Failed to create r2r context!");
    let mut node = Node::create(ctx, "mission_executor", "namespace").expect("Failed to get Node!");
    let params = node.params.lock().unwrap();

    let mission: Box<dyn Mission> = match params.get("mission_name") {
        Some(r2r::Parameter { value, .. }) => match value {
            ParameterValue::String(str) => match str.as_str() {
                "prequalify" => Box::new(missions::PrecualifyMission::new()),
                _ => panic!("mission_name param must be a mission that exists"),
            },
            _ => panic!("mission_name param must be passed a string"),
        },
        None => panic!("mission_name param must be passed to mission_executor"),
    };
    drop(params);

    let td = Arc::new(MissionExecutor::new(mission));

    let map_qos = QosProfile::default().keep_last(1).transient_local();
    let mut map_sub = node
        .subscribe::<MapMsg>("/hydrus/map", map_qos)
        .expect("Failed to subscribe to map");
    let mut odometry_sub = node
        .subscribe::<OdometryMsg>("/hydrus/odometry", QosProfile::default())
        .expect("Failed to subscribe to odometry");
    let thrusters_pub = node
        .create_publisher::<Float64MultiArray>("/hydrus/trusters", QosProfile::default())
        .expect("Failed to setup thruster publisher");

    // TODO: shouldn't this be some sort of priority queue since we wanted to schedule?
    let (mut reactions_tx, mut reactions_rx) = channel::<fn(&MissionExecutor) -> ()>(16);

    let consume_map_sub = |td: Arc<MissionExecutor>| async move {
        while let Some(msg) = map_sub.next().await {
            handle_map(&td, msg, &mut reactions_tx).await;
        }
    };

    let consume_odometry_sub = |td: Arc<MissionExecutor>| async move {
        while let Some(msg) = odometry_sub.next().await {
            handle_odometry(&td, msg);
        }
    };

    let scout = |td: Arc<MissionExecutor>| async move {
        // TODO: do scouting, this code has to have .await's so that abort works
    };
    let mut scout_handle = tokio::spawn(scout(Arc::clone(&td)));

    let consume_reactions = |td: Arc<MissionExecutor>| async move {
        while let Some(handle_reaction) = reactions_rx.recv().await {
            scout_handle.abort();
            handle_reaction(&td);
            scout_handle = tokio::spawn(scout(Arc::clone(&td)));
        }
    };

    tokio::spawn(consume_map_sub(Arc::clone(&td)));
    tokio::spawn(consume_odometry_sub(Arc::clone(&td)));
    tokio::spawn(consume_reactions(Arc::clone(&td)));

    loop {
        node.spin_once(Duration::from_millis(100));
    }
}
