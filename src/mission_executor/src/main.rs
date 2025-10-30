#![allow(unused)]

#![deny(unused_must_use)]
//! This process should just realize a `Mission` passed as a ROS arg (somehow) and terminate.
//! A `Mission` is a scenario where the submarine must react to diferent `ObjectCls` with their
//! respective sequences of actions.

mod missions;

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use futures::StreamExt;
use nalgebra::{ArrayStorage, Matrix3x4, Matrix6x4, Quaternion, SimdPartialOrd, UnitQuaternion, Vector, Vector3, Vector6, U8};
use r2r::qos::LivelinessPolicy;
use r2r::{Node, ParameterValue, QosProfile};
use tokio::sync::Mutex;
use tokio::sync::mpsc::{Sender, channel};

#[derive(Debug)]
struct Pose {
    pos: Vector3<f64>,
    rot: Quaternion<f64>,
}

struct MissionExecutor {
    pub cached_map: Mutex<MapMsg>,
    pub map_objects_count: AtomicUsize,
    pub pose: Mutex<Pose>,
    pub goal: Mutex<Vector6<f64>>,
    pub mission: Box<dyn Mission>,
}

impl MissionExecutor {
    pub fn new(mission: Box<dyn Mission>) -> Self {
        let origin = Pose { pos: Vector3::zeros(), rot: Quaternion::new(1.0, 0.0, 0.0, 0.0) };
        let goal = Vector6::zeros();
        Self {
            cached_map: Mutex::new(MapMsg::default()),
            map_objects_count: AtomicUsize::new(0),
            pose: Mutex::new(origin),
            goal: Mutex::new(goal),
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

    r2r::log_info!(
        "mission_planner",
        "Odometry {p:?} {o:?}",
    );

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
    // let other_directions = Matrix3x4::new(
    //   0.382, 0.382, -0.382, -0.382,
    //   -0.923, 0.923, -0.923, 0.000,
    //   0.000, 0.000, 0.000, 0.000,
    // );
    // let other_positions = Matrix3x4::new(
    //   0.198, 0.198, -0.198, -0.198,
    //   0.407, -0.408, 0.407, -0.408,
    //   0.000, 0.000, 0.000, 0.000,
    // );
    //
    // let mut other_tam = Matrix6x4::<f64>::default();
    // assert!(other_tam.ncols() == 4);
    // assert!(other_tam.ncols() == other_positions.ncols());
    // assert!(other_tam.ncols() == other_directions.ncols());
    // for i in 0..other_tam.ncols() {
    //     let position = other_positions.column(i);
    //     let direction = other_directions.column(i);
    //     let orthogonal = position.cross(&direction);
    //     let other_tam_col = Vector6::new(
    //         position[0],
    //         position[1],
    //         position[2],
    //         orthogonal[0],
    //         orthogonal[1],
    //         orthogonal[2],
    //     );
    //     other_tam.set_column(i, &other_tam_col);
    // }
    // r2r::log_info!("mission_executor", "tam = {other_tam:?}");
    // let wrench = Vector6::new(0.01, 0.01, 1.0, 0.01, 0.01, 0.01);
    // let other_tam_svd = other_tam.svd(true, true);

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
// QoS profile:
//   Reliability: RELIABLE
//   History (Depth): UNKNOWN
//   Durability: VOLATILE
//   Lifespan: Infinite
//   Deadline: Infinite
//   Liveliness: AUTOMATIC
//   Liveliness lease duration: Infinite
    let mut odometry_sub = node
        .subscribe::<OdometryMsg>("/hydrus/odometry", QosProfile::default())
        .expect("Failed to subscribe to odometry");
    let thrusters_pub = node
        .create_publisher::<Float64MultiArray>("/hydrus/thrusters", QosProfile::default())
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
            handle_odometry(&td, msg).await;
        }
    };

    let kp = Vector6::<f64>::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0) / 1e0;
    let ki = Vector6::<f64>::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0) / 1e0;
    let kd = Vector6::<f64>::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0) / 1e0;

    let go_to_goal = |td: Arc<MissionExecutor>| async move {
        let mut sum_err = Vector6::zeros();
        let mut prev_pose_err = Vector6::zeros();
        let mut prev_now = Instant::now();
        loop {
            let now = Instant::now();
            let dt = now.duration_since(prev_now).as_secs_f64();

            // TODO: fix odometry_sub so that we actually get updated position
            let pose = td.pose.lock().await;
            r2r::log_info!("mission_executor", "pose = {pose:?}");
            let goal = td.goal.lock().await;
            let p = pose.pos;
            let unit = UnitQuaternion::from_quaternion(pose.rot);
            let (roll, pitch, yaw) = unit.euler_angles();
            let current_pose = Vector6::new(p.x, p.y, p.z, roll, pitch, yaw);

            let pose_err = *goal - current_pose;
            let vel_err = (pose_err - prev_pose_err) / dt;
            sum_err += pose_err * dt;

            let wrench = kp.component_mul(&pose_err) + ki.component_mul(&sum_err) + kd.component_mul(&vel_err);
            r2r::log_info!("mission_executor", "wrench = {wrench:?}");

            // let other_values = other_tam_svd.solve(&wrench, 1e-10).expect("Rank-deficient");
            let mut thurstor_values = Vector::<f64, U8, ArrayStorage<f64, 8, 1>>::zeros();
            // thurstor_values[0] = other_values[0];
            // thurstor_values[1] = other_values[1];
            // thurstor_values[2] = other_values[2];
            // thurstor_values[3] = other_values[3];
            thurstor_values[4] = wrench.z;
            thurstor_values[5] = wrench.z;
            thurstor_values[6] = wrench.z;
            thurstor_values[7] = wrench.z;
            let minn = Vector::<f64, U8, ArrayStorage<f64, 8, 1>>::repeat(-5.0);
            let maxx = Vector::<f64, U8, ArrayStorage<f64, 8, 1>>::repeat(5.0);
            thurstor_values = thurstor_values.simd_clamp(minn, maxx) / 5.0;
            r2r::log_info!("mission_executor", "values_norm = {thurstor_values:?}");

            let mut thrusters_msg = Float64MultiArray::default();
            thrusters_msg.data.extend(thurstor_values.iter());
            thrusters_pub
                .publish(&thrusters_msg)
                .expect("Failed to publish");

            prev_pose_err = pose_err;
            prev_now = now;

            tokio::time::sleep(Duration::from_millis(100)).await;
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
    tokio::spawn(go_to_goal(Arc::clone(&td)));

    *td.goal.lock().await = Vector6::<f64>::new(0.0, 0.0, -0.5, 0.0, 1.0, 0.0); // TODO: get me for real

    loop {
        node.spin_once(Duration::from_millis(100));
    }
}
