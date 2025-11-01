#![allow(unused)]

#![deny(unused_must_use)]
//! This process should just realize a `Mission` passed as a ROS arg (somehow) and terminate.
//! A `Mission` is a scenario where the submarine must react to diferent `ObjectCls` with their
//! respective sequences of actions.

mod missions;

use std::sync::Arc;
use std::sync::atomic::{AtomicUsize, Ordering};
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;
use futures::StreamExt;
use nalgebra::{ArrayStorage, Quaternion, SimdPartialOrd, UnitQuaternion, Vector, Vector3, Vector6, U8};
use r2r::{Node, ParameterValue, QosProfile};
use tokio::sync::Notify;

#[derive(Clone, Copy, Debug)]
struct Pose {
    pos: Vector3<f64>,
    rot: Quaternion<f64>,
}

impl From<&r2r::geometry_msgs::msg::Pose> for Pose {
    fn from(value: &r2r::geometry_msgs::msg::Pose) -> Self {
        let p = &value.position;
        let o = &value.orientation;

        Self {
            pos: Vector3::new(p.x, p.y, p.z),
            rot: Quaternion::new(o.w, o.x, o.y, o.z),
        }
    }
}

struct MissionExecutor {
    pub map: ArcSwap<MapMsg>,
    pub map_objects_reacted: AtomicUsize,
    pub new_objects: Notify,
    pub pose: ArcSwap<Pose>,
    pub goal: ArcSwap<Vector6<f64>>,
    pub mission: Box<dyn Mission>,
}

const CLOSE_ENOUGH: f64 = 0.25;

impl MissionExecutor {
    pub fn new(mission: Box<dyn Mission>) -> Self {
        let origin = Pose { pos: Vector3::zeros(), rot: Quaternion::new(1.0, 0.0, 0.0, 0.0) };
        let goal = Vector6::zeros();
        Self {
            map: ArcSwap::new(Arc::new(MapMsg::default())),
            map_objects_reacted: AtomicUsize::new(0),
            new_objects: Notify::new(),
            pose: ArcSwap::new(Arc::new(origin)),
            goal: ArcSwap::new(Arc::new(goal)),
            mission,
        }
    }

    // maybe this should be done relative to an object
    /// blocks until `dest` is reached within `CLOSE_ENOUGH` distance
    pub fn move_to(&self, dest: Vector3<f64>) {
        *self.goal.load().xyz() = *dest;
        loop {
            let dist = self.pose.load().pos.metric_distance(&dest);
            if dist < CLOSE_ENOUGH {
                break
            } else {
                std::thread::sleep(Duration::from_millis(100));
            }
        }
    }
}

#[repr(i32)]
pub enum ObjectCls {
    Cube = 0,
    Rectangle = 1,
    Gate = 2,
    Shark = 3,
}

pub struct BoundingBox3D {
    center: Pose,
    size: Vector3<f64>,
}

pub struct MapObject {
    cls: ObjectCls,
    bbox: BoundingBox3D,
}

impl From<&r2r::interfaces::msg::MapObject> for MapObject {
    fn from(value: &r2r::interfaces::msg::MapObject) -> Self {
        let cls = unsafe { std::mem::transmute::<i32, ObjectCls>(value.cls) };
        Self {
            cls,
            bbox: BoundingBox3D {
                center: Pose::from(&value.bbox.center),
                size: Vector3::new(
                    value.bbox.size.x,
                    value.bbox.size.y,
                    value.bbox.size.z,
                ),
            },
        }
    }
}

// #[async_trait::async_trait]
trait Mission: Send + Sync {
    fn react_to_object(&self, td: &MissionExecutor, idx: usize);
}

type MapMsg = r2r::interfaces::msg::Map;
type OdometryMsg = r2r::nav_msgs::msg::Odometry;
type Float64MultiArray = r2r::std_msgs::msg::Float64MultiArray;

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
    let mut odometry_sub = node
        .subscribe::<OdometryMsg>("/hydrus/odometry", QosProfile::default())
        .expect("Failed to subscribe to odometry");
    let thrusters_pub = node
        .create_publisher::<Float64MultiArray>("/hydrus/thrusters", QosProfile::default())
        .expect("Failed to setup thruster publisher");

    let consume_map_sub = |td: Arc<MissionExecutor>| async move {
        while let Some(msg) = map_sub.next().await {
            td.map.store(Arc::new(msg));
            td.new_objects.notify_one();
        }
    };

    let consume_odometry_sub = |td: Arc<MissionExecutor>| async move {
        while let Some(msg) = odometry_sub.next().await {
            td.pose.store(Arc::new(Pose::from(&msg.pose.pose)));
        }
    };

    let kp = Vector6::<f64>::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0) / 0.5e0;
    let ki = Vector6::<f64>::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0) / 1e0;
    let kd = Vector6::<f64>::new(1.0, 1.0, 1.0, 1.0, 1.0, 1.0) / 1e0;

    let go_to_goal = |td: Arc<MissionExecutor>| async move {
        let mut sum_err = Vector6::zeros();
        let mut prev_pose_err = Vector6::zeros();
        let mut prev_now = Instant::now();
        loop {
            let now = Instant::now();
            let dt = now.duration_since(prev_now).as_secs_f64();

            let pose = **td.pose.load();
            r2r::log_info!("pose", "{pose:?}");
            let goal = **td.goal.load();
            r2r::log_info!("goal", "{goal:?}");
            let p = pose.pos;
            let unit = UnitQuaternion::from_quaternion(pose.rot);
            let (roll, pitch, yaw) = unit.euler_angles();
            let current_pose = Vector6::new(p.x, p.y, p.z, roll, pitch, yaw);

            let pose_err = goal - current_pose;
            let vel_err = (pose_err - prev_pose_err) / dt;
            sum_err += pose_err * dt;

            let wrench = kp.component_mul(&pose_err) + ki.component_mul(&sum_err) + kd.component_mul(&vel_err);
            r2r::log_info!("wrench", "{wrench:?}");

            // let other_values = other_tam_svd.solve(&wrench, 1e-10).expect("Rank-deficient");
            let mut thurstor_values = Vector::<f64, U8, ArrayStorage<f64, 8, 1>>::zeros();
            // thurstor_values[0] = other_values[0];
            // thurstor_values[1] = other_values[1];
            // thurstor_values[2] = other_values[2];
            // thurstor_values[3] = other_values[3];
            thurstor_values[4] = -wrench.z;
            thurstor_values[5] = -wrench.z;
            thurstor_values[6] = -wrench.z;
            thurstor_values[7] = -wrench.z;
            let minn = Vector::<f64, U8, ArrayStorage<f64, 8, 1>>::repeat(-5.0);
            let maxx = Vector::<f64, U8, ArrayStorage<f64, 8, 1>>::repeat(5.0);
            r2r::log_info!("thurstor_values", "{thurstor_values:?}");
            thurstor_values = thurstor_values.simd_clamp(minn, maxx) / 5.0;

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
        loop {
            // TODO: do scouting, this code has to have .await's so that abort works
            tokio::time::sleep(Duration::from_millis(100)).await;
        }
    };
    let mut scout_handle = tokio::spawn(scout(Arc::clone(&td)));

    let consume_new_objects = |td: Arc<MissionExecutor>| async move {
        loop {
            td.new_objects.notified().await;
            scout_handle.abort();
            let mut reacted = td.map_objects_reacted.load(Ordering::Relaxed);
            loop {
                let objects_len = td.map.load().objects.len();
                if reacted == objects_len { 
                    break;
                }
                while reacted < objects_len {
                    td.mission.react_to_object(&td, reacted);
                    reacted += 1;
                }
            }
            td.map_objects_reacted.store(reacted, Ordering::Relaxed);
            scout_handle = tokio::spawn(scout(Arc::clone(&td)));
        }
    };

    tokio::spawn(consume_map_sub(Arc::clone(&td)));
    tokio::spawn(consume_odometry_sub(Arc::clone(&td)));
    // tokio::spawn(consume_new_objects(Arc::clone(&td)));
    tokio::spawn(go_to_goal(Arc::clone(&td)));

    // TODO make depth work while tam is also existing
    td.goal.store(Arc::new(Vector6::<f64>::new(0.0, 0.0, 1.0, 0.0, 0.0, 0.0))); // TODO: get me for real

    loop {
        node.spin_once(Duration::from_millis(100));
    }
}
