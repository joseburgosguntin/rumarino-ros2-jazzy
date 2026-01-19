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
use nalgebra::{ArrayStorage, Matrix4x3, Quaternion, SimdPartialOrd, UnitQuaternion, Vector, Vector3, Vector6, U8};
type Vector8<T> = Vector<T, U8, ArrayStorage<T, 8, 1>>;
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

const CLOSE_ENOUGH: f64 = 1.0;

impl MissionExecutor {
    pub fn new(mission: Box<dyn Mission>) -> Self {
        // hardcoded so it doesn't freak out while it waits for first odometry
        let origin = Pose { pos: Vector3::new(-3.0, 1.0, 1.0), rot: Quaternion::identity() };
        let (roll, pitch, yaw) = UnitQuaternion::from_quaternion(origin.rot).euler_angles();
        let mut goal = Vector6::zeros();
        goal.fixed_rows_mut::<3>(0).copy_from(&origin.pos);
        goal.fixed_rows_mut::<3>(3).copy_from(&Vector3::new(roll, pitch, yaw));
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
        let mut old_goal = **self.goal.load();
        old_goal.x = dest.x;
        old_goal.y = dest.y;
        old_goal.z = dest.z;
        self.goal.store(Arc::new(old_goal));
        loop {
            let pose = self.pose.load();
            let goal = self.goal.load();
            let dist = pose.pos.metric_distance(&dest);
            r2r::log_info!("dist", "{dist}, pose: {pose:?}, goal: {goal:?}");
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

trait Mission: Send + Sync {
    fn react_to_object(&self, td: &MissionExecutor, idx: usize);
}

type MapMsg = r2r::interfaces::msg::Map;
type MapObjectMsg = r2r::interfaces::msg::MapObject;
type PointMsg = r2r::geometry_msgs::msg::Point;
type QuaternionMsg = r2r::geometry_msgs::msg::Quaternion;
type Vector3Msg = r2r::geometry_msgs::msg::Vector3;
type OdometryMsg = r2r::nav_msgs::msg::Odometry;
type Float64MultiArray = r2r::std_msgs::msg::Float64MultiArray;

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

    const KP: Vector6<f64> = Vector6::new(3.50, 3.50, 2.20, 2.40, 2.40, 2.20);
    const KI: Vector6<f64> = Vector6::new(0.10, 0.10, 0.05, 0.05, 0.05, 0.01);
    const KD: Vector6<f64> = Vector6::new(0.70, 0.90, 0.40, 0.20, 0.20, 0.80);

    const TAM_X_Y_YAW: Matrix4x3<f64> = Matrix4x3::new(
        -1.0,  1.0,  1.0,
        -1.0, -1.0, -1.0,
         1.0,  1.0, -1.0,
         1.0, -1.0,  1.0,
    );

    const TAM_Z_ROLL_PITCH: Matrix4x3<f64> = Matrix4x3::new(
        -1.0,  1.0,  1.0,
        -1.0, -1.0,  1.0,
        -1.0,  1.0, -1.0,
        -1.0, -1.0, -1.0,
    );

    const THRUSTOR_SATURATE: f64 = 5.0;

    let go_to_goal = |td: Arc<MissionExecutor>| async move {
        let mut sum_err = Vector6::zeros();
        let mut prev_pose_err = Vector6::zeros();
        let mut prev_now = Instant::now();
        loop {
            let now = Instant::now();
            let dt = now.duration_since(prev_now).as_secs_f64();

            let pose = **td.pose.load();
            let goal = **td.goal.load();
            let p = pose.pos;
            let unit = UnitQuaternion::from_quaternion(pose.rot);
            let (roll, pitch, yaw) = unit.euler_angles();
            let current_pose = Vector6::new(p.x, p.y, p.z, roll, pitch, yaw);

            // r2r::log_info!("goal", "{goal:?}");
            // r2r::log_info!("pose", "{current_pose:?}");

            let pose_err = goal - current_pose;
            let vel_err = (pose_err - prev_pose_err) / dt;
            sum_err += pose_err * dt;

            let wrench = KP.component_mul(&pose_err) + KI.component_mul(&sum_err) + KD.component_mul(&vel_err);

            let rotated = unit.conjugate() * wrench.xyz();
            let xy_error = wrench.xy().norm();
            // only apply yaw when close
            let yaw_scale = if xy_error < 0.5 { 1.0 } else { 0.0 };
            let input_x_y_yaw = Vector3::new(rotated.x, rotated.y, wrench[5] * yaw_scale);
            let input_z_roll_pitch = Vector3::new(wrench.z, -wrench[3], wrench[4]);

            // r2r::log_info!("wrench", "{wrench:?}");
            // r2r::log_info!("rotated", "{rotated:?}");

            let mut thurstor_values = Vector8::zeros();
            thurstor_values.fixed_rows_mut::<4>(0).copy_from(&(TAM_X_Y_YAW * input_x_y_yaw));
            thurstor_values.fixed_rows_mut::<4>(4).copy_from(&(TAM_Z_ROLL_PITCH * input_z_roll_pitch));

            // r2r::log_info!("thurstor_values", "{thurstor_values:?}");

            let minn = Vector8::repeat(-THRUSTOR_SATURATE);
            let maxx = Vector8::repeat(THRUSTOR_SATURATE);
            thurstor_values = thurstor_values.simd_clamp(minn, maxx) / THRUSTOR_SATURATE;

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
                    r2r::log_info!("reacting...", "{reacted}");
                    td.mission.react_to_object(&td, reacted);
                    r2r::log_info!("reacted", "{reacted}");
                    reacted += 1;
                }
            }
            td.map_objects_reacted.store(reacted, Ordering::Relaxed);
            scout_handle = tokio::spawn(scout(Arc::clone(&td)));
        }
    };

    // tokio::spawn(consume_map_sub(Arc::clone(&td)));
    tokio::spawn(consume_odometry_sub(Arc::clone(&td)));
    tokio::spawn(consume_new_objects(Arc::clone(&td)));
    tokio::spawn(go_to_goal(Arc::clone(&td)));

    let mut map_msg = MapMsg::default();

    map_msg.map_bounds.center.position = PointMsg {x: 0.0, y: 1.5, z: 2.0};
    map_msg.map_bounds.center.orientation = QuaternionMsg {w: 1.0, x: 0.0, y: 0.0, z: 0.0};
    map_msg.map_bounds.size = Vector3Msg { x: 1000.0, y: 1000.0, z: 3.0 };

    let mut gate_object_msg = MapObjectMsg::default();
    gate_object_msg.cls = ObjectCls::Gate as i32;
    gate_object_msg.bbox.center.position = PointMsg {x: 0.0, y: 1.5, z: 2.0};
    gate_object_msg.bbox.center.orientation = QuaternionMsg {w: 1.0, x: 0.0, y: 0.0, z: 0.0};
    gate_object_msg.bbox.size = Vector3Msg {x: 0.04, y: 3.0 + (2.0 * 0.04), z: 4.0};

    let mut cube_object_msg = MapObjectMsg::default();
    cube_object_msg.cls = ObjectCls::Cube as i32;
    cube_object_msg.bbox.center.position = PointMsg {x: 10.0, y: 1.0, z: 2.0};
    cube_object_msg.bbox.center.orientation = QuaternionMsg {w: 1.0, x: 0.0, y: 0.0, z: 0.0};
    cube_object_msg.bbox.size = Vector3Msg {x: 0.2, y: 0.2, z: 4.0};

    map_msg.objects.push(gate_object_msg);
    map_msg.objects.push(cube_object_msg);

    td.map.store(Arc::new(map_msg));
    td.new_objects.notify_one();

    r2r::log_info!("", "start spinning");
    loop {
        node.spin_once(Duration::from_millis(100));
    }
}
