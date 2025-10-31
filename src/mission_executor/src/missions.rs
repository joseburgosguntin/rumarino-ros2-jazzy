use nalgebra::{Quaternion, UnitQuaternion, Vector2, Vector3};

use crate::{Mission, MissionExecutor, ObjectCls};

// this could hold some state if necessary
// like the some sort of queue if a sequence of reactions is necessary
pub(crate) struct PrecualifyMission {}

const FAR_ENOUGH: f64 = 0.25;
const OVERSHOOT: f64 = 0.50;

impl PrecualifyMission {
    pub(crate) fn new() -> Self {
        Self {}
    }
    // TODO: these function are gonna need fuctions like: [move_to, look_at]
    //  these functions are expected to take controll and block the
    //  thread until they're done
    async fn go_around(&self, td: &MissionExecutor, idx: usize) {
        // check if having map lock outside of loop is ok
        // we might be starving map_sub
        let map = td.cached_map.lock().await;
        let object = &map.objects[idx];
        let object_pos = Vector3::new(
            object.bbox.center.position.x,
            object.bbox.center.position.y,
            object.bbox.center.position.z,
        );
        let object_rot = Quaternion::new(
            object.bbox.center.orientation.w,
            object.bbox.center.orientation.x,
            object.bbox.center.orientation.y,
            object.bbox.center.orientation.z,
        );

        // corners of a square centered in 0, with area 1
        let square_corners: [Vector2<f64>; 4] = [
            Vector2::new(0.5, 0.5),
            Vector2::new(0.5, -0.5),
            Vector2::new(-0.5, -0.5),
            Vector2::new(-0.5, 0.5),
        ];

        // absolute distance from any corner of the object
        const DISTANCE_TO_CORNER: f64 = 0.10; // 0.10 m == 10cm
        // a big number so that any corner is always closer
        const HUGE_NUMBER: f64 = 10000000.0;

        let mut corner_pluss = [Vector3::new(0.0, 0.0, 0.0); 4]; 
        let mut starting_corner = Vector2::new(HUGE_NUMBER, HUGE_NUMBER);
        let mut starting_i = usize::MAX;
        let initial_sub_pos = td.pose.lock().await.pos;
        for (i, square_corner) in square_corners.iter().enumerate() {
            let sub_pose = td.pose.lock().await;
            let rot_unit = UnitQuaternion::from_quaternion(object_rot);
            let rel_corner_3d = rot_unit * Vector3::new(square_corner.x, square_corner.y, 0.0);
            let rel_corner = Vector2::new(rel_corner_3d.x, rel_corner_3d.y);
            let pos2d = Vector2::new(object_pos.x, object_pos.y);
            let sub2d = Vector2::new(sub_pose.pos.x, sub_pose.pos.y);
            let corner_plus = pos2d + rel_corner + rel_corner.normalize() * DISTANCE_TO_CORNER;
            corner_pluss[i] = Vector3::new(corner_plus.x, corner_plus.y, sub_pose.pos.z);

            if (corner_plus - sub2d).norm() < starting_corner.norm() {
                starting_i = i;
                starting_corner = corner_plus;
            }
        }

        for i in 0..corner_pluss.len() {
            td.move_to(corner_pluss[starting_i + i % corner_pluss.len()]).await;
        }
        td.move_to(initial_sub_pos).await;
    }

    async fn go_through(&self, td: &MissionExecutor, idx: usize) {
        let sub_pose = td.pose.lock().await;
        let map = td.cached_map.lock().await;
        let object = &map.objects[idx];

        let object_pos = Vector3::new(
            object.bbox.center.position.x,
            object.bbox.center.position.y,
            object.bbox.center.position.z,
        );
        let object_pos_2d = Vector2::new(object_pos.x, object_pos.y);
        let sub_pos_2d = Vector2::new(sub_pose.pos.x, sub_pose.pos.y);

        let direction_2d = (object_pos_2d - sub_pos_2d).normalize();

        let before_2d = object_pos_2d - direction_2d * FAR_ENOUGH;
        let before = Vector3::new(before_2d.x, before_2d.y, object_pos.z);

        td.move_to(before).await;

        let overshoot_2d = object_pos_2d + direction_2d * OVERSHOOT;
        let overshoot = Vector3::new(overshoot_2d.x, overshoot_2d.y, object_pos.z);

        td.move_to(overshoot).await;
    }
}

#[async_trait::async_trait]
impl Mission for PrecualifyMission {
    async fn react_to_object(&self, td: &MissionExecutor, cls: ObjectCls, idx: usize) {
        match cls {
            ObjectCls::Rectangle | ObjectCls::Cube => self.go_around(td, idx).await,
            ObjectCls::Gate => self.go_through(td, idx).await,
            ObjectCls::Shark => (),
        }
    }
}
