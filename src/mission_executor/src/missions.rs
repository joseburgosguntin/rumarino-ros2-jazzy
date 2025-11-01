use nalgebra::{UnitQuaternion, Vector2, Vector3};

use crate::{MapObject, Mission, MissionExecutor, ObjectCls};

// this could hold some state if necessary
// like the some sort of queue if a sequence of reactions is necessary
pub(crate) struct PrecualifyMission {}

const FAR_ENOUGH: f64 = 0.25;
const OVERSHOOT: f64 = 0.50;

impl PrecualifyMission {
    pub(crate) fn new() -> Self {
        Self {}
    }

    fn go_around(&self, td: &MissionExecutor, idx: usize) {
        let object = MapObject::from(&td.map.load().objects[idx]);
        let object_pos = object.bbox.center.pos;
        let object_rot = object.bbox.center.rot;

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
        let initial_sub_pos = td.pose.load().pos;
        for (i, square_corner) in square_corners.iter().enumerate() {
            let sub_pose = td.pose.load();
            let rot_unit = UnitQuaternion::from_quaternion(object_rot);
            let actual_corner = square_corner.component_mul(&object.bbox.size.xy());
            let rotated_corner = (rot_unit * actual_corner.push(0.0)).xy();
            let pos2d = Vector2::new(object_pos.x, object_pos.y);
            let sub2d = Vector2::new(sub_pose.pos.x, sub_pose.pos.y);
            let corner_plus = pos2d + rotated_corner + rotated_corner.normalize() * DISTANCE_TO_CORNER;
            corner_pluss[i] = Vector3::new(corner_plus.x, corner_plus.y, sub_pose.pos.z);

            if (corner_plus - sub2d).norm() < starting_corner.norm() {
                starting_i = i;
                starting_corner = corner_plus;
            }
        }

        for i in 0..corner_pluss.len() {
            td.move_to(corner_pluss[starting_i + i % corner_pluss.len()]);
        }
        td.move_to(initial_sub_pos);
    }

    fn go_through(&self, td: &MissionExecutor, idx: usize) {
        let sub_pose = td.pose.load();
        let object = MapObject::from(&td.map.load().objects[idx]);

        let object_pos = object.bbox.center.pos;
        let object_pos_2d = object_pos.xy();
        let sub_pos_2d = sub_pose.pos.xy();

        let direction_2d = (object_pos_2d - sub_pos_2d).normalize();

        let before_2d = object_pos_2d - direction_2d * FAR_ENOUGH;
        let before = Vector3::new(before_2d.x, before_2d.y, object_pos.z);

        td.move_to(before);

        let overshoot_2d = object_pos_2d + direction_2d * OVERSHOOT;
        let overshoot = Vector3::new(overshoot_2d.x, overshoot_2d.y, object_pos.z);

        td.move_to(overshoot);
    }
}

impl Mission for PrecualifyMission {
    fn react_to_object(&self, td: &MissionExecutor, idx: usize) {
        let object = MapObject::from(&td.map.load().objects[idx]);
        match object.cls {
            ObjectCls::Rectangle | ObjectCls::Cube => self.go_around(td, idx),
            ObjectCls::Gate => self.go_through(td, idx),
            ObjectCls::Shark => (),
        }
    }
}
