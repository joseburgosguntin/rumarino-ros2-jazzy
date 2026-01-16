use crate::{BoundingBox3D, CLOSE_ENOUGH};
use nalgebra::{Isometry3, Point3, UnitQuaternion, UnitVector3, Vector3};
use parry3d_f64::query::{PointQuery, contact};
use parry3d_f64::shape::{Cuboid, Segment};

pub fn get_new_point(
    bbox: &BoundingBox3D,
    cuboid_pos: &Isometry3<f64>,
    segment: Segment,
    map_bounds: &BoundingBox3D,
) -> Option<Point3<f64>> {
    let half_extents = bbox.size * 0.5;

    let cuboid = Cuboid::new(half_extents);
    let mut target_point = segment.b;

    let pos2 = Isometry3::identity();

    let contact = contact(cuboid_pos, &cuboid, &pos2, &segment, 0.0).unwrap()?;

    if target_point.coords.metric_distance(&contact.point1.coords) < CLOSE_ENOUGH
        || target_point.coords.metric_distance(&contact.point2.coords) < CLOSE_ENOUGH
    {
        r2r::log_warn!(
            "mission_executor",
            "Point {:?} is too close to object. Moving target_point to avoid collision...",
            target_point
        );
        let offset = Point3::new(bbox.size.x, bbox.size.y, bbox.size.z);
        target_point -= offset.coords;
        todo!()
    }

    let midpoint: Point3<f64> = ((&contact.point2.coords + &contact.point1.coords) / 2.0).into();
    let seg_dir = segment.direction().unwrap();

    let points = get_points(
        &cuboid,
        cuboid_pos,
        &midpoint,
        &seg_dir,
        map_bounds,
    );

    let mut best_point = None;
    let mut smallest_dist = f64::MAX;

    for point in points {
        let dist = target_point.coords.metric_distance(&point.coords);

        if dist < smallest_dist {
            best_point = Some(point);
            smallest_dist = dist;
        }
    }
    best_point
}

fn get_points(
    cuboid: &Cuboid,
    cuboid_pos: &Isometry3<f64>,
    midpoint: &Point3<f64>,
    segment_dir: &UnitVector3<f64>,
    map_bounds: &BoundingBox3D,
) -> Vec<Point3<f64>> {
    let face_data: [(Vector3<f64>, f64); 6] = [
        (Vector3::x(), cuboid.half_extents.x),
        (-Vector3::x(), cuboid.half_extents.x),
        (Vector3::y(), cuboid.half_extents.y),
        (-Vector3::y(), cuboid.half_extents.y),
        (Vector3::z(), cuboid.half_extents.z),
        (-Vector3::z(), cuboid.half_extents.z),
    ];

    let local_midpoint = cuboid_pos.inverse_transform_vector(&midpoint.coords);
    let colliding_axis = get_dominant_axis(&segment_dir);

    let mut points = Vec::with_capacity(4);

    for (i, (face, extent)) in face_data.iter().enumerate() {
        let axis = i / 2;

        if axis == colliding_axis {
            continue;
        }

        let axis_value = match axis {
            0 => local_midpoint.x,
            1 => local_midpoint.y,
            2 => local_midpoint.z,
            _ => panic!(),
        };

        let dist = (extent - axis_value).abs();

        let world_face = cuboid_pos * face;

        let translation = world_face * (dist + 2.5);
        let point = midpoint + translation;

        let half_extents = map_bounds.size * 0.5;
        let bound_cuboid = Cuboid::new(half_extents);
        let pos = Isometry3::from_parts(map_bounds.center.pos.into(), UnitQuaternion::identity());
        if !bound_cuboid.contains_point(&pos, &point) {
            continue;
        }

        points.push(point);
    }

    points
}

fn get_dominant_axis(normal: &UnitVector3<f64>) -> usize {
    let normal_abs = normal.abs();

    if normal_abs.x >= normal_abs.y && normal_abs.x >= normal_abs.z {
        0
    } else if normal_abs.y >= normal_abs.z {
        1
    } else {
        2
    }
}
