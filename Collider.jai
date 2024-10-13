Collider :: struct {
    using shape:        Shape;
    position:           Vector2;
    rotation:           float;
    mass:               float;
    inertia:            float;
    inv_mass:           float;
    inv_inertia:        float;
    elasticity:         float;
    friction:           float;
    linear_velocity:    Vector2;
    angular_velocity:   float;
    force_accumulator:  Vector2;
    torque_accumulator: float;
    userdata:           *void;
}

make_circle :: (center: Vector2, radius: float, mass: float, elasticity: float, friction: float) -> Collider {
    collider: Collider;

    collider.shape       = make_circle_shape(radius);
    collider.position    = center;
    collider.mass        = mass;
    collider.inertia     = calculate_circle_inertia(radius, mass);
    collider.inv_mass    = ifx collider.mass    > 0.0 then 1.0 / collider.mass;
    collider.inv_inertia = ifx collider.inertia > 0.0 then 1.0 / collider.inertia;
    collider.elasticity  = elasticity;
    collider.friction    = friction;

    return collider;
}

make_polygon :: (vertices: [] Vector2, mass: float, elasticity: float, friction: float) -> Collider {
    collider: Collider;

    collider.shape       = make_polygon_shape(vertices);
    collider.position    = calculate_polygon_centroid(vertices);
    collider.mass        = mass;
    collider.inertia     = calculate_polygon_inertia(vertices, mass);
    collider.inv_mass    = ifx collider.mass    > 0.0 then 1.0 / collider.mass;
    collider.inv_inertia = ifx collider.inertia > 0.0 then 1.0 / collider.inertia;
    collider.elasticity  = elasticity;
    collider.friction    = friction;

    return collider;
}

make_rectangle :: (center: Vector2, width: float, height: float, mass: float, elasticity: float, friction: float) -> Collider {
    vertices := Vector2.[
        .{ center.x - width / 2.0, center.y - height / 2.0 },
        .{ center.x + width / 2.0, center.y - height / 2.0 },
        .{ center.x + width / 2.0, center.y + height / 2.0 },
        .{ center.x - width / 2.0, center.y + height / 2.0 },
    ];

    return make_polygon(vertices, mass, elasticity, friction);
}

is_static :: (using collider: Collider) -> bool {
    return inv_mass == 0.0;
}

is_point_inside :: (using collider: Collider, world_point: Vector2) -> bool {
    if kind == .CIRCLE {
        return is_point_inside_circle(position, circle.radius, world_point);
    } else {
        return is_point_inside_polygon(get_vertices(polygon), world_point);
    }
}

local_to_world :: (using collider: *Collider, local_point: Vector2) -> Vector2 {
	return rotate(local_point, rotation) + position;
}

world_to_local :: (using collider: *Collider, world_point: Vector2) -> Vector2 {
	return rotate(world_point - position, -rotation);
}

add_force :: (using collider: *Collider, force: Vector2) {
    force_accumulator += force;
}

add_torque :: (using collider: *Collider, torque: float) {
    torque_accumulator += torque;
}

add_force_at_point :: (using collider: *Collider, force: Vector2, world_point: Vector2) {
    torque := cross(world_point - position, force);

    force_accumulator  += force;
    torque_accumulator += torque;
}

apply_linear_impulse :: (using collider: *Collider, impulse: Vector2) {
    if is_static(collider) return;

    linear_velocity += impulse * inv_mass;
}

apply_angular_impulse :: (using collider: *Collider, impulse: float) {
    if is_static(collider) return;

    angular_velocity += impulse * inv_inertia;
}

apply_impulse_at_point :: (using collider: *Collider, impulse: Vector2, world_point: Vector2) {
    if is_static(collider) return;

    linear_impulse  := impulse;
    angular_impulse := cross(world_point, impulse);

    linear_velocity  += linear_impulse  * inv_mass;
    angular_velocity += angular_impulse * inv_inertia;
}

move :: (using collider: *Collider, delta: Vector2) {
    position += delta;

    if kind == .POLYGON {
        for * get_vertices(polygon) {
            it.* += delta;
        }
    }
}

rotate :: (using collider: *Collider, theta: float) {
    rotation += theta;

    if kind == .POLYGON {
        for * get_vertices(polygon) {
            it.* -= position;
            rotate(it, theta);
            it.* += position;
        }
    }
}

update :: (using collider: *Collider, dt: float) {
    if is_static(collider) return;

    // Semi-Implicit Euler integration

    linear_acceleration  := force_accumulator  * inv_mass;
    angular_acceleration := torque_accumulator * inv_inertia;

    linear_velocity  += linear_acceleration  * dt;
    angular_velocity += angular_acceleration * dt;

    LINEAR_CORRECTION  :: 0.999;
    ANGULAR_CORRECTION :: 0.995;

    linear_velocity  *= LINEAR_CORRECTION;
    angular_velocity *= ANGULAR_CORRECTION;

    move(collider, linear_velocity * dt);
    rotate(collider, angular_velocity * dt);

    force_accumulator  = .{};
    torque_accumulator = 0.0;
}
